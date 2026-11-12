/* Canonical explicit-world step pipeline. Body storage, contacts, solver
 * scratch, config, joints, and warm-start cache are owned per world. The GTK
 * callback (simulation_physics_loop.c) is now only an accumulator around this
 * function; there is no second stage sequence to keep in parity.
 * DESPOT threading: per-body phases, two-phase CCD, pair-local collision
 * detection, and independent island sweeps use the persistent pool
 * (MPE_THREADS, default 1). Shared-body contact preparation remains ordered
 * to preserve deterministic wake and manifold state. */
#include "physics_world.h"
#include "mpe_registry.h"
#include "mpe_loader.h"
#include "mpe_diag.h"
#include "mpe_platform.h"
#include "../physics/collision_mechanics.h"
#include "../physics/broadphase.h"
#include "../physics/constraint.h" /* MPE_FTC_067 */
#include "../physics/islands.h"
#include "../physics/depenetration.h"
#include "../scene/boundary.h"
#include "det_math.h" /* bit-identical damping factors on all IEEE targets */
#include "simd_math.h" /* DESPOT-2026-10-08 SIMD fast path (SSE2, scalar fallback) */
#include <pthread.h>
#include <stdatomic.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h> /* 2.6.5: sqrt/fabs/fmax on the hull spawn path */
#include <float.h> /* 2.6.5: FLT_MAX in the hull-hull support scan */
#include <alloca.h>
/* =====================================================================
 * PERSISTENT DETERMINISTIC THREAD POOL
 *
 * Threads are created ONCE (lazily, on first parallel call when
 * MPE_THREADS > 1) and sleep on a condition variable between ticks.
 * This eliminates the pthread_create/destroy overhead that previously
 * dominated every tick when running with multiple threads.
 *
 * Design rule: every thread writes to a DISJOINT set of bodies. Islands
 * are connected components (no shared bodies between islands), so
 * per-island solve + per-body integration are embarrassingly parallel.
 * Within one island the solve order is unchanged (Gauss-Seidel preserved).
 * Thread scheduling never affects results: disjoint writes + fixed order
 * = bitwise identical to serial, on every thread count.
 * ===================================================================== */
#define MPE_MAX_POOL_THREADS 8
static int s_pool_threads = 0;
static pthread_t s_pool_tids [MPE_MAX_POOL_THREADS];
static pthread_mutex_t s_pool_lock = PTHREAD_MUTEX_INITIALIZER;
/* A pool phase is global, so only one world may publish work to it at a
 * time. Keep this lock across publication and completion; s_pool_lock alone
 * protects each publication but would still let a second stepping thread
 * replace the first world's phase while its workers are running. */
static pthread_mutex_t s_pool_dispatch_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t s_pool_work_cond = PTHREAD_COND_INITIALIZER;
static pthread_cond_t s_pool_done_cond = PTHREAD_COND_INITIALIZER;
static volatile int s_pool_tasks_total = 0;
static volatile int s_pool_tasks_done = 0;
/* _Atomic for the C11-fallback path (MPE_PLATFORM atomic macros);
 * the GNU path treats it identically. */
static _Atomic int s_pool_tasks_remaining = 0;
static volatile int s_pool_shutdown = 0;
typedef void (*mpe_pool_func_t)(void *data, int start, int end);
static mpe_pool_func_t s_pool_func = NULL;
static void *s_pool_data = NULL;
static int s_pool_start = 0;
static int s_pool_end = 0;
static void *s_pool_worker (void *arg) {
    (void) arg;
    for (;;) {
        pthread_mutex_lock (&s_pool_lock);
        /* Wait until there are unclaimed units in the current phase, or
         * shutdown. Workers never exit mid-phase: when all units are
         * claimed they wait for the next broadcast (next mpe_pool_parallel_for
         * call resets done/total and broadcasts). */
        while (s_pool_tasks_done >= s_pool_tasks_total && !s_pool_shutdown) {
            pthread_cond_wait (&s_pool_work_cond, &s_pool_lock);
        }
        if (s_pool_shutdown) {
            pthread_mutex_unlock (&s_pool_lock);
            return NULL;
        }
        /* DESPOT-2026-10-19: claim AND snapshot the phase under the mutex.
         * Before this fix, s_pool_func/data/start/end were plain globals
         * written by the main thread outside the lock and read by workers
         * after unlocking — a worker descheduled between its claim and its
         * read could execute the NEXT phase's function on THIS phase's
         * chunk (UB: torn/mismatched dispatch; observed as a rare >180 s
         * hang of test_paranoia_determinism at MPE_THREADS=4). It also
         * checked `my_idx >= total` after unlocking, racing the reset. */
        int my_idx = s_pool_tasks_done++;
        if (my_idx >= s_pool_tasks_total) {
            /* Cannot normally happen now that claims are lock-serialized
             * against the phase reset; kept strictly safe: we were not
             * counted in `remaining`, so do not decrement it. */
            pthread_mutex_unlock (&s_pool_lock);
            continue;
        }
        mpe_pool_func_t func = s_pool_func;
        void *data = s_pool_data;
        int cs = s_pool_start + my_idx;
        int ce = cs + 1;
        if (ce > s_pool_end) { ce = s_pool_end; }
        pthread_mutex_unlock (&s_pool_lock);
        if (cs < ce && func) {
            func (data, cs, ce);
        }
        int remaining = MPE_ATOMIC_FETCH_SUB (&s_pool_tasks_remaining, 1, MPE_ATOMIC_ACQ_REL) - 1;
        if (remaining == 0) {
            pthread_mutex_lock (&s_pool_lock);
            pthread_cond_signal (&s_pool_done_cond);
            pthread_mutex_unlock (&s_pool_lock);
        }
    } return NULL;
}
static int mpe_thread_count (void) {
    const char *env = getenv ("MPE_THREADS");
    int threads = 1;
    if (env) {
        char *end = NULL;
        long tv = strtol (env, &end, 10);
        if ((end != env) && (*end == '\0') && (tv >= 1) && (tv <= 8)) { threads = (int) tv; }
        else if ((end != env) && (*end == '\0')) { threads = (tv < 1) ? 1 : 8; }
    }
    if (threads < 1) { threads = 1; }
    if (threads > 8) { threads = 8; }
    return threads;
}
static void mpe_pool_init (void) {
    /* DESPOT-2026-10-19: serialize lazy creation. Two threads stepping two
     * independent worlds could previously both enter here and create two
     * overlapping teams (or double-set s_pool_threads), which desynced the
     * worker wait counters. */
    pthread_mutex_lock (&s_pool_lock);
    if (s_pool_threads > 0) { pthread_mutex_unlock (&s_pool_lock); return; }
    s_pool_threads = mpe_thread_count ();
    if (s_pool_threads <= 1) { pthread_mutex_unlock (&s_pool_lock); return; }
    s_pool_shutdown = 0;
    s_pool_tasks_total = 0;
    s_pool_tasks_done = 0;
    MPE_ATOMIC_STORE_n (&s_pool_tasks_remaining, 0, MPE_ATOMIC_RELEASE);
    for (int t = 0; t < s_pool_threads; t++) {
        if (pthread_create (&s_pool_tids [t], NULL, s_pool_worker, NULL) != 0) {
            s_pool_tids [t] = 0;
            s_pool_threads = t;
            break;
        }
    }
    pthread_mutex_unlock (&s_pool_lock);
}
/* Parallel for-over-range: divides [start, end) into contiguous chunks,
 * one per thread, runs s_pool_func on each chunk, waits for completion.
 * Deterministic: same chunk boundaries every call. */
static void mpe_pool_parallel_for (int start, int end, mpe_pool_func_t func, void *data) {
    int n = end - start;
    if (n <= 0) { return; }
    int threads = s_pool_threads;
    if ((threads <= 1)) {
        func (data, start, end);
        return;
    }
    /* This pool has one process-wide phase descriptor. Serialize whole
     * dispatches, including the wait, so concurrent physics_world_step calls
     * cannot publish one world's callback/range over another's active work. */
    pthread_mutex_lock (&s_pool_dispatch_lock);
    /* DESPOT-2026-10-19: publish the whole phase under the mutex. The old
     * code wrote these globals unlocked, so workers could observe a mix of
     * the previous phase's func/range and the new phase's counters. */
    pthread_mutex_lock (&s_pool_lock);
    s_pool_func = func;
    s_pool_data = data;
    s_pool_start = start;
    s_pool_end = end;
    /* Total units (claimed under the lock). Remaining units (decremented
     * on completion — main thread waits for this to reach zero). */
    s_pool_tasks_total = n;
    s_pool_tasks_done = 0;
    MPE_ATOMIC_STORE_n (&s_pool_tasks_remaining, n, MPE_ATOMIC_RELEASE);
    pthread_cond_broadcast (&s_pool_work_cond);
    while (MPE_ATOMIC_LOAD_n (&s_pool_tasks_remaining, MPE_ATOMIC_ACQUIRE) > 0) {
        pthread_cond_wait (&s_pool_done_cond, &s_pool_lock);
    }
    pthread_mutex_unlock (&s_pool_lock);
    pthread_mutex_unlock (&s_pool_dispatch_lock);
}
/* ---- Parallel slice helpers (operate on disjoint body ranges) ---------- */
typedef struct {
    physics_world *world;
    int start;
    int end;
} mpe_slice_t;
static void mpe_sanitize_range (void *data, int start, int end) {
    physics_world *w = (physics_world *) data;
    for (int i = start; i < end; i++) {
        rigidbody_sanitize (&w -> bodies [i]);
    }
}
static void mpe_parallel_sanitize (physics_world *world) {
    int n = world -> body_count;
    if (n <= 0) { return; }
    mpe_pool_init ();
    int threads = s_pool_threads;
    if (threads <= 1) {
        for (int i = 0; i < n; i++) {
            rigidbody_sanitize (&world -> bodies [i]);
        } return;
    }
    mpe_pool_parallel_for (0, n, mpe_sanitize_range, world);
}
/* Forward declaration: defined later in this TU (solver stage dispatch). */
static float mpe_step_resolve (physics_world *world, collision_data *m, float dt, bool friction_only, int iter,
                               const mpe_config_t *cfg);
/* ---- Parallel island solve -------------------------------------------
 *
 * Each task solves ONE island's manifolds (2 visits each, preserving the
 * manifold sort order) plus that island's active constraints. Writes only
 * to bodies in that island, so threads are disjoint.
 */
/* Run one solver "sweep" across all awake islands in parallel: each island's
 * manifolds are solved by one thread, in sort order. Joined before return
 * so the next serial phase (constraint_solve_all) sees consistent state. */
typedef struct {
    physics_world *world;
    int island_id;
    int *manifold_indices;
    int manifold_count;
    float dt;
    int iter;
    bool friction_only;
} mpe_island_task_t;
static void mpe_solve_island_range (void *data, int start, int end) {
    mpe_island_task_t *tasks = (mpe_island_task_t *) data;
    for (int i = start; i < end; i++) {
        mpe_island_task_t *t = &tasks [i];
        physics_world *w = t -> world;
        const mpe_config_t *cfg = mpe_world_cfg (w);
        for (int mi = 0; mi < t -> manifold_count; mi++) {
            int m = t -> manifold_indices [mi];
            if (!w -> manifold_awake [m]) { continue; }
            mpe_step_resolve (w, &w -> manifolds [m], t -> dt, t -> friction_only, t -> iter, cfg);
            mpe_step_resolve (w, &w -> manifolds [m], t -> dt, t -> friction_only, t -> iter + 1, cfg);
        }
    }
}
static void mpe_parallel_solve_sweep (physics_world *world, int manifold_count, float dt, int iter,
                                      const mpe_config_t *cfg, bool friction_only) {
    int threads = s_pool_threads;
    int island_total = islands_count (world);
    if ((threads <= 1) || (island_total <= 1) || (manifold_count <= 0)) {
        /* Serial fallback: identical order, identical results. */
        for (int o = 0; o < manifold_count; o++) {
            int m = world -> manifold_order [o];
            if (!world -> manifold_awake [m]) { continue; }
            mpe_step_resolve (world, &world -> manifolds [m], dt, friction_only, iter, cfg);
            mpe_step_resolve (world, &world -> manifolds [m], dt, friction_only, iter + 1, cfg);
        } return;
    }
    mpe_pool_init ();
    threads = s_pool_threads;
    if ((threads <= 1) || (island_total <= 1) || (manifold_count <= 0)) {
        for (int o = 0; o < manifold_count; o++) {
            int m = world -> manifold_order [o];
            if (!world -> manifold_awake [m]) { continue; }
            mpe_step_resolve (world, &world -> manifolds [m], dt, friction_only, iter, cfg);
            mpe_step_resolve (world, &world -> manifolds [m], dt, friction_only, iter + 1, cfg);
        } return;
    }
    /* Assign each manifold to its island (via body A). */
    int *manifold_island = (int *) alloca ((size_t) manifold_count * sizeof (int));
    for (int m = 0; m < manifold_count; m++) {
        rigidbody *ma = world -> manifolds [m].object_a;
        int isl = islands_body_island (world, ma);
        manifold_island [m] = (isl < 0) ? -1 : isl;
    }
    /* Count manifolds per island. */
    int *isl_manifold_count = (int *) alloca ((size_t) island_total * sizeof (int));
    memset (isl_manifold_count, 0, (size_t) island_total * sizeof (int));
    for (int m = 0; m < manifold_count; m++) {
        int isl = manifold_island [m];
        if (isl >= 0) { isl_manifold_count [isl]++; }
    }
    /* Build offset table (prefix sum) for scatter. */
    int *isl_manifold_off = (int *) alloca ((size_t) (island_total + 1) * sizeof (int));
    isl_manifold_off [0] = 0;
    for (int i = 0; i < island_total; i++) { isl_manifold_off [i + 1] = isl_manifold_off [i] + isl_manifold_count [i]; }
    /* Scatter manifold indices into per-island buckets, preserving sort order. */
    int *isl_manifold_idx = (int *) alloca ((size_t) manifold_count * sizeof (int));
    int *isl_cursor = (int *) alloca ((size_t) island_total * sizeof (int));
    memcpy (isl_cursor, isl_manifold_off, (size_t) island_total * sizeof (int));
    for (int o = 0; o < manifold_count; o++) {
        int m = world -> manifold_order [o];
        int isl = manifold_island [m];
        if (isl >= 0) { isl_manifold_idx [isl_cursor [isl]++] = m; }
    }
    /* Build active-island list (awake islands with manifolds) in batches of
     * MPE_MAX_POOL_THREADS. DESPOT-2026-10-19: the old code stopped filling
     * at MPE_MAX_POOL_THREADS and solved only that prefix — with more than
     * 8 independent awake islands (routine: >8 separated bodies on a floor)
     * every island past the 8th was silently never solved, its bodies
     * sinking until a lower-index island slept. It also read tasks[0]
     * uninitialized when there were zero awake islands with manifolds
     * (wild mpe_island_task_t on the stack -> crash in a settled scene).
     * Batches keep the disjoint-write property (islands share no bodies). */
    mpe_island_task_t tasks [MPE_MAX_POOL_THREADS];
    int num_active = 0;
    for (int isl = 0; isl < island_total; isl++) {
        if (!world -> island_awake_flags [isl]) { continue; }
        if (isl_manifold_count [isl] <= 0) { continue; }
        tasks [num_active].world = world;
        tasks [num_active].island_id = isl;
        tasks [num_active].manifold_indices = &isl_manifold_idx [isl_manifold_off [isl]];
        tasks [num_active].manifold_count = isl_manifold_count [isl];
        tasks [num_active].dt = dt;
        tasks [num_active].iter = iter;
        tasks [num_active].friction_only = friction_only;
        num_active++;
        if (num_active == MPE_MAX_POOL_THREADS) {
            mpe_pool_parallel_for (0, num_active, mpe_solve_island_range, tasks);
            num_active = 0;
        }
    }
    if (num_active == 0) { return; }
    if (num_active == 1) {
        /* Single island - solve serially. friction_only is passed through:
         * the old hardcoded `false` ran full normal solves on this path
         * where the multi-island/serial callers requested friction-only
         * relaxation, over-solving contacts on one shape of scene. */
        for (int k = 0; k < tasks [0].manifold_count; k++) {
            int m = tasks [0].manifold_indices [k];
            if (!world -> manifold_awake [m]) { continue; }
            mpe_step_resolve (world, &world -> manifolds [m], dt, friction_only, iter, mpe_world_cfg (world));
            mpe_step_resolve (world, &world -> manifolds [m], dt, friction_only, iter + 1, mpe_world_cfg (world));
        } return;
    }
    mpe_pool_parallel_for (0, num_active, mpe_solve_island_range, tasks);
}
/* Parallel per-body integration (same disjoint-write pattern as sanitize). */
typedef struct {
    physics_world *world;
    int start;
    int end;
    float dt;
    const mpe_config_t *cfg;
    unsigned char *joint_membership;
} mpe_integrate_slice_t;
static void mpe_integrate_range (void *data, int start, int end) {
    mpe_integrate_slice_t *s = (mpe_integrate_slice_t *) data;
    for (int i = start; i < end; i++) {
        rigidbody *ib = &s -> world -> bodies [i];
        float step_dt = s -> dt;
        if ((s -> world -> ccd_time_remaining) && (s -> world -> ccd_time_remaining [i] < step_dt) &&
            (s -> world -> ccd_time_remaining [i] > 0.0f)) { step_dt = s -> world -> ccd_time_remaining [i]; }
        bool has_joint = (i < mpe_max_bodies) ? (s -> joint_membership [i] != 0) : false;
        bool contact_free = ((s -> world -> has_contact) ? (s -> world -> has_contact [i] == 0) : false);
        bool have_v0 = (s -> world -> tick_v0 && (s -> world -> tick_v0_capacity >= s -> world -> body_count));
        /* External force => constrained path: the analytic free-flight
         * solution covers gravity+drag only and would drop the force. */
        bool free_flight = contact_free && !has_joint && (ib -> ext_force_this_tick == 0);
        if (free_flight && !have_v0) { free_flight = false; }
        if (free_flight && have_v0) { ib -> velocity = s -> world -> tick_v0 [i]; }
        rb_integrate_position_exact (ib, step_dt, s -> cfg, free_flight);
        ib -> ext_force_this_tick = 0; /* consumed for this tick */
        rigidbody_sanitize (ib);
    }
}
static void mpe_parallel_integrate (physics_world *world, float dt, const mpe_config_t *cfg,
                                    unsigned char *joint_membership) {
    int n = world -> body_count;
    if (n <= 0) { return; }
    mpe_pool_init ();
    int threads = s_pool_threads;
    if ((threads <= 1) || (n < 64)) {
        for (int i = 0; i < n; i++) {
            rigidbody *ib = &world -> bodies [i];
            float step_dt = dt;
            if ((world -> ccd_time_remaining) && (world -> ccd_time_remaining [i] < step_dt) &&
                (world -> ccd_time_remaining [i] > 0.0f)) { step_dt = world -> ccd_time_remaining [i]; }
            bool has_joint = (i < mpe_max_bodies) ? (joint_membership [i] != 0) : false;
            bool contact_free = ((world -> has_contact) ? (world -> has_contact [i] == 0) : false);
            bool have_v0 = (world -> tick_v0 && (world -> tick_v0_capacity >= world -> body_count));
            bool free_flight = contact_free && !has_joint && (ib -> ext_force_this_tick == 0);
            if (free_flight && !have_v0) { free_flight = false; }
            if (free_flight && have_v0) { ib -> velocity = world -> tick_v0 [i]; }
            rb_integrate_position_exact (ib, step_dt, cfg, free_flight);
            ib -> ext_force_this_tick = 0; /* consumed for this tick */
            rigidbody_sanitize (ib);
        } return;
    }
    mpe_integrate_slice_t slice;
    slice.world = world;
    slice.dt = dt;
    slice.cfg = cfg;
    slice.joint_membership = joint_membership;
    mpe_pool_parallel_for (0, n, mpe_integrate_range, &slice);
}
/* ---- Parallel CCD clamp ------------------------------------------------
 * Symmetric two-phase sweep. Phase 1 computes every TOI against the
 * tick-start snapshot (bodies are read-only), then the pool returns (a
 * barrier), then phase 2 applies every clamp (disjoint writes to
 * position/velocity/orientation). Without the barrier, one thread's clamp
 * (phase 2) raced another thread's TOI read (phase 1), and the
 * interleaving decided which body stopped — the paranoia_ccd flake, and a
 * tunneling hole for the body that was not clamped. Bitwise identical to
 * serial at any thread count. */
typedef struct {
    physics_world *world;
    float dt;
    const mpe_config_t *cfg;
} mpe_ccd_slice_t;
static void mpe_ccd_toi_range (void *data, int start, int end) {
    mpe_ccd_slice_t *s = (mpe_ccd_slice_t *) data;
    collision_ccd_toi_range_world (s -> world, start, end,
                                   s -> dt, s -> cfg,
                                   s -> world -> ccd_best_tois, s -> world -> ccd_hit_flags);
}
static void mpe_ccd_apply_range (void *data, int start, int end) {
    mpe_ccd_slice_t *s = (mpe_ccd_slice_t *) data;
    collision_ccd_apply_range (s -> world -> bodies, s -> world -> body_count, start, end,
                               s -> dt, s -> cfg,
                               s -> world -> ccd_time_remaining, s -> world -> ccd_best_tois,
                               s -> world -> ccd_hit_flags);
}
static void mpe_parallel_ccd_sweep (physics_world *world, float dt, const mpe_config_t *cfg) {
    int n = world -> body_count;
    if (n <= 0) { return; }
    if (!world -> ccd_time_remaining || !world -> ccd_best_tois || !world -> ccd_hit_flags) {
        collision_ccd_sweep_clamp_world (world, dt);
        return;
    }
    mpe_pool_init ();
    int threads = s_pool_threads;
    if (threads <= 1) {
        collision_ccd_sweep_clamp_world (world, dt);
        return;
    }
    mpe_ccd_slice_t slice;
    slice.world = world;
    slice.dt = dt;
    slice.cfg = cfg;
    mpe_pool_parallel_for (0, n, mpe_ccd_toi_range, &slice);
    mpe_pool_parallel_for (0, n, mpe_ccd_apply_range, &slice);
} /* Apply the stateful half of contact processing in broadphase pair order.
 * Detection can run in parallel into disjoint manifold slots, but wake state,
 * contact flags, and inverse inertia are body-owned and may be shared by
 * several pairs. */
static void physics_world_prepare_detected_pair (physics_world *world, int index_a, int index_b,
                                                  float dt, collision_data *source,
                                                  collision_data *manifold) {
    if (!world || !source || !manifold || index_a < 0 || index_a >= world->body_count ||
        index_b < 0 || index_b >= world->body_count) return;
    rigidbody *body_a = &world->bodies[index_a];
    rigidbody *body_b = &world->bodies[index_b];
    bool a_was_sleeping = body_a->is_sleeping;
    bool b_was_sleeping = body_b->is_sleeping;
    if (!a_was_sleeping || !b_was_sleeping) {
        /* MIG WP2 2.4/2.5: novelty is part-scoped. A composite that rolls
         * a new part onto a sleeper is a new contact edge even when another
         * part of the same body pair was already cached. */
        uint8_t edge_part_a = (source->contact_count > 0) ? source->contacts[0].part_a : 0u;
        uint8_t edge_part_b = (source->contact_count > 0) ? source->contacts[0].part_b : 0u;
        bool new_edge = !contact_cache_has_part_pair (world, body_a->object_id, edge_part_a,
                                                      body_b->object_id, edge_part_b);
        if (a_was_sleeping && !body_b->static_state && new_edge) rigidbody_wake (body_a);
        if (b_was_sleeping && !body_a->static_state && new_edge) rigidbody_wake (body_b);
        const mpe_config_t *cfg = mpe_world_cfg (world);
        float wake_lin_sq = cfg->sleep.wake_linear_thresh_sq;
        float wake_ang_sq = cfg->sleep.wake_angular_thresh_sq;
        bool a_fast = !a_was_sleeping &&
            (vector3_length_squared (body_a->velocity) > wake_lin_sq ||
             vector3_length_squared (body_a->angular_velocity) > wake_ang_sq);
        bool b_fast = !b_was_sleeping &&
            (vector3_length_squared (body_b->velocity) > wake_lin_sq ||
             vector3_length_squared (body_b->angular_velocity) > wake_ang_sq);
        if (a_was_sleeping && !body_b->static_state && b_fast) rigidbody_wake (body_a);
        if (b_was_sleeping && !body_a->static_state && a_fast) rigidbody_wake (body_b);
        float deepest = 0.0f;
        for (int i = 0; i < source->contact_count; i++)
            if (source->contacts[i].penetration > deepest) deepest = source->contacts[i].penetration;
        if (deepest > cfg->depenetration.wake_depth_thresh) {
            if (a_was_sleeping) rigidbody_wake (body_a);
            if (b_was_sleeping) rigidbody_wake (body_b);
        }
        collision_prepare_solver (world, source, manifold, dt);
        if (world->has_contact) {
            world->has_contact[index_a] = 1;
            world->has_contact[index_b] = 1;
        }
    }
}

/* ---- Parallel narrowphase --------------------------------------------
 * Per pair, pre-assigned manifold slot (slot == pair index). Workers only
 * write detection results to disjoint slots. Body wake, contact flags, and
 * solver preparation run serially after the barrier in pair order to avoid
 * races when several contacts share a body. */
typedef struct {
    physics_world *world;
    float dt;
    const mpe_config_t *cfg;
} mpe_narrowphase_slice_t;
static void mpe_narrowphase_range (void *data, int start, int end) {
    mpe_narrowphase_slice_t *s = (mpe_narrowphase_slice_t *) data;
    physics_world *w = s -> world;
    for (int p = start; p < end; p++) {
        /* Zero the slot first so non-collided pairs have contact_count=0
         * (slot may contain garbage from a previous tick — malloc'd, not
         * calloc'd). Collided pairs are overwritten by prepare_solver. */
        w -> manifolds [p] = (collision_data) {0};
        w -> manifold_awake [p] = 0;
        int index_a = w -> pair_buffer [p].object_index_a;
        int index_b = w -> pair_buffer [p].object_index_b;
        if ((index_a < 0) || (index_a >= w -> body_count) || (index_b < 0) || (index_b >= w -> body_count)) {
            w -> pair_skipped [p] = 1;
            continue;
        }
        rigidbody *body_a = &w -> bodies [index_a];
        rigidbody *body_b = &w -> bodies [index_b];
        if ((body_a -> is_sleeping) && (body_b -> is_sleeping)) {
            w -> pair_skipped [p] = 1;
            continue;
        }
        /* MIG WP2 2.4: one body pair can emit one manifold per touching
         * part pair, so composite pairs cannot use the one-slot-per-pair
         * parallel mapping. They are detected serially after the barrier
         * (physics_world_process_pair appends to the manifold tail); the
         * ghost scratch is thread-local either way. */
        if ((body_a -> type == object_composite) || (body_b -> type == object_composite)) {
            w -> pair_skipped [p] = 0;
            continue;
        }
        collision_data narrowphase_collision = {0};
        bool collided = mpe_shape_dispatch (w, body_a, body_b, &narrowphase_collision);
        if (!collided) {
            w -> manifold_awake [p] = 0;
            continue;
        }
        w->manifolds[p] = narrowphase_collision;
        w -> manifold_awake [p] = 1;
    }
}
/* Grow the manifold arrays (all four, atomically per step) until at least
 * `needed` slots exist, or the hard ceiling is reached. The step path must
 * call this BEFORE it memsets/processes pair_count slots: the previous
 * single grow attempt ran inside the parallel narrowphase, after the
 * caller had already memset pair_count slots regardless of capacity. */
static int physics_world_ensure_manifold_capacity (physics_world *world, int needed) {
    if (!world) { return -1; }
    while (world -> manifold_capacity < needed) {
        if (physics_world_grow_manifolds (world) != 0) { return -1; }
    }
    return 0;
}
static int mpe_parallel_narrowphase (physics_world *world, int pair_count, int slot_count, float dt,
                                     const mpe_config_t *cfg) {
    if ((pair_count <= 0) || (slot_count <= 0) || (!world) || (!world -> pair_buffer) || (!world -> manifolds) ||
        (!world -> manifold_awake)) {
        return slot_count;
    }
    if (slot_count > world -> manifold_capacity) { slot_count = world -> manifold_capacity; }
    mpe_pool_init ();
    int threads = s_pool_threads;
    /* The parallel path writes each pair's manifold at slot == pair index,
     * so it is only valid when one zeroed slot exists per pair. When the
     * pair count exceeds the growable capacity, fall back to the serial
     * compactor: it grows per collision, counts every collided overflow
     * pair, and never writes past capacity. */
    if ((threads <= 1) || (slot_count < pair_count)) {
        int manifold_count = 0;
        for (int p = 0; p < pair_count; p++) {
            int index_a = world -> pair_buffer [p].object_index_a;
            int index_b = world -> pair_buffer [p].object_index_b;
            if ((index_a < 0) || (index_a >= world -> body_count) || (index_b < 0) || (index_b >= world -> body_count)) { continue; }
            rigidbody *body_a = &world -> bodies [index_a];
            rigidbody *body_b = &world -> bodies [index_b];
            if ((body_a -> is_sleeping) && (body_b -> is_sleeping)) { continue; }
            physics_world_process_pair (world, index_a, index_b, dt, &manifold_count);
        }
        /* The serial compactor fills [0, k); the caller zeroed every slot
         * up to slot_count, so slots [k, slot_count) stay empty and are
         * awake-skipped exactly like non-collided parallel slots. A
         * composite pair may append more manifolds than the pair count, so
         * return the larger of the compacted count and the zeroed span. */
        return (manifold_count > slot_count) ? manifold_count : slot_count;
    }
    mpe_narrowphase_slice_t slice;
    slice.world = world;
    slice.dt = dt;
    slice.cfg = cfg;
    mpe_pool_parallel_for (0, pair_count, mpe_narrowphase_range, &slice);
    for (int p = 0; p < pair_count; p++) {
        if (!world->manifold_awake[p]) continue;
        int ia = world->pair_buffer[p].object_index_a;
        int ib = world->pair_buffer[p].object_index_b;
        if (ia < 0 || ia >= world->body_count || ib < 0 || ib >= world->body_count) continue;
        collision_data source = world->manifolds[p];
        physics_world_prepare_detected_pair (world, ia, ib, dt, &source, &world->manifolds[p]);
    }
    /* Composite part-pair expansion: serial because each pair may append
     * several manifolds. Slots [pair_count, total) are the expansion tail;
     * physics_world_process_pair grows the arrays on demand and counts any
     * overflow pair. Primitive pairs already prepared above are untouched. */
    int total_manifolds = slot_count;
    for (int p = 0; p < pair_count; p++) {
        int ia = world->pair_buffer[p].object_index_a;
        int ib = world->pair_buffer[p].object_index_b;
        if ((ia < 0) || (ia >= world->body_count) || (ib < 0) || (ib >= world->body_count)) { continue; }
        rigidbody *body_a = &world->bodies[ia];
        rigidbody *body_b = &world->bodies[ib];
        if ((body_a->type != object_composite) && (body_b->type != object_composite)) { continue; }
        if (world->pair_skipped[p]) { continue; }
        physics_world_process_pair (world, ia, ib, dt, &total_manifolds);
        world->pair_skipped[p] = 0;
    }
    return total_manifolds;
}
/* Canonical spring pass is weakly linked so spring-less headless test
 * binaries (which omit physics/spring_joint.c for its GL dependency)
 * still link; the GUI engine and TUI link it and get real forces. */
MPE_WEAK void mpe_springs_apply (physics_world *world, float dt);
#include "../config/mpe_config.h"
#include "../config/mpe_constants.h"
#include <stdlib.h>
#include <math.h>
#include <string.h> /* MPE_FTC_076a */
/* Live-world registry: init registers, cleanup removes. Loader and
 * registry-unregister paths iterate it to detach/reset every world that
 * references dying code BEFORE dlclose or slot reuse. Fixed cap of 64
 * live worlds (headless tests + GUI + TUI stay far below; overflow
 * refuses init-time registration but stepping still works — the world
 * just can't be auto-purged, documented in the header). */
static physics_world *s_live_worlds [MPE_MAX_LIVE_WORLDS];
static int s_live_count = 0;
static pthread_mutex_t s_live_lock = PTHREAD_MUTEX_INITIALIZER;
static void live_world_add (physics_world *world) {
    pthread_mutex_lock (&s_live_lock);
    for (int i = 0; i < s_live_count; i++) {
        if (s_live_worlds [i] == world) {
            pthread_mutex_unlock (&s_live_lock);
            return;
        }
    }
    if (s_live_count < MPE_MAX_LIVE_WORLDS)
        s_live_worlds [s_live_count++] = world;
    else {
        /* DESPOT-2026-10-07 P1-2: overflow used to silently skip registration
         * while stepping continued — the world became invisible to
         * auto-purge and unload required manual detach. Loud now. */
        fprintf (stderr, "[mpe] LIVE-WORLD cap %d reached; world %p steps but is invisible to auto-purge (detach manually before unload)\n",
                 MPE_MAX_LIVE_WORLDS, (const void *) world);
        fflush (stderr);
    } pthread_mutex_unlock (&s_live_lock);
} /* `physics_world` is commonly a stack object whose bytes are indeterminate
 * before its first init. Detect re-init by address in the registry instead
 * of probing fields in that uninitialized object. */
static bool live_world_contains (const physics_world *world) {
    bool found = false;
    pthread_mutex_lock (&s_live_lock);
    for (int i = 0; i < s_live_count; i++) {
        if (s_live_worlds [i] == world) {
            found = true;
            break;
        }
    } pthread_mutex_unlock (&s_live_lock);
    return found;
}
static void live_world_remove (physics_world *world) {
    pthread_mutex_lock (&s_live_lock);
    for (int i = 0; i < s_live_count; i++) {
        if (s_live_worlds [i] == world) {
            for (int j = i; j + 1 < s_live_count; j++)
                s_live_worlds [j] = s_live_worlds [j + 1];
            s_live_count--;
            break;
        }
    } pthread_mutex_unlock (&s_live_lock);
}
int physics_world_live_list (physics_world **out, int cap) {
    if (!out || cap <= 0)
        return 0;
    pthread_mutex_lock (&s_live_lock);
    int n = (s_live_count < cap) ? s_live_count : cap;
    for (int i = 0; i < n; i++)
        out [i] = s_live_worlds [i];
    int total = s_live_count;
    pthread_mutex_unlock (&s_live_lock);
    (void) total;
    return n;
} /* Reset stage slots aliasing dead registry entries (called by registry
 * unregister paths while the old code is still mapped). Foreign stage
 * state is owned by the module (header: never freed by the world), so the
 * module's stage_detach hook runs first (resolved by code address — stage
 * slots keep the iface, not the desc), then the pointers are dropped. */
void physics_world_forget_stage_pointers (const mpe_broadphase_if_t *bi, const mpe_solver_if_t *si) {
    physics_world *ws [MPE_MAX_LIVE_WORLDS];
    int n = physics_world_live_list (ws, MPE_MAX_LIVE_WORLDS);
    for (int i = 0; i < n; i++) {
        if (bi && ws [i] -> broadphase_if == bi) {
            if (bi -> generate)
                mpe_loader_call_stage_detach_for_fn ((const void *) bi -> generate, ws [i]);
            ws [i] -> broadphase_if = NULL;
            ws [i] -> broadphase_state = NULL;
        }
        if (si && ws [i] -> solver_if == si) {
            if (si -> resolve)
                mpe_loader_call_stage_detach_for_fn ((const void *) si -> resolve, ws [i]);
            else if (si -> poisson)
                mpe_loader_call_stage_detach_for_fn ((const void *) si -> poisson, ws [i]);
            else if (si -> rolling)
                mpe_loader_call_stage_detach_for_fn ((const void *) si -> rolling, ws [i]);
            else if (si -> split)
                mpe_loader_call_stage_detach_for_fn ((const void *) si -> split, ws [i]);
            ws [i] -> solver_if = NULL;
            ws [i] -> solver_state = NULL;
        }
    }
} /* Detach a named tick module from every live world (registry unregister
 * path). Detach hooks run while the .so is still mapped. */
void physics_world_detach_module_everywhere (const char *name) {
    if (!name)
        return;
    physics_world *ws [MPE_MAX_LIVE_WORLDS];
    int n = physics_world_live_list (ws, MPE_MAX_LIVE_WORLDS);
    for (int i = 0; i < n; i++)
        physics_world_detach_module (ws [i], name);
} /* NOTE: no file-scope simulation state in this TU. The application
 * primary lives in core/mpe_primary.c; all stepping takes explicit
 * worlds (see physics_world_get_primary contract in the header). */
/* Helper: malloc-or-NULL (leaves member NULL on failure; users degrade
 * gracefully — add fns reject NULL bodies, pairing returns 0, solves skip).
 * Worlds that fail init are safely unusable, never crash-prone. */
static void physics_world_free_ptr (void **slot) {
    if (slot && *slot) {
        free (*slot);
        *slot = NULL;
    }
}
void physics_world_init (physics_world *world) {
    if (!world) { return; }
    /* MPE_FTC_076a (upheld DESPOT-2026-09-26, amended DESPOT-2026-10-07 P1-2):
     * live_world_contains compares only the ADDRESS, so it is safe on
     * uninitialized stack bytes. Check BEFORE memset: if this address is
     * already live, cleanup FIRST (valid contents) then zero. The old
     * zero-first order made cleanup a no-op (tick_modules already wiped)
     * and orphaned loader attachments forever (unload -2 forever).
     * Fresh stack garbage that is NOT live takes the memset path; no field
     * is ever probed on garbage. Re-init without cleanup still requires
     * caller discipline, but re-init WITH a live entry no longer leaks. */
    bool was_live = live_world_contains (world);
    if (was_live) { physics_world_cleanup (world); }
    memset (world, 0, sizeof (physics_world));
    det_pin_fp_state ();
    /* Static plane body (floor at y=0) - disabled by default. */
    world -> static_plane_enabled = false;
    /* Initialize static plane body (floor at y=0) with neutral restitution. */
    rigidbody_initialisation_sphere (&world -> static_plane_body, 1.0f, 0.0f, (vector3) {0.0f, 0.0f, 0.0f});
    world -> static_plane_body.static_state = true;
    world -> static_plane_body.inverse_mass = 0.0f;
    world -> static_plane_body.inverse_inertia_tensor_local = (math3) {{{0}}};
    world -> static_plane_body.inverse_inertia_system = (math3) {{{0}}};
    world -> static_plane_body.restitution = 1.0f;
    world -> static_plane_body.object_id = 0xFFFFFFFFu;
    world -> static_plane_body.object_generation = 1;
    world -> static_plane_body.body_index = -1; /* Not in body array */
    world -> static_plane_body.friction_static = g_cfg.world.floor_friction_s;
    world -> static_plane_body.friction_kinetic = g_cfg.world.floor_friction_k;
    /* Phase-1: default-bind global config (back-compat). Caller may
     * rebind via physics_world_set_config() for isolation. */
    world -> cfg = &g_cfg;
    mpe_register_builtins ();
    live_world_add (world);
    /* DESPOT-2026-10-08 P2 joint-pool hardening: clear is_active inside init
     * (one loop, zero behaviour change for paired callers that also call
     * joint_init_pool/constraint_pool_init). init-alone callers no longer
     * get phantom joints from stack garbage when the memset path is ever
     * bypassed by a future early-return. */
    for (int ji = 0; ji < mpe_max_joints; ji++) {
        world -> spring_joints [ji].is_active = false;
        world -> revolute_constraints [ji].is_active = false;
    } world -> spring_joint_count = 0;
    world -> revolute_constraint_count = 0;
    /* Growable pools: start small, double on demand (see growers below).
     * body_capacity tracks the live allocation (not the ceiling). */
    if (!world -> bodies) {
        world -> bodies = (rigidbody *) malloc ((size_t) mpe_initial_bodies * sizeof (rigidbody));
        world -> body_capacity = world -> bodies ? mpe_initial_bodies : 0;
    }
    if (!world -> world_contact_cache) {
        world -> world_contact_cache =
            (cached_contact *) malloc ((size_t) mpe_initial_contacts * sizeof (cached_contact)); /* MFS_131A */
        world -> world_contact_cache_capacity = world -> world_contact_cache ? mpe_initial_contacts : 0;
    }
    /* Warm-start hash heads: heap, not stack (worlds are often stack-local;
     * an inline 16 KB array risks overflow next to big frames). */
    if (!world -> contact_hash_head) {
        world -> contact_hash_head = (int32_t *) malloc ((size_t) 4096 * sizeof (int32_t));
        if (world -> contact_hash_head) {
            for (int i = 0; i < 4096; i++) {
                world -> contact_hash_head [i] = -1;
            }
        }
    }
    /* Solver scratch (heap; worlds are frequently stack-local). Sizes are
     * compile-time caps from mpe_constants.h; pages fault on first touch. */
    if (!world -> broadphase) {
        world -> broadphase = (broadphase_workspace *) calloc (1, sizeof (broadphase_workspace));
        if (world -> broadphase) { world -> broadphase -> current_cell_size = 5.0f; }
    }
    if (!world -> pair_buffer) { world -> pair_buffer = (broadphase_pair *) malloc ((size_t) mpe_max_broadphase_pairs * sizeof (broadphase_pair)); }
    if (!world -> manifolds) {
        world -> manifold_capacity = a3_max_manifolds / 8; /* start at 1024, grow to 8192 */
        world -> manifolds = (collision_data *) malloc ((size_t) world -> manifold_capacity * sizeof (collision_data));
    }
    if (!world -> manifold_awake) { world -> manifold_awake = (unsigned char *) malloc ((size_t) world -> manifold_capacity * sizeof (unsigned char)); }
    if (!world -> manifold_order) { world -> manifold_order = (int *) malloc ((size_t) world -> manifold_capacity * sizeof (int)); }
    if (!world -> manifold_sort_keys) { world -> manifold_sort_keys = (float *) malloc ((size_t) world -> manifold_capacity * sizeof (float)); }
    if (!world -> pair_skipped) { world -> pair_skipped = (unsigned char *) malloc ((size_t) mpe_max_broadphase_pairs * sizeof (unsigned char)); }
    if (!world -> island_parent) { world -> island_parent = (int *) malloc ((size_t) mpe_max_bodies * sizeof (int)); }
    if (!world -> island_label) { world -> island_label = (int *) malloc ((size_t) mpe_max_bodies * sizeof (int)); }
    if (!world -> island_awake_flags) { world -> island_awake_flags = (unsigned char *) malloc ((size_t) mpe_max_bodies * sizeof (unsigned char)); }
    if (!world -> ccd_time_remaining) { world -> ccd_time_remaining = (float *) malloc ((size_t) mpe_max_bodies * sizeof (float)); }
    if (!world -> ccd_best_tois) { world -> ccd_best_tois = (float *) malloc ((size_t) mpe_max_bodies * sizeof (float)); }
    if (!world -> ccd_hit_flags) { world -> ccd_hit_flags = (unsigned char *) malloc ((size_t) mpe_max_bodies * sizeof (unsigned char)); }
    if (!world -> has_contact) { world -> has_contact = (unsigned char *) malloc ((size_t) mpe_max_bodies * sizeof (unsigned char)); }
    if (!world -> tick_v0) {
        world -> tick_v0 = (vector3 *) malloc ((size_t) mpe_max_bodies * sizeof (vector3));
        world -> tick_v0_capacity = world -> tick_v0 ? mpe_max_bodies : 0;
    }
    /* id->index cache (heap; worlds are frequently stack-local). */
    if (!world -> id_cache_keys) {
        world -> id_cache_size = mpe_id_cache_size;
        world -> id_cache_keys = (uint32_t *) calloc ((size_t) world -> id_cache_size, sizeof (uint32_t));
        world -> id_cache_vals = (int *) malloc ((size_t) world -> id_cache_size * sizeof (int));
        world -> id_cache_valid = (unsigned char *) calloc ((size_t) world -> id_cache_size, sizeof (unsigned char));
        if (!world -> id_cache_keys || !world -> id_cache_vals || !world -> id_cache_valid) {
            physics_world_free_ptr ((void **) &world -> id_cache_keys);
            physics_world_free_ptr ((void **) &world -> id_cache_vals);
            physics_world_free_ptr ((void **) &world -> id_cache_valid);
            world -> id_cache_size = 0;
        } world -> id_cache_revision = 0;
    } world -> body_count = 0;
    if (world -> next_object_id == 0) { world -> next_object_id = 1; }
}
void physics_world_cleanup (physics_world *world) {
    if (!world) { return; }
    /* DESPOT-2026-10-01: never trust a wild count from a never-init world.
     * Clamp to the static slot ceiling before walking the table.
     * DESPOT-2026-10-03: that ceiling was 8, but tick_modules[] is [16] and
     * physics_world_attach_module accepts 16. A world with 9..16 modules
     * attached therefore had its count zeroed here and detached NOTHING:
     * every module's per-world state leaked, detach() was never called, and
     * mpe_loader_release_module never ran, so the loader handle kept
     * attachments > 0 and every later mpe_loader_unload returned -2 busy
     * forever. One constant, total loss of a safety constraint. The ceiling
     * must be the array size; better, the single place that knows it. */
    if (world -> tick_module_count < 0 || world -> tick_module_count > MPE_MAX_TICK_MODULES) { world -> tick_module_count = 0; }
    /* Detach modules first: plugin attach() may own per-world state. */
    for (int i = world -> tick_module_count - 1; i >= 0; i--) {
        if (world -> tick_modules [i] && world -> tick_modules [i] -> detach) { world -> tick_modules [i] -> detach (world, world -> tick_module_state [i]); }
        mpe_loader_release_module (world -> tick_modules [i]);
        world -> tick_modules [i] = NULL;
        world -> tick_module_state [i] = NULL;
    }
    /* Stage slots never outlive the world: a cleaned world must not call
     * unloaded code if it is re-init-ed without memset or inspected. */
    world -> broadphase_if = NULL;
    world -> solver_if = NULL;
    world -> broadphase_state = NULL;
    world -> solver_state = NULL;
    live_world_remove (world);
    world -> tick_module_count = 0;
    physics_world_free_ptr ((void **) &world -> bodies);
    physics_world_free_ptr ((void **) &world -> composite_parts);
    world -> composite_part_count = 0;
    world -> composite_part_capacity = 0;
    /* MIG WP2.6 2.6.5: hull pools die with the world. */
    physics_world_free_ptr ((void **) &world -> hull_verts);
    world -> hull_vert_count = 0;
    world -> hull_vert_capacity = 0;
    physics_world_free_ptr ((void **) &world -> hull_faces);
    world -> hull_face_count = 0;
    world -> hull_face_capacity = 0;
    physics_world_free_ptr ((void **) &world -> world_contact_cache);
    physics_world_free_ptr ((void **) &world -> contact_hash_head);
    broadphase_cleanup (world); /* frees the node pool; struct freed below */
    physics_world_free_ptr ((void **) &world -> broadphase);
    physics_world_free_ptr ((void **) &world -> pair_buffer);
    physics_world_free_ptr ((void **) &world -> manifolds);
    physics_world_free_ptr ((void **) &world -> manifold_awake);
    physics_world_free_ptr ((void **) &world -> manifold_order);
    physics_world_free_ptr ((void **) &world -> manifold_sort_keys);
    physics_world_free_ptr ((void **) &world -> pair_skipped);
    physics_world_free_ptr ((void **) &world -> island_parent);
    physics_world_free_ptr ((void **) &world -> island_label);
    physics_world_free_ptr ((void **) &world -> island_awake_flags);
    physics_world_free_ptr ((void **) &world -> ccd_time_remaining);
    physics_world_free_ptr ((void **) &world -> ccd_best_tois);
    physics_world_free_ptr ((void **) &world -> ccd_hit_flags);
    physics_world_free_ptr ((void **) &world -> has_contact);
    physics_world_free_ptr ((void **) &world -> tick_v0);
    world -> tick_v0_capacity = 0;
    physics_world_free_ptr ((void **) &world -> id_cache_keys);
    physics_world_free_ptr ((void **) &world -> id_cache_vals);
    physics_world_free_ptr ((void **) &world -> id_cache_valid);
    world -> id_cache_size = 0;
    world -> world_contact_cache_count = 0;
    world -> manifold_capacity = 0;
    world -> body_count = 0;
    world -> body_capacity = 0;
    world -> cfg = &g_cfg;
}
void physics_world_set_config (physics_world *world, mpe_config_t *cfg) {
    if (!world)
        return;
    world -> cfg = cfg ? cfg : &g_cfg;
} /* Pool growers: ×2 up to the compile-time ceilings. Manifold pointers
 * into bodies[] are rebuilt every tick, and caches store ids (never
 * pointers), so relocation during add_* (outside any step) is safe. */
int physics_world_grow_bodies (physics_world *world) {
    if (!world || !world -> bodies) { return -1; }
    if (world -> body_capacity >= mpe_max_bodies) { return -1; }
    int want = world -> body_capacity > 0 ? world -> body_capacity * 2 : mpe_initial_bodies;
    if (want > mpe_max_bodies) { want = mpe_max_bodies; }
    rigidbody *grown = (rigidbody *) realloc (world -> bodies, (size_t) want * sizeof (rigidbody));
    if (!grown) { return -1; }
    world -> bodies = grown;
    world -> body_capacity = want;
    return 0;
}
/* MIG WP2 2.1/2.2: the P2 cap in force for one world. Default 16, frozen
 * ceiling 16 (configuration can only lower the stored ceiling); a
 * non-positive/garbage config value falls back to the default. */
static int physics_world_composite_cap (const physics_world *world) {
    const mpe_config_t *cfg = mpe_world_cfg (world);
    int cap = (cfg && cfg -> composite.max_parts > 0) ? cfg -> composite.max_parts : 16;
    if (cap > 16) { cap = 16; }
    return cap;
}
/* MIG WP2 2.1: composite part pool grower. Ceiling = mpe_max_bodies * the
 * frozen P2 cap (16) — the most parts any legal body population can own.
 * Doubles from 16; relocation is safe because bodies store part_base
 * indices, never pointers into this array. */
static int physics_world_grow_composite_parts (physics_world *world) {
    if (!world) { return -1; }
    const int ceiling = mpe_max_bodies * 16;
    if (world -> composite_part_capacity >= ceiling) { return -1; }
    int want = world -> composite_part_capacity > 0 ? world -> composite_part_capacity * 2 : 16;
    if (want > ceiling) { want = ceiling; }
    composite_part *grown =
        (composite_part *) realloc (world -> composite_parts, (size_t) want * sizeof (composite_part));
    if (!grown) { return -1; }
    world -> composite_parts = grown;
    world -> composite_part_capacity = want;
    return 0;
}
int physics_world_composite_parts_append (physics_world *world, const composite_part *parts, int count,
                                          int first_index, int *stored_out) {
    if (!world || !parts || count <= 0 || first_index < 0) { return -1; }
    const int cap = physics_world_composite_cap (world);
    const int stored = (count > cap) ? cap : count;
    /* Validate the entries that will actually be stored before mutating:
     * the pool holds core primitives only; object_custom/composite are not
     * legal parts (2.2's assembly validation rejects them with mig errors). */
    for (int i = 0; i < stored; i++) {
        if (parts [i].type != object_sphere && parts [i].type != object_cube &&
            parts [i].type != object_cylinder) {
            return -1;
        }
    }
    while (world -> composite_part_count + stored > world -> composite_part_capacity) {
        if (physics_world_grow_composite_parts (world) != 0) { return -1; }
    }
    /* Write-on-success: the substitute is counted only once the storage is
     * guaranteed, so a failed append leaves both the pool and the counter
     * untouched. */
    if (count > cap) { mpe_note_composite_clamp ((double) count, (double) stored); }
    const int base = world -> composite_part_count;
    for (int i = 0; i < stored; i++) {
        composite_part *dst = &world -> composite_parts [base + i];
        *dst = parts [i];
        dst -> part_index = first_index + i;
        dst -> active = 1;
    }
    world -> composite_part_count = base + stored;
    if (stored_out) { *stored_out = stored; }
    return base;
}
int physics_world_grow_contact_cache (physics_world *world) {
    if (!world || !world -> world_contact_cache) { return -1; }
    if (world -> world_contact_cache_capacity >= max_cached_contacts) { return -1; }
    int want = world -> world_contact_cache_capacity > 0 ? world -> world_contact_cache_capacity * 2 : mpe_initial_contacts;
    if (want > max_cached_contacts) { want = max_cached_contacts; }
    cached_contact *grown =
        (cached_contact *) realloc (world -> world_contact_cache, (size_t) want * sizeof (cached_contact));
    if (!grown) { return -1; }
    world -> world_contact_cache = grown;
    world -> world_contact_cache_capacity = want;
    return 0;
}
int physics_world_grow_manifolds (physics_world *world) {
    if (!world || !world -> manifolds || !world -> manifold_awake || !world -> manifold_order || !world -> manifold_sort_keys) { return -1; }
    if (world -> manifold_capacity >= a3_max_manifolds) { return -1; }
    int want = world -> manifold_capacity > 0 ? world -> manifold_capacity * 2 : a3_max_manifolds / 8;
    if (want > a3_max_manifolds) { want = a3_max_manifolds; }
    /* Atomic: allocate all to temporaries, commit only if all succeed. */
    collision_data *grown = (collision_data *) malloc ((size_t) want * sizeof (collision_data));
    unsigned char *grown_awake = (unsigned char *) malloc ((size_t) want * sizeof (unsigned char));
    int *grown_order = (int *) malloc ((size_t) want * sizeof (int));
    float *grown_keys = (float *) malloc ((size_t) want * sizeof (float));
    if (!grown || !grown_awake || !grown_order || !grown_keys) {
        free (grown);
        free (grown_awake);
        free (grown_order);
        free (grown_keys);
        return -1;
    } memcpy (grown, world -> manifolds, (size_t) world -> manifold_capacity * sizeof (collision_data));
    memcpy (grown_awake, world -> manifold_awake, (size_t) world -> manifold_capacity);
    memcpy (grown_order, world -> manifold_order, (size_t) world -> manifold_capacity * sizeof (int));
    memcpy (grown_keys, world -> manifold_sort_keys, (size_t) world -> manifold_capacity * sizeof (float));
    free (world -> manifolds);
    free (world -> manifold_awake);
    free (world -> manifold_order);
    free (world -> manifold_sort_keys);
    world -> manifolds = grown;
    world -> manifold_awake = grown_awake;
    world -> manifold_order = grown_order;
    world -> manifold_sort_keys = grown_keys;
    world -> manifold_capacity = want;
    return 0;
}
mpe_config_t *physics_world_get_config (physics_world *world) {
    if (!world || !world -> cfg)
        return &g_cfg;
    return world -> cfg;
}
void physics_world_bump_revision (physics_world *world) {
    if (!world)
        return;
    world -> body_revision++;
}
static void physics_world_id_cache_rebuild (physics_world *world) {
    for (int i = 0; i < world -> id_cache_size; i++) {
        world -> id_cache_valid [i] = 0;
    }
    if (!world -> bodies || world -> body_count <= 0) {
        world -> id_cache_revision = world -> body_revision;
        return;
    } uint32_t mask = (uint32_t) (world -> id_cache_size - 1);
    for (int i = 0; i < world -> body_count; i++) {
        uint32_t id = world -> bodies [i].object_id;
        if (id == 0 || id == 0xFFFFFFFFu) { continue; }
        uint32_t h = (id * 2654435761u) & mask;
        for (int probe = 0; probe < 32; probe++) {
            uint32_t s = (h + (uint32_t) probe) & mask;
            if (!world -> id_cache_valid [s]) {
                world -> id_cache_keys [s] = id;
                world -> id_cache_vals [s] = i;
                world -> id_cache_valid [s] = 1;
                break;
            }
        }
    } world -> id_cache_revision = world -> body_revision;
}
int physics_world_index_by_id (physics_world *world, uint32_t id) {
    if (!world || !world -> bodies || world -> body_count <= 0 || id == 0) { return -1; }
    if (world -> id_cache_size > 0 && world -> id_cache_keys && world -> id_cache_vals && world -> id_cache_valid) {
        if (world -> id_cache_revision != world -> body_revision) { physics_world_id_cache_rebuild (world); }
        uint32_t mask = (uint32_t) (world -> id_cache_size - 1);
        uint32_t h = (id * 2654435761u) & mask;
        for (int probe = 0; probe < 32; probe++) {
            uint32_t s = (h + (uint32_t) probe) & mask;
            if (!world -> id_cache_valid [s]) { break; }
            if (world -> id_cache_keys [s] == id) {
                int idx = world -> id_cache_vals [s];
                if (idx >= 0 && idx < world -> body_count && world -> bodies [idx].object_id == id) { return idx; }
                break;
            }
        }
    }
    for (int i = 0; i < world -> body_count; i++) {
        if (world -> bodies [i].object_id == id) { return i; }
    } return -1;
}
rigidbody *physics_world_body_by_id (physics_world *world, uint32_t id) {
    int idx = physics_world_index_by_id (world, id);
    if (idx < 0) { return NULL; }
    return &world -> bodies [idx];
}
void physics_world_set_broadphase (physics_world *world, const mpe_broadphase_if_t *iface) {
    if (!world)
        return;
    world -> broadphase_if = iface;
}
void physics_world_set_solver (physics_world *world, const mpe_solver_if_t *iface) {
    if (!world)
        return;
    world -> solver_if = iface;
}
void physics_world_set_broadphase_state (physics_world *world, void *state) {
    if (!world)
        return;
    world -> broadphase_state = state;
}
void physics_world_set_solver_state (physics_world *world, void *state) {
    if (!world)
        return;
    world -> solver_state = state;
} /* DESPOT-2026-09-29: re-resolve a snapshotted module's live state.
 *
 * physics_world_step snapshots {desc, state} and then invokes each hook,
 * which the header and readme both describe as safe ("a hook may
 * attach/detach (structural change deferred in effect to next tick)").
 * Deferring the TABLE change is not enough: physics_world_detach_module runs
 * desc->detach(), which FREES mod_state (ftc_fleet_detach -> ftc_fleet_destroy
 * -> free). The snapshot then reached that slot and called
 * mods[mi]->pre_step(world, dt, states[mi]) with a dangling state pointer.
 * The dlclose variant is worse: a hook that also calls mpe_loader_unload on a
 * sibling unmaps the image, and the loop reads the function pointer straight
 * out of unmapped memory.
 *
 * Fix: never call through the snapshotted state. Look the descriptor up in
 * the LIVE table each time and skip it if it is no longer attached. Modules
 * attached mid-tick are correctly deferred to the next tick (matching the
 * documented contract), and modules detached mid-tick are never invoked. */
bool physics_world_module_live_state (physics_world *world, const mpe_module_desc_t *desc, void **out_state) {
    if (out_state)
        * out_state = NULL;
    if (!world || !desc)
        return false;
    int n = world -> tick_module_count;
    if (n > 16)
        n = 16;
    for (int i = 0; i < n; i++) {
        if (world -> tick_modules [i] == desc) {
            if (out_state)
                * out_state = world -> tick_module_state [i];
            return true;
        }
    } return false;
}
const char *physics_world_attach_strerror (int code) {
    switch (code) {
        case 0: return ("ok");
        case MPE_ATTACH_E_NULL: return ("null world or module descriptor");
        case MPE_ATTACH_E_ABI: return ("module ABI does not match the engine (rebuild the plugin against this tree)");
        case MPE_ATTACH_E_COUNT: return ("world tick-module count is corrupt (cleanup reset it; re-init the world)");
        case MPE_ATTACH_E_TABLE: return ("module table full (detach another module first)");
        case MPE_ATTACH_E_HOOK: return ("the module's own attach hook refused (see its message above)");
        case MPE_ATTACH_E_NAME: return ("module has no name");
        default: return ("unknown attach failure");
    }
}
int physics_world_attach_module (physics_world *world, const mpe_module_desc_t *desc) {
    /* Every rejection is a DISTINCT negative code and a diag record. These
     * used to collapse into a bare -1, which forced the UI to print a guess
     * ("table full / attach hook") that was wrong more often than right.
     * All codes stay < 0 so every existing `>= 0` / `< 0` caller is
     * unaffected — only the ability to name the cause is new. */
    if (!world || !desc) {
        MPE_DIAG_ERROR ("attach", "E_NULL", "attach rejected: %s (world=%p desc=%p)", world ? "descriptor is NULL" : "world is NULL",
                        (const void *) world, (const void *) desc);
        return MPE_ATTACH_E_NULL;
    }
    if (desc->abi < MPE_MODULE_ABI_MIN || desc->abi > MPE_MODULE_ABI) {
        MPE_DIAG_ERROR ("attach", "E_ABI", "attach rejected: '%s' has ABI %u, engine accepts %u..%u",
                        desc->name ? desc->name : "(unnamed)", desc->abi,
                        (unsigned) MPE_MODULE_ABI_MIN, (unsigned) MPE_MODULE_ABI);
        return MPE_ATTACH_E_ABI;
    }
    /* DESPOT-2026-10-12 P1: validate count BEFORE any indexed read. A
     * never-init'ed stack world holds an indeterminate count; the old order
     * iterated tick_modules[] first (OOB read + NULL-deref in the E_TABLE
     * listing path). Corrupt count is refused before it is used as a bound. */
    if (world->tick_module_count < 0 || world->tick_module_count > MPE_MAX_TICK_MODULES) {
        MPE_DIAG_ERROR ("attach", "E_COUNT", "attach rejected: tick_module_count=%d outside [0,%d] for '%s'",
                        world->tick_module_count, MPE_MAX_TICK_MODULES, desc->name ? desc->name : "(unnamed)");
        return MPE_ATTACH_E_COUNT;
    }
    for (int i = 0; i < world->tick_module_count; i++)
        if (world->tick_modules [i] == desc)
            return i;
    if (world->tick_module_count >= MPE_MAX_TICK_MODULES) {
        MPE_DIAG_ERROR ("attach", "E_TABLE", "attach rejected: table full (%d/%d slots in use); attached: ", world->tick_module_count,
                        MPE_MAX_TICK_MODULES);
        for (int i = 0; i < world->tick_module_count; i++) {
            MPE_DIAG_INFO ("attach", "E_TABLE_LIST", "  slot %d: '%s'", i,
                           world->tick_modules [i] && world->tick_modules [i]->name ? world->tick_modules [i]->name : "?");
        }
        return MPE_ATTACH_E_TABLE;
    }
    if (!desc->name || !*desc->name) {
        /* Only the null case is a guard: a missing name makes every later
         * strcmp and the registry's owned 64-byte copy undefined. Length is
         * NOT enforced here — the registry already truncates safely, and
         * adding a second, different limit is how a valid module would end
         * up rejected for no stated reason. */
        MPE_DIAG_ERROR ("attach", "E_NAME", "attach rejected: module name is missing");
        return MPE_ATTACH_E_NAME;
    }
void *st = NULL;
if (desc->attach) {
        /* Call the hook EXACTLY once: it owns side effects (spawning a
         * child process, allocating pools). Capturing the code once and
         * reporting that value is the whole point — the module knows why
         * it refused, and guessing on its behalf is what cost days. */
        int hook_rc = desc->attach (world, &st);
        if (hook_rc != 0) {
            MPE_DIAG_ERROR ("attach", "E_HOOK", "attach rejected: '%s' attach hook returned %d (state=%p)", desc->name, hook_rc, st);
            return MPE_ATTACH_E_HOOK;
        }
    }
    world -> tick_modules [world -> tick_module_count] = desc;
    world -> tick_module_state [world -> tick_module_count] = st;
    mpe_loader_retain_module (desc);
    return world -> tick_module_count++;
}
int physics_world_detach_module (physics_world *world, const char *name) {
    if (!world || !name) {
        MPE_DIAG_ERROR ("detach", "E_NULL", "detach rejected: %s", world ? "name is NULL" : "world is NULL");
        return MPE_DETACH_E_NULL;
    }
    /* DESPOT-2026-10-12 P1: same corrupt-count guard as attach. Detach had
     * no clamp at all; a garbage count drove OOB reads in both the search
     * loop and the not-found listing loop below. */
    if (world->tick_module_count < 0 || world->tick_module_count > MPE_MAX_TICK_MODULES) {
        MPE_DIAG_ERROR ("detach", "E_COUNT", "detach rejected: tick_module_count=%d outside [0,%d] for '%s'",
                        world->tick_module_count, MPE_MAX_TICK_MODULES, name);
        return MPE_DETACH_E_COUNT;
    }
    for (int i = 0; i < world->tick_module_count; i++) {
        if (world -> tick_modules [i] && world -> tick_modules [i] -> name &&
            strcmp (world -> tick_modules [i] -> name, name) == 0) {
            if (world -> tick_modules [i] -> detach)
                world -> tick_modules [i] -> detach (world, world -> tick_module_state [i]);
            mpe_loader_release_module (world -> tick_modules [i]);
            for (int j = i; j + 1 < world -> tick_module_count; j++) {
                world -> tick_modules [j] = world -> tick_modules [j + 1];
                world -> tick_module_state [j] = world -> tick_module_state [j + 1];
            } world -> tick_module_count--;
            world -> tick_modules [world -> tick_module_count] = NULL;
            world -> tick_module_state [world -> tick_module_count] = NULL;
            MPE_DIAG_INFO ("detach", "OK", "detached '%s' from slot %d (%d remaining)", name, i, world->tick_module_count);
            return 0;
        }
    }
    /* Name the real situation: not attached, vs attached under a DIFFERENT
     * name (usually a wrong argument or a name collision). */
    MPE_DIAG_ERROR ("detach", "E_NOTFOUND", "detach rejected: no module named '%s' is attached; attached now: ", name);
    if (world->tick_module_count == 0) {
        MPE_DIAG_INFO ("detach", "E_NOTFOUND_LIST", "  (none — the world has zero attached modules)");
    }
    for (int i = 0; i < world->tick_module_count; i++)
        MPE_DIAG_INFO ("detach", "E_NOTFOUND_LIST", "  slot %d: '%s'", i,
                       world->tick_modules [i] && world->tick_modules [i] -> name ? world->tick_modules [i] -> name : "?");
    return MPE_DETACH_E_NOTFOUND;
} /* Registry-first shape dispatch with built-in fallback.
 * Handles swapped (cube,sphere)/(cyl,sphere)/(cyl,cube) by trying the
 * registered orientation first, then the swapped orientation with a
 * normal flip — mirroring the legacy inline chains. Custom shapes
 * go through the registry only (no built-in fallback). */
/* Sanitise a manifold a FOREIGN pair handler just wrote.
 *
 * DESPOT-2026-09-29: `collision_data` is handed to plugin pair handlers by
 * pointer, and nothing validated what they wrote back. contacts[] is a fixed
 * 4-element array, but the handler chooses contact_count, so returning
 * count = 64 made collision_prepare_solver write 60 contact_point_data
 * structs past the end of the manifold slot -- corrupting neighbouring
 * manifolds and then the heap. A handler returning object_a = NULL NULL-derefs
 * two lines into the solver (the guard only covered the cache id).
 *
 * mpe_module.h documents the struct as NOT a frozen ABI, so a plugin is
 * exactly as likely to get this wrong as to get it right. Clamp here, at the
 * single choke point every plugin path passes through, and refuse a manifold
 * whose bodies are not the pair we asked about.
 */
static bool a3_sanitize_plugin_manifold (collision_data *out, const rigidbody *a, const rigidbody *b) {
    if (!out)
        return false;
    if (out -> contact_count < 0 || out -> contact_count > MPE_MAX_MANIFOLD_CONTACTS) { out -> contact_count = MPE_MAX_MANIFOLD_CONTACTS; }
    /* The pipeline is dispatching the pair (a,b); that is what the solver
     * must act on, so the dispatcher asserts it rather than trusting the
     * handler. A handler that leaves these NULL would NULL-deref two lines
     * into collision_prepare_solver; one that points them at other bodies
     * would have the solver apply impulses to unrelated objects. Both are
     * removed by overwriting, which also keeps a sloppy-but-harmless handler
     * (tests/module_test.c sets object_b = a) working. */
    out -> object_a = (rigidbody *) a;
    out -> object_b = (rigidbody *) b;
    if (out -> contact_count > 0) {
        float n2 = vector3_length_squared (out -> normal_vector);
        if (!isfinite (n2) || !(n2 > 1e-12f)) {
            out -> contact_count = 0;
            return false;
        }
        /* DESPOT-2026-10-07 P1-1: a buggy/hostile mpe_collide_fn could return
         * penetration=NaN, position=1e30 and the solver would ingest it into
         * velocities of two bodies per tick. Validate every contact field
         * at this choke point; zero-count the manifold on violation. */
        for (int i = 0; i < out -> contact_count; i++) {
            contact_point_data *cp = &out -> contacts [i];
            if (!isfinite (cp -> penetration) || (cp -> penetration < -10.0f) || (cp -> penetration > 10.0f)) {
                out -> contact_count = 0;
                return false;
            }
            if (!isfinite (cp -> position.x) || !isfinite (cp -> position.y) || !isfinite (cp -> position.z)) {
                out -> contact_count = 0;
                return false;
            }
            if ((cp -> position.x < -500.0f) || (cp -> position.x > 500.0f) || (cp -> position.y < -500.0f) ||
                (cp -> position.y > 1000.0f) || (cp -> position.z < -500.0f) || (cp -> position.z > 500.0f)) {
                out -> contact_count = 0;
                return false;
            }
            if (!isfinite (cp -> local_position_a.x) || !isfinite (cp -> local_position_a.y) ||
                !isfinite (cp -> local_position_a.z) || !isfinite (cp -> local_position_b.x) ||
                !isfinite (cp -> local_position_b.y) || !isfinite (cp -> local_position_b.z)) {
                out -> contact_count = 0;
                return false;
            }
            if (!isfinite (cp -> ra.x) || !isfinite (cp -> ra.y) || !isfinite (cp -> ra.z) || !isfinite (cp -> rb.x) ||
                !isfinite (cp -> rb.y) || !isfinite (cp -> rb.z)) {
                out -> contact_count = 0;
                return false;
            }
            if (!isfinite (cp -> tangent_vector.x) || !isfinite (cp -> tangent_vector.y) ||
                !isfinite (cp -> tangent_vector.z)) {
                out -> contact_count = 0;
                return false;
            }
        }
    } return true;
}
/* ---- MIG WP2 2.4: composite ghost-view narrowphase ---------------------
 *
 * A composite body is one rigid body whose collision surface is the union
 * of its active parts. The primitive pair functions take rigidbody poses,
 * so the composite branch refreshes a pair of ghost views (memcpy of the
 * parent + part-local pose override) and reuses the registry/builtin pair
 * path unchanged. Ghosts are thread-local scratch: under MPE_THREADS > 1
 * every worker owns its own pair, writes are pair-local, and the pair
 * functions only read the ghost after the refresh. Nothing in the parent
 * is mutated by detection.
 *
 * Contact identity maps back to the PARENT bodies; each contact carries
 * the per-body monotonic part index (0/0 for a non-composite side). The
 * world pipeline emits ONE manifold per colliding part pair: collision_data
 * carries a single normal, so merging part pairs with different normals
 * into one manifold would solve contacts against the wrong plane.    */
#define MPE_COMPOSITE_MAX_PARTS 16
#define MPE_COMPOSITE_MAX_PART_PAIRS (MPE_COMPOSITE_MAX_PARTS * MPE_COMPOSITE_MAX_PARTS)
typedef struct {
    rigidbody a;
    rigidbody b;
} mpe_composite_ghosts;
static _Thread_local mpe_composite_ghosts s_composite_ghosts;
typedef bool (*mpe_composite_emit_fn) (void *ctx, const collision_data *manifold);
/* Bounded, write-on-success pool lookup: a malformed slice is refused, not
 * read past the world pool. */
static const composite_part *mpe_composite_part_at (const physics_world *world, const rigidbody *body, int slot) {
    if ((!world) || (!body) || (body -> type != object_composite)) { return NULL; }
    if ((body -> part_base < 0) || (body -> composite_part_count < 0)) { return NULL; }
    if ((slot < 0) || (slot >= body -> composite_part_count)) { return NULL; }
    long index = (long) body -> part_base + (long) slot;
    if ((index < 0) || (index >= (long) world -> composite_part_count)) { return NULL; }
    return &world -> composite_parts [index];
} static bool mpe_composite_side_count (const physics_world *world, const rigidbody *body, int *count_out) {
    if ((!body) || (!count_out)) { return false; }
    if (body -> type != object_composite) {
        *count_out = 1;
        return true;
    }
    if ((!world) || (body -> part_base < 0) || (body -> composite_part_count < 0)) { return false; }
    long end = (long) body -> part_base + (long) body -> composite_part_count;
    if (end > (long) world -> composite_part_count) { return false; }
    *count_out = body -> composite_part_count;
    return true;
} /* Refresh the ghost from the parent, then override exactly the fields a
 * primitive pair function reads as geometry/material. The parent's motion
 * state is inherited (foreign handlers may read it); the composed-mass and
 * pool bookkeeping is cleared so a ghost can never be mistaken for a body. */
static void mpe_composite_ghost_refresh (rigidbody *ghost, const rigidbody *parent, const composite_part *part) {
    memcpy (ghost, parent, sizeof (*ghost));
    ghost -> type = part -> type;
    ghost -> position = simd_add (parent -> position,
                                  vector4_rotate_to_vector3 (parent -> orientation, part -> local_position));
    ghost -> orientation = vector4_normalisation (
        vector4_multiplication (parent -> orientation, part -> local_orientation));
    ghost -> radius = part -> radius;
    ghost -> half_extensions = part -> half_extensions;
    ghost -> cylinder_half_length = part -> cylinder_half_length;
    ghost -> mass = part -> mass;
    ghost -> inverse_mass =
        ((part -> mass > 0.0f) && isfinite (part -> mass)) ? (1.0f / part -> mass) : 0.0f;
    ghost -> friction_static = part -> friction_static;
    ghost -> friction_kinetic = part -> friction_kinetic;
    ghost -> restitution = part -> restitution;
    ghost -> mass_is_composed = false;
    ghost -> composite_part_count = 0;
    ghost -> part_base = 0;
    rigidbody_update_axes (ghost);
} /* MIG WP2 2.6: stage one floor manifold per touching composite part.
 * The existing static-plane primitive path runs against the same ghost
 * views as 2.4; the solver identity is remapped to the parent and every
 * contact carries the part index, so floor warm starts are part-scoped
 * exactly like broadphase contacts. Manifolds are staged before the wake
 * decision because a sleeping parent must be awake before its floor
 * manifolds run collision_prepare_solver (effective mass is read there). */
typedef struct {
    collision_data manifolds [MPE_COMPOSITE_MAX_PARTS];
    int count;
    float deepest;
} mpe_composite_floor_batch;
static void mpe_composite_floor_detect (physics_world *world, rigidbody *rb, const mpe_config_t *cfg,
                                        mpe_composite_floor_batch *batch) {
    batch -> count = 0;
    batch -> deepest = 0.0f;
    if ((!world) || (!rb) || (!cfg) || (rb -> type != object_composite)) { return; }
    for (int slot = 0; slot < rb -> composite_part_count; slot++) {
        const composite_part *part = mpe_composite_part_at (world, rb, slot);
        if ((!part) || (!part -> active)) { continue; }
        rigidbody ghost;
        mpe_composite_ghost_refresh (&ghost, rb, part);
        collision_data part_floor = {0};
        if (!collision_static_plane_body (&world -> static_plane_body, &ghost, 0.0f,
                                          &part_floor, cfg)) { continue; }
        if (part_floor.contact_count <= 0) { continue; }
        part_floor.object_a = rb;
        part_floor.object_b = &world -> static_plane_body;
        uint8_t part_index = ((part -> part_index >= 0) && (part -> part_index <= 255))
                                 ? (uint8_t) part -> part_index : 0u;
        float part_deepest = 0.0f;
        for (int c = 0; c < part_floor.contact_count; c++) {
            part_floor.contacts [c].part_a = part_index;
            part_floor.contacts [c].part_b = 0u;
            if (part_floor.contacts [c].penetration > part_deepest) {
                part_deepest = part_floor.contacts [c].penetration;
            }
        }
        if (part_deepest > batch -> deepest) { batch -> deepest = part_deepest; }
        if (batch -> count < MPE_COMPOSITE_MAX_PARTS) {
            batch -> manifolds [batch -> count++] = part_floor;
        } else {
            world -> manifold_overflow_count++;
        }
    }
} static float mpe_vector3_axis (vector3 value, int axis) {
    return (axis == 0) ? value.x : ((axis == 1) ? value.y : value.z);
} /* Conservative world AABB of one primitive (the local box hull of a
 * cylinder is (half_length, radius, radius), axle local X — the house
 * convention). Used only as a prefilter; pair functions remain the truth. */
static void mpe_composite_world_aabb (const rigidbody *body, vector3 *center_out, vector3 *half_out) {
    vector3 local = {body -> radius, body -> radius, body -> radius};
    if (body -> type == object_cube) {
        local = body -> half_extensions;
    } else if (body -> type == object_cylinder) {
        local = (vector3) {body -> cylinder_half_length, body -> radius, body -> radius};
    }
    *center_out = body -> position;
    for (int axis = 0; axis < 3; axis++) {
        float extent = fabsf (mpe_vector3_axis (body -> cached_axes [0], axis)) * fabsf (local.x) +
                       fabsf (mpe_vector3_axis (body -> cached_axes [1], axis)) * fabsf (local.y) +
                       fabsf (mpe_vector3_axis (body -> cached_axes [2], axis)) * fabsf (local.z);
        if (axis == 0) { half_out -> x = extent; }
        else if (axis == 1) { half_out -> y = extent; }
        else { half_out -> z = extent; }
    }
} static bool mpe_composite_aabb_overlap (vector3 ca, vector3 ha, vector3 cb, vector3 hb, float margin) {
    if (fabsf (ca.x - cb.x) > ha.x + hb.x + margin) { return false; }
    if (fabsf (ca.y - cb.y) > ha.y + hb.y + margin) { return false; }
    if (fabsf (ca.z - cb.z) > ha.z + hb.z + margin) { return false; }
    return true;
} static bool mpe_shape_dispatch_primitive (physics_world *world, rigidbody *a, rigidbody *b,
                                           collision_data *out);
/* Iterate the active part pairs in fixed definition order. The prefilter
 * runs in the parent pair frame with the solver's slop as margin: a pair
 * admitted as a zero-depth resting contact by the narrowphase must never be
 * discarded here. */
static bool mpe_composite_iterate (physics_world *world, rigidbody *a, rigidbody *b,
                                   mpe_composite_emit_fn emit, void *ctx) {
    if ((!world) || (!a) || (!b) || (!emit)) { return false; }
    bool a_composite = (a -> type == object_composite);
    bool b_composite = (b -> type == object_composite);
    if ((!a_composite) && (!b_composite)) { return false; }
    int a_count = 1, b_count = 1;
    if (!mpe_composite_side_count (world, a, &a_count) ||
        !mpe_composite_side_count (world, b, &b_count)) { return false; }
    const mpe_config_t *cfg = mpe_world_cfg (world);
    float margin = cfg -> solver.penetration_slop;
    if ((!(margin >= 0.0f)) || (!isfinite (margin))) { margin = 0.0f; }
    int part_index_a = 0, part_index_b = 0;
    for (int ia = 0; ia < a_count; ia++) {
        rigidbody *ghost_a = a;
        if (a_composite) {
            const composite_part *part = mpe_composite_part_at (world, a, ia);
            if ((!part) || (!part -> active)) { continue; }
            mpe_composite_ghost_refresh (&s_composite_ghosts.a, a, part);
            ghost_a = &s_composite_ghosts.a;
            part_index_a = part -> part_index;
        }
        for (int ib = 0; ib < b_count; ib++) {
            rigidbody *ghost_b = b;
            if (b_composite) {
                const composite_part *part = mpe_composite_part_at (world, b, ib);
                if ((!part) || (!part -> active)) { continue; }
                mpe_composite_ghost_refresh (&s_composite_ghosts.b, b, part);
                ghost_b = &s_composite_ghosts.b;
                part_index_b = part -> part_index;
            }
            vector3 center_a, half_a, center_b, half_b;
            mpe_composite_world_aabb (ghost_a, &center_a, &half_a);
            mpe_composite_world_aabb (ghost_b, &center_b, &half_b);
            if (!mpe_composite_aabb_overlap (center_a, half_a, center_b, half_b, margin)) { continue; }
            collision_data part_collision = {0};
            if (!mpe_shape_dispatch_primitive (world, ghost_a, ghost_b, &part_collision)) { continue; }
            if (part_collision.contact_count <= 0) { continue; }
            /* Parent identity + part stamps: the solver applies impulses to
             * the real bodies, and the cache key can tell the parts apart. */
            part_collision.object_a = a;
            part_collision.object_b = b;
            uint8_t part_a_u8 = (part_index_a >= 0 && part_index_a <= 255) ? (uint8_t) part_index_a : 0u;
            uint8_t part_b_u8 = (part_index_b >= 0 && part_index_b <= 255) ? (uint8_t) part_index_b : 0u;
            for (int c = 0; c < part_collision.contact_count; c++) {
                part_collision.contacts [c].part_a = part_a_u8;
                part_collision.contacts [c].part_b = part_b_u8;
            } if (!emit (ctx, &part_collision)) { return true; }
        }
    } return true;
} static void a3_clear_contact_parts (collision_data *out) {
    if ((!out) || (out -> contact_count <= 0)) { return; }
    int count = out -> contact_count;
    if (count > MPE_MAX_MANIFOLD_CONTACTS) { count = MPE_MAX_MANIFOLD_CONTACTS; }
    for (int i = 0; i < count; i++) {
        out -> contacts [i].part_a = 0u;
        out -> contacts [i].part_b = 0u;
    }
} /* The pre-composite dispatch body, unchanged: registry-first with the
 * builtin fallback and swapped-orientation handling. Composite iteration
 * calls this directly (its ghosts are primitives) so the branch below can
 * never recurse. */
static bool mpe_shape_dispatch_primitive (physics_world *world, rigidbody *a, rigidbody *b, collision_data *out) {
    /* Builtins are once-registered at world init; no per-pair call. */
    int ca = (a -> type == object_custom) ? a -> custom_shape : -1;
    int cb = (b -> type == object_custom) ? b -> custom_shape : -1;
    mpe_collide_fn fn = mpe_find_pair_handler ((int) a -> type, (int) b -> type, ca, cb);
    if (fn) {
        if (!fn (a, b, out, world))
            return false;
        if (!a3_sanitize_plugin_manifold (out, a, b))
            return false;
        a3_clear_contact_parts (out);
        return true;
    }
    /* try swapped orientation (registry may hold canonical order only) */
    fn = mpe_find_pair_handler ((int) b -> type, (int) a -> type, cb, ca);
    if (fn) {
        collision_data tmp = {0};
        if (!fn (b, a, &tmp, world))
            return false;
        if (!a3_sanitize_plugin_manifold (&tmp, b, a))
            return false;
        *out = tmp;
        out -> normal_vector = simd_scale (tmp.normal_vector, -1.0f);
        out -> object_a = a;
        out -> object_b = b;
        /* Swap body-local frames: contacts were solved in (b,a) space. */
        for (int i = 0; i < out -> contact_count; i++) {
            vector3 t = out -> contacts [i].local_position_a;
            out -> contacts [i].local_position_a = out -> contacts [i].local_position_b;
            out -> contacts [i].local_position_b = t;
            out -> contacts [i].tangent_vector = simd_scale (out -> contacts [i].tangent_vector, -1.0f);
            /* tangent2 intentionally NOT negated: t2 = n×t1 is invariant
             * under the double flip, and prepare rebuilds it from
             * position+normal anyway (collision_solver.c). */
            vector3 r = out -> contacts [i].ra;
            out -> contacts [i].ra = out -> contacts [i].rb;
            out -> contacts [i].rb = r;
        } a3_clear_contact_parts (out);
        return true;
    } return false;
} typedef struct {
    collision_data best;
    float best_depth;
    bool found;
} mpe_best_sink;
static bool mpe_best_emit (void *ctx, const collision_data *manifold) {
    mpe_best_sink *s = (mpe_best_sink *) ctx;
    float depth = -1.0f;
    for (int i = 0; i < manifold -> contact_count; i++) {
        if (manifold -> contacts [i].penetration > depth) { depth = manifold -> contacts [i].penetration; }
    } if ((!s -> found) || (depth > s -> best_depth)) {
        s -> best = *manifold;
        s -> best_depth = depth;
        s -> found = true;
    } return true;
} /* Single-manifold entry (direct callers, depenetration). For composites it
 * returns the part-pair manifold with the deepest contact; the world step
 * path uses mpe_shape_dispatch_all below because one body pair can emit one
 * manifold per touching part pair. A NULL world cannot resolve a pool slice,
 * so a composite direct call without a world has no collision surface. */
bool mpe_shape_dispatch (physics_world *world, rigidbody *a, rigidbody *b, collision_data *out) {
    if ((!a) || (!b) || (!out)) { return false; }
    /* Render-only proxies never collide (semantic backstop for direct
     * dispatch users; broadphase/floor/CCD skip them earlier for perf). */
    if (a -> no_collide || b -> no_collide) { return false; }
    if ((a -> type != object_composite) && (b -> type != object_composite)) {
        return mpe_shape_dispatch_primitive (world, a, b, out);
    } if (!world) { return false; }
    mpe_best_sink sink = {{0}, -1.0f, false};
    mpe_composite_iterate (world, a, b, mpe_best_emit, &sink);
    if (!sink.found) { return false; }
    *out = sink.best;
    return true;
} typedef struct {
    collision_data *out;
    int max_manifolds;
    int written;
    int total;
} mpe_all_sink;
static bool mpe_all_emit (void *ctx, const collision_data *manifold) {
    mpe_all_sink *s = (mpe_all_sink *) ctx;
    s -> total++;
    if (s -> written < s -> max_manifolds) { s -> out [s -> written++] = *manifold; }
    return true;
} /* Multi-manifold entry for the world pipeline: one collision_data per
 * colliding part pair, parent identities, part indices stamped per contact.
 * `dropped_out` (optional) reports part-pair manifolds that did not fit in
 * `max_manifolds`; the caller counts them, never silently loses them. */
int mpe_shape_dispatch_all (physics_world *world, rigidbody *a, rigidbody *b,
                            collision_data *out, int max_manifolds, int *dropped_out) {
    if (dropped_out) { *dropped_out = 0; }
    if ((!a) || (!b) || (!out) || (max_manifolds <= 0)) { return 0; }
    if (a -> no_collide || b -> no_collide) { return 0; }
    if ((a -> type != object_composite) && (b -> type != object_composite)) {
        collision_data manifold = {0};
        if (!mpe_shape_dispatch_primitive (world, a, b, &manifold)) { return 0; }
        out [0] = manifold;
        return 1;
    } if (!world) { return 0; }
    mpe_all_sink sink = {out, max_manifolds, 0, 0};
    mpe_composite_iterate (world, a, b, mpe_all_emit, &sink);
    if (dropped_out && (sink.total > sink.written)) { *dropped_out = sink.total - sink.written; }
    return sink.written;
}
int physics_world_add_sphere (physics_world *world, float radius, float mass, vector3 position) {
    if ((!world) || (!world -> bodies)) { return -1; }
    if (world -> body_count >= world -> body_capacity && physics_world_grow_bodies (world) != 0) { return -1; }
    rigidbody *rb = &world -> bodies [world -> body_count];
    rigidbody_initialisation_sphere (rb, radius, mass, position);
    /* DESPOT-2026-10-07 P1-3: initialisers stamp from global g_cfg. Two
     * worlds with different physics_world_set_config() still built
     * identical-friction bodies. Re-stamp from the owning world's cfg so
     * per-world config is authoritative at construction, not just at step. */
    {
        const mpe_config_t *wcfg = mpe_world_cfg (world);
        rb -> friction_static = wcfg -> body_defaults.sphere_fric_s;
        rb -> friction_kinetic = wcfg -> body_defaults.sphere_fric_k;
        rb -> restitution = wcfg -> body_defaults.sphere_restitution;
    }
    if (world -> next_object_id == 0 || world -> next_object_id == 0xFFFFFFFFu) { world -> next_object_id = 1; }
    rb -> object_id = world -> next_object_id++;
    if (world -> next_object_id == 0 || world -> next_object_id == 0xFFFFFFFFu) { world -> next_object_id = 1; }
    rb -> object_generation = 1;
    rb -> body_index = world -> body_count;
    rigidbody_sanitize (rb);
    physics_world_bump_revision (world);
    return world -> body_count++;
}
int physics_world_add_cube (physics_world *world, vector3 position, vector3 half_extensions, float mass) {
    if ((!world) || (!world -> bodies)) { return -1; }
    if (world -> body_count >= world -> body_capacity && physics_world_grow_bodies (world) != 0) { return -1; }
    rigidbody *rb = &world -> bodies [world -> body_count];
    rigidbody_initialisation_cube (rb, position, half_extensions, mass);
    /* DESPOT-2026-10-07 P1-3: per-world material stamp (see add_sphere). */
    {
        const mpe_config_t *wcfg = mpe_world_cfg (world);
        rb -> friction_static = wcfg -> body_defaults.cube_fric_s;
        rb -> friction_kinetic = wcfg -> body_defaults.cube_fric_k;
        rb -> restitution = wcfg -> body_defaults.cube_restitution;
    }
    if (world -> next_object_id == 0 || world -> next_object_id == 0xFFFFFFFFu) { world -> next_object_id = 1; }
    rb -> object_id = world -> next_object_id++;
    if (world -> next_object_id == 0 || world -> next_object_id == 0xFFFFFFFFu) { world -> next_object_id = 1; }
    rb -> object_generation = 1;
    rb -> body_index = world -> body_count;
    rigidbody_sanitize (rb);
    physics_world_bump_revision (world);
    return world -> body_count++;
} /* MPE_FTC_091 */
int physics_world_add_cylinder (physics_world *world, float radius, float half_length, float mass, vector3 position) {
    if ((!world) || (!world -> bodies)) { return -1; }
    if (world -> body_count >= world -> body_capacity && physics_world_grow_bodies (world) != 0) { return -1; }
    rigidbody *rb = &world -> bodies [world -> body_count];
    rigidbody_initialisation_cylinder (rb, radius, half_length, mass, position);
    /* DESPOT-2026-10-07 P1-3: per-world material stamp (see add_sphere). */
    {
        const mpe_config_t *wcfg = mpe_world_cfg (world);
        rb -> friction_static = wcfg -> body_defaults.cylinder_fric_s;
        rb -> friction_kinetic = wcfg -> body_defaults.cylinder_fric_k;
        rb -> restitution = wcfg -> body_defaults.cylinder_restitution;
    }
    if (world -> next_object_id == 0 || world -> next_object_id == 0xFFFFFFFFu) { world -> next_object_id = 1; }
    rb -> object_id = world -> next_object_id++;
    if (world -> next_object_id == 0 || world -> next_object_id == 0xFFFFFFFFu) { world -> next_object_id = 1; }
    rb -> object_generation = 1;
    rb -> body_index = world -> body_count;
    rigidbody_sanitize (rb);
    physics_world_bump_revision (world);
    return world -> body_count++;
}
int physics_world_add_custom (physics_world *world, int custom_shape, vector3 position, float mass, float radius) {
    if ((!world) || (!world -> bodies))
        return -1;
    if (world -> body_count >= world -> body_capacity && physics_world_grow_bodies (world) != 0)
        return -1;
    if (custom_shape < 100)
        custom_shape = 100;
    rigidbody *rb = &world -> bodies [world -> body_count];
    /* Backing is a sphere (bounding volume + inertia sane until the
     * plugin overrides); type marks it foreign for dispatch. */
    rigidbody_initialisation_sphere (rb, radius > 0.0f ? radius : 0.5f, mass, position);
    rb -> type = object_custom;
    rb -> custom_shape = custom_shape;
    if (world -> next_object_id == 0 || world -> next_object_id == 0xFFFFFFFFu)
        world -> next_object_id = 1;
    rb -> object_id = world -> next_object_id++;
    if (world -> next_object_id == 0 || world -> next_object_id == 0xFFFFFFFFu)
        world -> next_object_id = 1;
    rb -> object_generation = 1;
    rb -> body_index = world -> body_count;
    rigidbody_sanitize (rb);
    rb -> type = object_custom; /* sanitize must not reset foreign type */
    rb -> custom_shape = custom_shape;
    physics_world_bump_revision (world);
    return world -> body_count++;
}
/* ---- MIG composite application point (the WP1 1.4 stub completed by WP2
 * 2.2): one object_composite body at the assembly COM carrying the
 * composed mass and AUTHORED inertia tensor, plus the stored part slice
 * in the per-world pool.
 *
 * Contract, in order:
 *  1. Validate the WHOLE caller assembly with mig_compose — a malformed
 *     part anywhere refuses the whole assembly (write-on-success), even
 *     when that part would fall outside the P2 cap.
 *  2. Apply the P2 cap (first max_parts kept). The body's truth is
 *     composed over the STORED slice, so mass/inertia never include
 *     discarded weight; the substitution is counted exactly once, only
 *     after the pool append has succeeded.
 *  3. Compose the stored slice through mig_make_bridge, which returns
 *     COM-framed parts and totals about the COM (decision (A)).
 *  4. Pre-validate the engine windows (mass [1e-4,1e6], bounding radius
 *     [0.01,100], finite authored tensor with a finite non-zero inverse).
 *     Every rejection is a named -1 and mutates nothing — never a counted
 *     substitution.
 *  5. Reserve body storage, append the pool slice, then write the body:
 *     mass/inverse/authored tensors, type object_composite,
 *     part_base/count, sphere-backed bound. The sanitize pass runs under
 *     the composed-inertia guard (rigidbody.c), which restores from the
 *     authored tensor instead of a type default; rigidbody_update_axes
 *     refreshes the world-frame cache.
 * Part materials are copied per part from the config defaults for the
 * part's own primitive type, so 2.4's ghost views never re-read the
 * parent body's body-level materials. */
static int physics_world_composite_inverse (const math3 *inertia, math3 *inverse_out) {
    if ((!inertia) || (!inverse_out)) { return -1; }
    bool nonzero = false;
    for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 3; c++) {
            if (!isfinite (inertia -> matrix [r][c])) { return -1; }
        }
    }
    const math3 inv = math3_inverse (*inertia);
    for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 3; c++) {
            if (!isfinite (inv.matrix [r][c])) { return -1; }
            if (inv.matrix [r][c] != 0.0f) { nonzero = true; }
        }
    }
    if (!nonzero) { return -1; }
    *inverse_out = inv;
    return 0;
}
/* Copy one COM-framed mig part into the engine's pool record. Mass is the
 * EFFECTIVE kg from mig_part_mass (density mode already converted by the
 * same exact formulas mig_compose used), so the pool never stores a
 * density where a mass belongs. */
static void physics_world_composite_part_from_mig (composite_part *dst, const mig_part *src, int part_index,
                                                   const mpe_config_t *cfg) {
    *dst = (composite_part) {0};
    dst -> local_position = src -> position;
    dst -> local_orientation = src -> orientation;
    dst -> part_index = part_index;
    dst -> active = 1;
    double effective_mass = 0.0;
    (void) mig_part_mass (src, &effective_mass); /* validated by the compose pass */
    dst -> mass = (float) effective_mass;
    if (src -> type == mig_part_cube) {
        dst -> type = object_cube;
        dst -> half_extensions = src -> half_extensions;
        dst -> friction_static = cfg -> body_defaults.cube_fric_s;
        dst -> friction_kinetic = cfg -> body_defaults.cube_fric_k;
        dst -> restitution = cfg -> body_defaults.cube_restitution;
    } else if (src -> type == mig_part_cylinder) {
        dst -> type = object_cylinder;
        dst -> radius = src -> radius;
        dst -> cylinder_half_length = src -> cylinder_half_length;
        dst -> friction_static = cfg -> body_defaults.cylinder_fric_s;
        dst -> friction_kinetic = cfg -> body_defaults.cylinder_fric_k;
        dst -> restitution = cfg -> body_defaults.cylinder_restitution;
    } else {
        dst -> type = object_sphere;
        dst -> radius = src -> radius;
        dst -> friction_static = cfg -> body_defaults.sphere_fric_s;
        dst -> friction_kinetic = cfg -> body_defaults.sphere_fric_k;
        dst -> restitution = cfg -> body_defaults.sphere_restitution;
    }
}
/* ---- MIG WP2.6 2.6.5: hull mesh pools + hull bodies ----
 * Pool discipline mirrors the composite parts pool: grow by doubling,
 * contiguous slices anchored on the body, counts reset by
 * physics_world_clear (the only compaction point), storage freed by
 * cleanup, write-on-success appends. Ceiling = mpe_max_bodies * the
 * largest catalog mesh (the 2.6.3 ellipsoid: 86 verts / 504 face
 * indices — the icosa's 12/60 was the previous largest) — the most
 * pool any legal body population can own. The pools grow lazily by
 * doubling from the icosa size, so worlds with a few hulls stay
 * small; the ceiling is the bound, not an allocation. */
static int physics_world_grow_hull_verts (physics_world *world) {
    if (!world) { return -1; }
    const int ceiling = mpe_max_bodies * MIG_HULL_MAX_VERTS;
    if (world -> hull_vert_capacity >= ceiling) { return -1; }
    int want = (world -> hull_vert_capacity > 0) ? world -> hull_vert_capacity * 2 : MIG_HULL_ICOSA_VERTS;
    if (want > ceiling) { want = ceiling; }
    mig_hull_point *grown =
        (mig_hull_point *) realloc (world -> hull_verts, (size_t) want * sizeof (mig_hull_point));
    if (!grown) { return -1; }
    world -> hull_verts = grown;
    world -> hull_vert_capacity = want;
    return 0;
}
static int physics_world_grow_hull_faces (physics_world *world) {
    if (!world) { return -1; }
    const int ceiling = mpe_max_bodies * MIG_HULL_MAX_FACE_INDICES;
    if (world -> hull_face_capacity >= ceiling) { return -1; }
    int want = (world -> hull_face_capacity > 0) ? world -> hull_face_capacity * 2 : (MIG_HULL_ICOSA_FACES * 3);
    if (want > ceiling) { want = ceiling; }
    int *grown = (int *) realloc (world -> hull_faces, (size_t) want * sizeof (int));
    if (!grown) { return -1; }
    world -> hull_faces = grown;
    world -> hull_face_capacity = want;
    return 0;
}
static int physics_world_hull_verts_append (physics_world *world, const mig_hull_point *verts, int count) {
    if (!world || !verts || count <= 0) { return -1; }
    while (world -> hull_vert_count + count > world -> hull_vert_capacity) {
        if (physics_world_grow_hull_verts (world) != 0) { return -1; }
    }
    const int base = world -> hull_vert_count;
    for (int i = 0; i < count; i++) { world -> hull_verts [base + i] = verts [i]; }
    world -> hull_vert_count = base + count;
    return base;
}
static int physics_world_hull_faces_append (physics_world *world, const int *faces, int count) {
    if (!world || !faces || count <= 0) { return -1; }
    while (world -> hull_face_count + count > world -> hull_face_capacity) {
        if (physics_world_grow_hull_faces (world) != 0) { return -1; }
    }
    const int base = world -> hull_face_count;
    for (int i = 0; i < count; i++) { world -> hull_faces [base + i] = faces [i]; }
    world -> hull_face_count = base + count;
    return base;
}
static void physics_world_hull_verts_retreat (physics_world *world, int count) {
    /* Rollback after a failed face append: the vertex slice is unclaimed
     * (no body references it yet), so dropping the tail restores the
     * pool exactly. */
    if (world && (count > 0) && (world -> hull_vert_count >= count)) { world -> hull_vert_count -= count; }
}
int physics_world_add_hull (physics_world *world, mpe_hull_shape_kind kind,
                            double s, double h, float mass, vector3 position) {
    if ((!world) || (!world -> bodies)) { return -1; }
    /* Build the canonical mesh in scratch: the largest catalog mesh is
     * the 2.6.3 ellipsoid (86 verts / 168 faces); every scratch buffer
     * in this file sizes to MIG_HULL_MAX_VERTS so a bigger catalog
     * shape can never overflow one silently. */
    mig_hull_point v [MIG_HULL_MAX_VERTS];
    mig_hull_point_set set;
    if (mig_hull_point_set_init (&set, v, MIG_HULL_MAX_VERTS) != mig_hull_ok) { return -1; }
    const int *faces = NULL;
    int nf = 0;
    switch (kind) {
    case mpe_hull_shape_prism:
        if (!(s > 0.0) || !(h > 0.0)) { return -1; }
        if (mig_hull_build_prism_equilateral (s, h, &set) != mig_hull_ok) { return -1; }
        faces = mig_hull_prism_faces;
        nf = MIG_HULL_PRISM_FACES;
        break;
    case mpe_hull_shape_pyr4:
        if (!(s > 0.0) || !(h > 0.0)) { return -1; }
        if (mig_hull_build_pyr4_square (s, h, &set) != mig_hull_ok) { return -1; }
        faces = mig_hull_pyr4_faces;
        nf = MIG_HULL_PYR4_FACES;
        break;
    case mpe_hull_shape_pyr3:
        if (!(s > 0.0) || !(h > 0.0)) { return -1; }
        if (mig_hull_build_pyr3_equilateral (s, h, &set) != mig_hull_ok) { return -1; }
        faces = mig_hull_pyr3_faces;
        nf = MIG_HULL_PYR3_FACES;
        break;
    case mpe_hull_shape_icosa:
        if (!(s > 0.0)) { return -1; } /* h unused for the icosahedron */
        if (mig_hull_build_icosahedron (s, &set) != mig_hull_ok) { return -1; }
        faces = mig_hull_icosahedron_faces;
        nf = MIG_HULL_ICOSA_FACES;
        break;
    case mpe_hull_shape_ellipsoid:
        /* 2.6.3: (s, h) are the equatorial and polar diameters — the
         * spheroid a = b = s/2, c = h/2 through the tri-axial builder
         * (a general (a, b, c) spawn API is a later brick). */
        if (!(s > 0.0) || !(h > 0.0)) { return -1; }
        if (mig_hull_build_ellipsoid (s / 2.0, s / 2.0, h / 2.0, &set) != mig_hull_ok) { return -1; }
        faces = mig_hull_ell_faces;
        nf = MIG_HULL_ELL_FACES;
        break;
    default:
        return -1;
    }
    if (mig_hull_point_set_validate (&set) != mig_hull_ok) { return -1; }
    /* Unit-density truth: V (= mass at density 1), COM, inertia about
     * the COM through the same divergence-theorem path the 2.5.3 gate
     * pins against closed forms. */
    mig_hull_mass unit;
    if (mig_hull_mesh_props (set.points, set.count, faces, nf, 1.0, &unit) != mig_hull_ok) { return -1; }
    /* Engine policy windows, pre-validated so sanitize substitutes
     * nothing: mass [1e-4, 1e6] (add_composite's windows). */
    if (!(mass >= 1e-4f) || (mass > 1e6f)) { return -1; }
    /* Inertia scales linearly with density: I_real = I_unit · mass / V_unit. */
    const double iscale = (double) mass / unit.mass;
    if (!isfinite (iscale) || !(iscale > 0.0)) { return -1; }
    const double max_diag = fmax (unit.inertia [0][0],
                                  fmax (unit.inertia [1][1], unit.inertia [2][2]));
    /* The catalog shapes are diagonal in their canonical axes (2.6.1's
     * gate pins the off-diagonals through mesh_props). A non-negligible
     * off-diagonal here would mean a non-catalog mesh slipped in —
     * refuse rather than silently drop angular coupling. */
    if (fabs (unit.inertia [0][1]) > 1e-9 * max_diag ||
        fabs (unit.inertia [0][2]) > 1e-9 * max_diag ||
        fabs (unit.inertia [1][2]) > 1e-9 * max_diag) { return -1; }
    /* Rebase the vertices to the COM frame (engine invariant: position
     * IS the COM) and measure the bounding circumradius. */
    mig_hull_point local [MIG_HULL_MAX_VERTS];
    double bound2 = 0.0;
    for (int i = 0; i < set.count; i++) {
        local [i].x = set.points [i].x - unit.com [0];
        local [i].y = set.points [i].y - unit.com [1];
        local [i].z = set.points [i].z - unit.com [2];
        const double d2 = local [i].x * local [i].x + local [i].y * local [i].y +
                          local [i].z * local [i].z;
        if (!isfinite (d2)) { return -1; }
        if (d2 > bound2) { bound2 = d2; }
    }
    const double bound = sqrt (bound2);
    if ((!isfinite (bound)) || (!(bound >= 0.01)) || (bound > 100.0)) { return -1; }
    /* Reserve before the pool appends so the only remaining failure is
     * the append itself; a failed append leaves body_count untouched. */
    if (world -> body_count >= world -> body_capacity && physics_world_grow_bodies (world) != 0) { return -1; }
    const int vbase = physics_world_hull_verts_append (world, local, set.count);
    if (vbase < 0) { return -1; }
    const int fbase = physics_world_hull_faces_append (world, faces, nf * 3);
    if (fbase < 0) { physics_world_hull_verts_retreat (world, set.count); return -1; }
    rigidbody *rb = &world -> bodies [world -> body_count];
    rigidbody_initialisation_sphere (rb, (float) bound, mass, position);
    /* Per-world material stamp (sphere defaults — a hull material
     * surface is a later brick, same as the composite body level). */
    const mpe_config_t *wcfg = mpe_world_cfg (world);
    rb -> friction_static = wcfg -> body_defaults.sphere_fric_s;
    rb -> friction_kinetic = wcfg -> body_defaults.sphere_fric_k;
    rb -> restitution = wcfg -> body_defaults.sphere_restitution;
    rb -> type = object_hull;
    rb -> inertia_tensor_local = (math3) {{{(float) (iscale * unit.inertia [0][0]), 0.0f, 0.0f},
                                           {0.0f, (float) (iscale * unit.inertia [1][1]), 0.0f},
                                           {0.0f, 0.0f, (float) (iscale * unit.inertia [2][2])}}};
    /* The initializer left the SPHERE inverse for this radius/mass; the
     * composed truth replaces the whole pair (add_composite's convention:
     * both the authored tensor and its inverse come from the mesh, never
     * a type-default formula). */
    rb -> inverse_inertia_tensor_local =
        (math3) {{{1.0f / (float) (iscale * unit.inertia [0][0]), 0.0f, 0.0f},
                  {0.0f, 1.0f / (float) (iscale * unit.inertia [1][1]), 0.0f},
                  {0.0f, 0.0f, 1.0f / (float) (iscale * unit.inertia [2][2])}}};
    /* mass_is_composed: the tensor arrived through mesh_props (authored
     * truth), not a type-default formula — same protection class as the
     * composite's authored tensor (sanitize restores the inverse pair
     * from the authored tensor, never a sphere/cube formula). */
    rb -> mass_is_composed = true;
    rb -> hull_vert_base = vbase;
    rb -> hull_vert_count = set.count;
    rb -> hull_face_base = fbase;
    rb -> hull_face_count = nf;
    if (world -> next_object_id == 0 || world -> next_object_id == 0xFFFFFFFFu) { world -> next_object_id = 1; }
    rb -> object_id = world -> next_object_id++;
    if (world -> next_object_id == 0 || world -> next_object_id == 0xFFFFFFFFu) { world -> next_object_id = 1; }
    rb -> object_generation = 1;
    rb -> body_index = world -> body_count;
    rigidbody_sanitize (rb); /* composed guard: authored tensor passes through */
    rigidbody_update_axes (rb);
    physics_world_bump_revision (world);
    return world -> body_count++;
}
/* ---- MIG WP2.6 2.6.5: hull collision consumption (composite.hull-gated) ----
 * Every entry refuses on flag 0 (one hull_flag_skips each), on pool-slice
 * defects, and on non-finite geometry. Manifold convention matches the
 * builtins: normal points FROM the first argument TOWARD the second;
 * contacts carry world positions only (collision_prepare_solver computes
 * ra/rb). Depths are the SAT MTDs (2.5.4 / 2.6.4); contact points are
 * single-point approximations, documented per function — the manifold
 * brick replaces them with feature-based contact sets. */
static bool mpe_hull_slice_ok (const physics_world *world, const rigidbody *rb) {
    if ((!world) || (!rb) || (rb -> type != object_hull)) { return false; }
    if ((rb -> hull_vert_base < 0) || (rb -> hull_vert_count <= 0) ||
        (rb -> hull_face_base < 0) || (rb -> hull_face_count <= 0)) { return false; }
    if ((long) rb -> hull_vert_base + (long) rb -> hull_vert_count > (long) world -> hull_vert_count) { return false; }
    if ((long) rb -> hull_face_base + (long) rb -> hull_face_count > (long) world -> hull_face_count) { return false; }
    return true;
}
bool physics_world_hull_floor_detect (physics_world *world, rigidbody *hull,
                                      float plane_y, collision_data *out) {
    if ((!world) || (!hull) || (!out) || (!world -> bodies)) { return false; }
    if (hull -> type != object_hull) { return false; }
    if (!mpe_world_cfg (world) -> composite.hull) {
        world -> hull_flag_skips++;
        return false;
    }
    if (!mpe_hull_slice_ok (world, hull)) { return false; }
    const mpe_config_t *cfg = mpe_world_cfg (world);
    const vector4 q = hull -> orientation;
    vector3 candidate_positions [MIG_HULL_MAX_VERTS];
    float candidate_penetrations [MIG_HULL_MAX_VERTS];
    int candidate_count = 0;
    for (int i = 0; i < hull -> hull_vert_count; i++) {
        const mig_hull_point *v = &world -> hull_verts [hull -> hull_vert_base + i];
        vector3 p = vector4_rotate_to_vector3 (q, (vector3) {(float) v -> x, (float) v -> y, (float) v -> z});
        p = simd_add (p, hull -> position);
        if ((!isfinite (p.x)) || (!isfinite (p.y)) || (!isfinite (p.z))) { return false; }
        const float penetration = plane_y - p.y;
        /* Slop-gated like collision_static_plane_cube: admit within slop
         * as zero-depth for persistent friction, no bounce. */
        if (penetration > -cfg -> solver.penetration_slop) {
            candidate_positions [candidate_count] = p;
            candidate_penetrations [candidate_count] = (penetration > 0.0f) ? penetration : 0.0f;
            candidate_count++;
        }
    }
    if (candidate_count == 0) { return false; }
    out -> object_a = hull;
    out -> object_b = &world -> static_plane_body;
    out -> normal_vector = (vector3) {0.0f, -1.0f, 0.0f};
    out -> contact_count = 0;
    int max_contacts = (candidate_count < 4) ? candidate_count : 4;
    for (int i = 0; i < max_contacts; i++) {
        int best = i;
        for (int j = i + 1; j < candidate_count; j++) {
            if (candidate_penetrations [j] > candidate_penetrations [best]) { best = j; }
        }
        if (best != i) {
            vector3 temp_position = candidate_positions [i];
            candidate_positions [i] = candidate_positions [best];
            candidate_positions [best] = temp_position;
            float temp_pen = candidate_penetrations [i];
            candidate_penetrations [i] = candidate_penetrations [best];
            candidate_penetrations [best] = temp_pen;
        }
        contact_point_data *cp = &out -> contacts [out -> contact_count];
        cp -> position = candidate_positions [i];
        cp -> penetration = candidate_penetrations [i];
        out -> contact_count++;
    }
    return true;
}
bool mpe_hull_collide_sphere (rigidbody *hull, rigidbody *sphere, void *out_p, mpe_world_t *world) {
    collision_data *out = (collision_data *) out_p;
    if ((!hull) || (!sphere) || (!out) || (!world)) { return false; }
    if ((hull -> type != object_hull) || (sphere -> type != object_sphere)) { return false; }
    if (!mpe_world_cfg (world) -> composite.hull) { world -> hull_flag_skips++; return false; }
    if (!mpe_hull_slice_ok (world, hull)) { return false; }
    if (!isfinite (sphere -> radius) || sphere -> radius <= 0.0f) { return false; }
    /* Sphere centre in the hull's COM-local frame (conjugate rotation;
     * vector4_rotate_to_vector3 normalises internally). */
    const vector4 q = hull -> orientation;
    const vector4 q_conj = {q.w, -q.x, -q.y, -q.z};
    const vector3 d = simd_sub (sphere -> position, hull -> position);
    const vector3 c_local = vector4_rotate_to_vector3 (q_conj, d);
    if ((!isfinite (c_local.x)) || (!isfinite (c_local.y)) || (!isfinite (c_local.z))) { return false; }
    mig_hull_sat_result r;
    const double center [3] = {(double) c_local.x, (double) c_local.y, (double) c_local.z};
    const mig_hull_err e = mig_hull_sat_hull_sphere (&world -> hull_verts [hull -> hull_vert_base],
                                                     hull -> hull_vert_count,
                                                     &world -> hull_faces [hull -> hull_face_base],
                                                     hull -> hull_face_count,
                                                     center, (double) sphere -> radius, &r);
    if (e != mig_hull_ok) { return false; }
    if (!r.overlap) { return false; }
    /* Axis: hull-local, FROM the hull TOWARD the sphere → world. */
    const vector3 n_local = {(float) r.axis [0], (float) r.axis [1], (float) r.axis [2]};
    const vector3 n = vector4_rotate_to_vector3 (q, n_local);
    if ((!isfinite (n.x)) || (!isfinite (n.y)) || (!isfinite (n.z))) { return false; }
    float depth = (float) r.depth;
    if (depth < 0.0f) { depth = 0.0f; } /* touching = overlap at depth 0 */
    out -> object_a = hull;
    out -> object_b = sphere;
    out -> normal_vector = n;
    out -> contact_count = 1;
    contact_point_data *cp = &out -> contacts [0];
    /* Deepest sphere point toward the hull (on the sphere surface). */
    cp -> position = simd_sub (sphere -> position, simd_scale (n, sphere -> radius));
    cp -> penetration = depth;
    return true;
}
bool mpe_hull_collide_cube (rigidbody *hull, rigidbody *cube, void *out_p, mpe_world_t *world) {
    collision_data *out = (collision_data *) out_p;
    if ((!hull) || (!cube) || (!out) || (!world)) { return false; }
    if ((hull -> type != object_hull) || (cube -> type != object_cube)) { return false; }
    if (!mpe_world_cfg (world) -> composite.hull) { world -> hull_flag_skips++; return false; }
    if (!mpe_hull_slice_ok (world, hull)) { return false; }
    const vector3 he = cube -> half_extensions;
    if ((!isfinite (he.x)) || (!isfinite (he.y)) || (!isfinite (he.z))) { return false; }
    /* Hull vertices into the cube's COM-local frame: there the box is
     * AXIS-ALIGNED by construction (half extensions), so the 2.5.4
     * hull-vs-box reference applies unchanged. Rotations preserve
     * distances, so the SAT depth is frame-independent. */
    const vector4 qc = cube -> orientation;
    const vector4 qc_conj = {qc.w, -qc.x, -qc.y, -qc.z};
    const vector4 qh = hull -> orientation;
    mig_hull_point local [MIG_HULL_MAX_VERTS];
    for (int i = 0; i < hull -> hull_vert_count; i++) {
        const mig_hull_point *v = &world -> hull_verts [hull -> hull_vert_base + i];
        vector3 p = vector4_rotate_to_vector3 (qh, (vector3) {(float) v -> x, (float) v -> y, (float) v -> z});
        p = simd_add (p, hull -> position);
        p = simd_sub (p, cube -> position);
        p = vector4_rotate_to_vector3 (qc_conj, p);
        if ((!isfinite (p.x)) || (!isfinite (p.y)) || (!isfinite (p.z))) { return false; }
        local [i].x = (double) p.x;
        local [i].y = (double) p.y;
        local [i].z = (double) p.z;
    }
    mig_hull_sat_result r;
    const double center [3] = {0.0, 0.0, 0.0};
    const double half [3] = {(double) he.x, (double) he.y, (double) he.z};
    const mig_hull_err e = mig_hull_sat_hull_box (local, hull -> hull_vert_count,
                                                  &world -> hull_faces [hull -> hull_face_base],
                                                  hull -> hull_face_count, center, half, &r);
    if (e != mig_hull_ok) { return false; }
    if (!r.overlap) { return false; }
    /* Axis: cube-local, FROM the hull TOWARD the box → world. */
    const vector3 n_local = {(float) r.axis [0], (float) r.axis [1], (float) r.axis [2]};
    const vector3 n = vector4_rotate_to_vector3 (qc, n_local);
    if ((!isfinite (n.x)) || (!isfinite (n.y)) || (!isfinite (n.z))) { return false; }
    float depth = (float) r.depth;
    if (depth < 0.0f) { depth = 0.0f; }
    out -> object_a = hull;
    out -> object_b = cube;
    out -> normal_vector = n;
    out -> contact_count = 1;
    contact_point_data *cp = &out -> contacts [0];
    /* Cube-surface point on the hull side: centre − n·(box support along
     * −n). Deterministic and inside the overlap for convex pairs. */
    const vector3 ax0 = vector4_rotate_to_vector3 (qc, (vector3) {1.0f, 0.0f, 0.0f});
    const vector3 ax1 = vector4_rotate_to_vector3 (qc, (vector3) {0.0f, 1.0f, 0.0f});
    const vector3 ax2 = vector4_rotate_to_vector3 (qc, (vector3) {0.0f, 0.0f, 1.0f});
    const float support = he.x * fabsf (vector3_dot (ax0, n)) + he.y * fabsf (vector3_dot (ax1, n)) +
                          he.z * fabsf (vector3_dot (ax2, n));
    cp -> position = simd_sub (cube -> position, simd_scale (n, support));
    cp -> penetration = depth;
    return true;
}
bool mpe_hull_collide_hull (rigidbody *a, rigidbody *b, void *out_p, mpe_world_t *world) {
    collision_data *out = (collision_data *) out_p;
    if ((!a) || (!b) || (!out) || (!world)) { return false; }
    if ((a -> type != object_hull) || (b -> type != object_hull)) { return false; }
    if (!mpe_world_cfg (world) -> composite.hull) { world -> hull_flag_skips++; return false; }
    if ((!mpe_hull_slice_ok (world, a)) || (!mpe_hull_slice_ok (world, b))) { return false; }
    /* B's vertices into A's COM-local frame; A's stored slice is already
     * local, so the 2.6.4 generic pair applies unchanged. */
    const vector4 qa = a -> orientation;
    const vector4 qa_conj = {qa.w, -qa.x, -qa.y, -qa.z};
    const vector4 qb = b -> orientation;
    mig_hull_point localb [MIG_HULL_MAX_VERTS];
    for (int i = 0; i < b -> hull_vert_count; i++) {
        const mig_hull_point *v = &world -> hull_verts [b -> hull_vert_base + i];
        vector3 p = vector4_rotate_to_vector3 (qb, (vector3) {(float) v -> x, (float) v -> y, (float) v -> z});
        p = simd_add (p, b -> position);
        p = simd_sub (p, a -> position);
        p = vector4_rotate_to_vector3 (qa_conj, p);
        if ((!isfinite (p.x)) || (!isfinite (p.y)) || (!isfinite (p.z))) { return false; }
        localb [i].x = (double) p.x;
        localb [i].y = (double) p.y;
        localb [i].z = (double) p.z;
    }
    mig_hull_sat_result r;
    const mig_hull_err e = mig_hull_sat_hull_hull (&world -> hull_verts [a -> hull_vert_base], a -> hull_vert_count,
                                                   &world -> hull_faces [a -> hull_face_base], a -> hull_face_count,
                                                   localb, b -> hull_vert_count,
                                                   &world -> hull_faces [b -> hull_face_base], b -> hull_face_count,
                                                   &r);
    if (e != mig_hull_ok) { return false; }
    if (!r.overlap) { return false; }
    /* Axis: A-local, FROM A TOWARD B → world. */
    const vector3 n_local = {(float) r.axis [0], (float) r.axis [1], (float) r.axis [2]};
    const vector3 n = vector4_rotate_to_vector3 (qa, n_local);
    if ((!isfinite (n.x)) || (!isfinite (n.y)) || (!isfinite (n.z))) { return false; }
    float depth = (float) r.depth;
    if (depth < 0.0f) { depth = 0.0f; }
    /* Contact: midpoint of the two SAT support points along ±n (the
     * standard single-point SAT contact approximation). */
    float best_a = -FLT_MAX;
    float best_b = FLT_MAX;
    vector3 pa = a -> position, pb = b -> position;
    for (int i = 0; i < a -> hull_vert_count; i++) {
        const mig_hull_point *v = &world -> hull_verts [a -> hull_vert_base + i];
        vector3 p = vector4_rotate_to_vector3 (qa, (vector3) {(float) v -> x, (float) v -> y, (float) v -> z});
        p = simd_add (p, a -> position);
        const float d = vector3_dot (p, n);
        if (d > best_a) { best_a = d; pa = p; }
    }
    for (int i = 0; i < b -> hull_vert_count; i++) {
        const mig_hull_point *v = &world -> hull_verts [b -> hull_vert_base + i];
        vector3 p = vector4_rotate_to_vector3 (qb, (vector3) {(float) v -> x, (float) v -> y, (float) v -> z});
        p = simd_add (p, b -> position);
        const float d = vector3_dot (p, n);
        if (d < best_b) { best_b = d; pb = p; }
    }
    out -> object_a = a;
    out -> object_b = b;
    out -> normal_vector = n;
    out -> contact_count = 1;
    contact_point_data *cp = &out -> contacts [0];
    cp -> position = simd_scale (simd_add (pa, pb), 0.5f);
    cp -> penetration = depth;
    return true;
}
int physics_world_add_composite (physics_world *world, const mig_part *parts, int count) {
    if ((!world) || (!world -> bodies) || (!parts) || count <= 0) { return -1; }
    const int cap = physics_world_composite_cap (world);
    const int stored = (count > cap) ? cap : count;
    /* Whole-assembly structural validation including any over-cap tail:
     * one malformed discarded part still refuses the whole assembly. The
     * stored slice is re-composed below because the body must not carry
     * weight it does not store. */
    if (stored < count) {
        mig_totals all;
        if (mig_compose (parts, count, &all) != mig_compose_ok) { return -1; }
    }
    mig_part scratch [16];
    mig_bridge bridge;
    if (mig_make_bridge (parts, stored, scratch, &bridge) != mig_compose_ok) { return -1; }
    /* Engine policy windows, pre-validated so sanitize substitutes
     * nothing: mass [1e-4, 1e6], bound [0.01, 100]. */
    if (!(bridge.totals.mass >= 1e-4) || (bridge.totals.mass > 1e6)) { return -1; }
    float bound = 0.0f;
    for (int i = 0; i < stored; i++) {
        const mig_part *p = &bridge.parts [i];
        float part_bound = 0.0f;
        if (p -> type == mig_part_sphere) { part_bound = p -> radius; }
        else if (p -> type == mig_part_cube) {
            part_bound = sqrtf (p -> half_extensions.x * p -> half_extensions.x +
                                p -> half_extensions.y * p -> half_extensions.y +
                                p -> half_extensions.z * p -> half_extensions.z);
        } else { /* mig_part_cylinder */
            part_bound = sqrtf (p -> cylinder_half_length * p -> cylinder_half_length + p -> radius * p -> radius);
        }
        /* bridge.parts are already COM-framed, so |position| IS the
         * COM offset; orientations cannot change a worst-case norm. */
        const float r = sqrtf (p -> position.x * p -> position.x + p -> position.y * p -> position.y +
                               p -> position.z * p -> position.z) + part_bound;
        if (r > bound) { bound = r; }
    }
    if ((!isfinite (bound)) || (!(bound >= 0.01f)) || (bound > 100.0f)) { return -1; }
    math3 authored_inverse;
    if (physics_world_composite_inverse (&bridge.totals.inertia, &authored_inverse) != 0) { return -1; }
    /* Build the pool records before touching world storage. */
    composite_part staged [16];
    const mpe_config_t *wcfg = mpe_world_cfg (world);
    for (int i = 0; i < stored; i++) {
        physics_world_composite_part_from_mig (&staged [i], &bridge.parts [i], i, wcfg);
    }
    /* Reserve before the pool append so the only remaining failure is the
     * append itself; a failed append leaves body_count and the counter
     * untouched. */
    if (world -> body_count >= world -> body_capacity && physics_world_grow_bodies (world) != 0) { return -1; }
    int stored_actual = 0;
    const int base = physics_world_composite_parts_append (world, staged, stored, 0, &stored_actual);
    if (base < 0) { return -1; }
    if (count > stored) { mpe_note_composite_clamp ((double) count, (double) stored); }
    rigidbody *rb = &world -> bodies [world -> body_count];
    rigidbody_initialisation_sphere (rb, bound, (float) bridge.totals.mass, bridge.totals.com);
    /* DESPOT-2026-10-07 P1-3: per-world material stamp (body level; the
     * per-part copies above are what 2.4's ghost views read). */
    rb -> friction_static = wcfg -> body_defaults.sphere_fric_s;
    rb -> friction_kinetic = wcfg -> body_defaults.sphere_fric_k;
    rb -> restitution = wcfg -> body_defaults.sphere_restitution;
    rb -> type = object_composite;
    rb -> inertia_tensor_local = bridge.totals.inertia;
    rb -> inverse_inertia_tensor_local = authored_inverse;
    rb -> inverse_inertia_system = authored_inverse; /* identity body frame */
    rb -> mass_is_composed = true;
    rb -> composite_part_count = stored;
    rb -> part_base = base;
    if (world -> next_object_id == 0 || world -> next_object_id == 0xFFFFFFFFu) { world -> next_object_id = 1; }
    rb -> object_id = world -> next_object_id++;
    if (world -> next_object_id == 0 || world -> next_object_id == 0xFFFFFFFFu) { world -> next_object_id = 1; }
    rb -> object_generation = 1;
    rb -> body_index = world -> body_count;
    rigidbody_sanitize (rb); /* composed guard: authored tensors pass through */
    rigidbody_update_axes (rb);
    physics_world_bump_revision (world);
    return world -> body_count++;
}
void physics_world_clear (physics_world *world) {
    if (!world) { return; }
    world -> body_count = 0;
    /* MIG WP2 2.1: clear is the ONLY pool compaction point — every body's
     * slice dies with its body, so the count returns to 0 while the
     * allocated capacity is kept for reuse. */
    world -> composite_part_count = 0;
    /* MIG WP2.6 2.6.5: hull pool slices die with their bodies the same
     * way; capacity is kept for reuse. */
    world -> hull_vert_count = 0;
    world -> hull_face_count = 0;
    world -> hull_flag_skips = 0;
    world -> world_contact_cache_count = 0; /* MFS_131A */
    world -> manifold_overflow_count = 0;
    world -> contact_cache_hits = 0;
    world -> contact_cache_misses = 0;
    world -> contact_cache_hits_applied = 0;
    /* Clear joints: stale body_id_a/b would alias recycled IDs after respawn.
     * DESPOT-2026-10-07 P0-1: counts alone are not enough — spring/constraint
     * solvers loop over is_active, not the count. Zero both. */
    for (int i = 0; i < mpe_max_joints; i++) {
        world -> spring_joints [i].is_active = false;
        world -> revolute_constraints [i].is_active = false;
    } world -> spring_joint_count = 0;
    world -> revolute_constraint_count = 0;
    physics_world_bump_revision (world);
    /* TRUTH: next_object_id monotonic wraps at 4G to 0, colliding with
     * cache sentinel id==0 (no match) + floor 0xFFFFFFFF. Skip 0/0xFFFFFFFF
     * on wrap. clear() does NOT reset IDs (stable across clears would alias
     * old cache entries); instead ensure wrap skips sentinels. */
    if (world -> next_object_id == 0 || world -> next_object_id == 0xFFFFFFFFu) { world -> next_object_id = 1; }
} /* One broadphase pair through narrowphase + wake-on-contact + solver
 * prep. Shared by the main pair loop and the sleep-wake revisit pass
 * below (extracted verbatim from the former single loop). Non-static:
 * the legacy GUI tick reuses it so both step paths share one wake +
 * dispatch implementation (registry-routed, per-world config). */
void physics_world_process_pair (physics_world *world, int index_a, int index_b, float dt, int *manifold_count_ptr) {
    if (!world || !world -> bodies || !manifold_count_ptr || !world -> manifolds) { return; }
    if ((index_a < 0) || (index_a >= world -> body_count)) { return; }
    if ((index_b < 0) || (index_b >= world -> body_count)) { return; }
    rigidbody *body_a = &world -> bodies [index_a];
    rigidbody *body_b = &world -> bodies [index_b];
    /* Static-static pairs have no possible response (both inverse masses
     * zero, neither can wake). The broadphase now culls them; this guard
     * also protects direct callers (alternative stage implementations,
     * legacy loops) from paying narrowphase/solver cost for immovable
     * geometry. */
    if (body_a -> static_state && body_b -> static_state) { return; }
    /* MIG WP2 2.4: a composite body emits one manifold per colliding part
     * pair. They are appended and prepared individually so each keeps its
     * own normal (collision_data has exactly one). */
    if ((body_a -> type == object_composite) || (body_b -> type == object_composite)) {
        if (body_a->is_sleeping && body_b->is_sleeping) { return; }
        if ((*manifold_count_ptr) >= world -> manifold_capacity) {
            if (physics_world_grow_manifolds (world) != 0) {
                world -> manifold_overflow_count++;
                return;
            }
        }
        int room = world -> manifold_capacity - *manifold_count_ptr;
        int dropped = 0;
        collision_data *slots = &world -> manifolds [*manifold_count_ptr];
        int written = mpe_shape_dispatch_all (world, body_a, body_b, slots, room, &dropped);
        for (int i = 0; i < written; i++) {
            collision_data source = slots [i];
            physics_world_prepare_detected_pair (world, index_a, index_b, dt, &source, &slots [i]);
        } *manifold_count_ptr += written;
        if (dropped > 0) { world -> manifold_overflow_count += dropped; }
        return;
    }
    collision_data narrowphase_collision = {0};
    /* Phase-2: registry-first dispatch (foreign shapes plug in here),
         * built-in table fallback preserves exact legacy behaviour. */
    bool collided = mpe_shape_dispatch (world, body_a, body_b, &narrowphase_collision);
    if (collided) {
        if ((*manifold_count_ptr) >= world -> manifold_capacity) {
            if (physics_world_grow_manifolds (world) != 0) {
                world -> manifold_overflow_count++;
                return;
            }
        }
        if (body_a->is_sleeping && body_b->is_sleeping) return;
        physics_world_prepare_detected_pair (world, index_a, index_b, dt,
                                             &narrowphase_collision,
                                             &world->manifolds[*manifold_count_ptr]);
        (*manifold_count_ptr)++;
    }
} /* Solver stage dispatch: foreign solver_if hooks override per stage,
 * builtins run on the tick's config snapshot. Keeps the iteration loop
 * readable while making every stage hot-swappable. */
static float mpe_step_resolve (physics_world *world, collision_data *m, float dt, bool friction_only, int iter,
                               const mpe_config_t *cfg) {
    if (world -> solver_if && world -> solver_if -> resolve) { return world -> solver_if -> resolve (world, m, dt, friction_only, iter, world -> solver_state); }
    return collision_resolve_iterative (m, dt, friction_only, iter, cfg);
}
static void mpe_step_poisson (physics_world *world, collision_data *manifolds, int n, const mpe_config_t *cfg) {
    collision_snapshot_compression_impulses (manifolds, n);
    if (world -> solver_if && world -> solver_if -> poisson) {
        world -> solver_if -> poisson (world, manifolds, n, world -> solver_state);
        return;
    } collision_apply_poisson_restitution (manifolds, n, cfg);
}
static void mpe_step_rolling (physics_world *world, collision_data *manifolds, int n, float dt,
                              const mpe_config_t *cfg) {
    if (world -> solver_if && world -> solver_if -> rolling) {
        world -> solver_if -> rolling (world, manifolds, n, dt, world -> solver_state);
        return;
    } collision_apply_rolling_resistance (manifolds, n, dt, cfg);
}
static void mpe_step_split (physics_world *world, collision_data *manifolds, int n, float dt, const mpe_config_t *cfg) {
    if (world -> solver_if && world -> solver_if -> split) {
        world -> solver_if -> split (world, manifolds, n, dt, world -> solver_state);
        return;
    } collision_apply_split_impulse (manifolds, n, dt, cfg);
}
/* --- Shared step sub-blocks (extracted 2026-10-25, behaviour-preserving) ---
 * physics_world_step carried two bodies of code twice apiece: the manifold
 * awake-flag pass ran once before forces and again after motors/forces (a
 * motor wake flips a body awake after the first pass, which left joint+contact
 * solves skipped for that tick — the DESPOT island-rebuild note below), and
 * the foreign tick-module snapshot/call loop ran once pre-integration
 * (pre_step) and once at end of step (post_step). The call sites below
 * produce the same machine state as the deleted copies; the pipeline can now
 * drift this logic in exactly one place. Verified: suite 46/46 incl. the
 * 5-regime matrix, determinism twins bitwise identical, engine build 0
 * warnings under -Werror. */

/* One pass over the solver manifolds: an empty pair (non-collided, NULL
 * object pointers) is never awake — islands_body_awake (NULL) returns true
 * and would wrongly mark it — and a pair is awake when either body's
 * union-find island is awake. Fully-sleeping islands then skip the iteration
 * and relaxation sweeps (exact no-ops: zero velocity, zeroed inverses). */
static void mpe_step_flag_manifolds_awake (physics_world *world, int manifold_count) {
    for (int m = 0; m < manifold_count; m++) {
        rigidbody *ma = world -> manifolds [m].object_a;
        rigidbody *mb = world -> manifolds [m].object_b;
        if (!ma || !mb || world -> manifolds [m].contact_count == 0) {
            world -> manifold_awake [m] = 0;
            continue;
        }
        world -> manifold_awake [m] = (unsigned char) (islands_body_awake (world, ma) || islands_body_awake (world, mb));
    }
}

/* Foreign forcefield/motor tick modules. Snapshot the descriptor table first:
 * a hook may attach or detach mid-loop (structural changes are deferred to
 * next tick), and walking world -> tick_modules directly would read a table a
 * hook just mutated. Each call then re-resolves the module's live state
 * through physics_world_module_live_state (the older revision's
 * a3_module_live_state): a module that legitimately holds NULL state is
 * distinguished from one that is no longer attached, so a NULL-state module
 * detached mid-tick is still skipped. Called pre=1 before integration
 * (forcefield/motor hooks) and pre=0 at end of step (loggers/correctors).
 * The old revision snapshot a parallel states [] array beside mods []; that
 * copy was dead (never read since live re-resolution landed) and is not
 * reproduced here. */
static void mpe_step_call_foreign_modules (physics_world *world, float dt, bool pre) {
    const mpe_module_desc_t *mods [16];
    int nmods = world -> tick_module_count < 16 ? world -> tick_module_count : 16;
    for (int mi = 0; mi < nmods; mi++) {
        mods [mi] = world -> tick_modules [mi];
    }
    for (int mi = 0; mi < nmods; mi++) {
        if (!mods [mi]) { continue; }
        if (pre && !mods [mi] -> pre_step) { continue; }
        if ((!pre) && !mods [mi] -> post_step) { continue; }
        void *live = NULL;
        if (!physics_world_module_live_state (world, mods [mi], &live)) { continue; }
        if (pre) { mods [mi] -> pre_step (world, dt, live); }
        else { mods [mi] -> post_step (world, dt, live); }
    }
}

void physics_world_step (physics_world *world, float dt) {
    if (!world) { return; }
    /* Telemetry is reset every step so a previous tick cannot leak into the
     * overlay (and a degraded/empty tick reads as zero, not stale). */
    world -> last_broadphase_pair_count = 0;
    world -> last_manifold_count = 0;
    if ((!world -> bodies) || (!(dt > 0.0f)) || (!isfinite (dt)) || (world -> body_count <= 0)) { return; }
    /* DESPOT-2026-10-07 LIE-05: fixed-dt is caller discipline. Bitwise
     * determinism requires identical dt sequences; variable-dt callers
     * silently desync (damping pow(drag,dt), CCD sweep*dt, rotor |w|*dt/2).
     * The GUI loop is fixed 1/60; headless callers must pin dt=1/60.
     * Non-1/60 dt still steps (no breakage) but is LOUD so a variable-dt
     * harness cannot claim determinism. */
    if ((dt < 0.016666f) || (dt > 0.016668f)) {
        /* P7: _Atomic — two worlds stepping on two threads raced the plain
         * int (UB). Telemetry limit stands: first regime consumes the budget. */
        static _Atomic int s_dt_warned = 0;
        if (!MPE_ATOMIC_LOAD_n (&s_dt_warned, MPE_ATOMIC_RELAXED)) {
            MPE_ATOMIC_STORE_n (&s_dt_warned, 1, MPE_ATOMIC_RELAXED);
            fprintf (stderr, "[mpe] NON-CANONICAL dt=%.6f (canonical 1/60=0.016667); determinism twins must use identical dt\n",
                     (double) dt);
        }
    }
    /* TRUTH: spiral-of-death guard rescales time (dt>0.1s clamped, sim lags
     * wall) instead of substepping. Substepping would preserve time at the
     * cost of unbounded catch-up work; clamping bounds work and keeps every
     * tick at fixed dt (determinism's first requirement). Time dilation
     * under extreme lag is documented behavior, not hidden.
     * DESPOT-2026-10-13: counted — twins with different lag histories used
     * to diverge silently with no budget anywhere. */
    if (dt > 0.1f) {
        mpe_note_solver_clamp ("step dt", (double) dt, 0.1);
        dt = 0.1f;
    }
    /* DESPOT-2026-10-08 threading: per-body sanitize is order-independent
     * (disjoint writes). MT when MPE_THREADS>1 and n>=64, else serial.
     * Bitwise identical either way. */
    mpe_parallel_sanitize (world);
    /* Contact preparation accumulates relative speed for sleep gating. Reset
     * before narrowphase so this tick's measurements survive to the sleep
     * update below (and stale values from last tick cannot leak forward). */
    for (int i = 0; i < world -> body_count; i++) {
        world -> bodies [i].max_relative_speed_sq = 0.0f;
    }
    /* TRUTH: snapshot config once per tick. Menu/terminal mutating g_cfg
     * mid-tick (between substeps) would otherwise change behavior halfway
     * through the frame. Hot path uses these locals, never g_cfg directly.
     * Phase-1: snapshot from per-world cfg (defaults to global). */
    const mpe_config_t *step_cfg = mpe_world_cfg (world);
    const float step_gravity = step_cfg -> world.gravity;
    const float step_drag = step_cfg -> world.drag;
    const float step_ang_scale = step_cfg -> world.angular_damping_scale;
    const int step_iterations = step_cfg -> timestep.solver_iterations;
    /* TRUTH P0-1: reset contact flags (set below on every manifold). */
    if (world -> has_contact) {
        for (int i = 0; i < world -> body_count; i++) {
            world -> has_contact [i] = 0;
        }
    }
    /* CCD: clamp fast bodies to their time-of-impact pose before pairing,
     * so discrete narrowphase cannot tunnel past thin geometry.
     * TRUTH P0-3: remainder (dt-toi) recorded per body; post-solve
     * integration advances only the remainder (see below). */
    mpe_parallel_ccd_sweep (world, dt, step_cfg);
    int pair_count = 0;
    if (world -> body_count >= 2) {
        if (world -> broadphase_if && world -> broadphase_if -> generate)
            pair_count = world -> broadphase_if -> generate (world, world -> pair_buffer, mpe_max_broadphase_pairs, dt,
                                                         world -> broadphase_state);
        else
        pair_count = broadphase_generate_pairing (world, world -> pair_buffer, mpe_max_broadphase_pairs, dt);
    }
    int broadphase_pair_count = pair_count; /* saved for depenetration pass */
    world -> last_broadphase_pair_count = pair_count;
    /* Low-memory contract: degraded (pairless) tick instead of a crash when
     * scratch failed to allocate. TRUTH: sort_keys missing left order_out
     * uninitialized -> OOB solve. Check it too.
     * DESPOT-2026-10-07 P1-6: island/ccd/has_contact/id_cache/tick_v0 gaps
     * were individually NULL-tolerant but silently solved with stale flags
     * and no counter. Count degraded ticks loudly; twins with different OOM
     * histories must not diverge silently. */
    if ((!world -> pair_buffer) || (!world -> manifolds) || (!world -> manifold_awake) || (!world -> manifold_order) ||
        (!world -> manifold_sort_keys) || (!world -> pair_skipped) || (!world -> broadphase) || (!world -> island_parent) ||
        (!world -> island_label) || (!world -> island_awake_flags) || (!world -> ccd_time_remaining) ||
        (!world -> has_contact) || (!world -> tick_v0)) {
        static _Atomic unsigned long s_degraded_ticks = 0;
        unsigned long ndeg = MPE_ATOMIC_FETCH_ADD (&s_degraded_ticks, 1ul, MPE_ATOMIC_RELAXED) + 1ul;
        if ((ndeg <= 8) || ((ndeg % 1000) == 0)) {
            fprintf (stderr, "[mpe] DEGRADED tick %lu: scratch missing (low-mem); solving pairless, islands/ccd/contact flags stale\n",
                     ndeg);
        } return;
    } int manifold_count = 0;
    for (int q = 0; q < pair_count; q++) {
        world -> pair_skipped [q] = 0;
    } contact_cache_stats_reset (world);
    /* Zero every manifold slot before narrowphase: slots are malloc'd, not
     * calloc'd, so non-collided pairs would contain garbage contact_count
     * and object pointers. The parallel range zeroes per-slot; the serial
     * fallback (physics_world_process_pair) does not, so zero up front for
     * both paths. Bitwise identical since only collided pairs are written
     * afterwards and empty slots are awake-skipped below.
     * DESPOT-2026-10-19: capacity is grown to the pair count BEFORE this
     * memset. The old code memset pair_count slots and set manifold_count =
     * pair_count regardless of capacity, so a tick with more broadphase
     * pairs than manifold slots (biobuzz: 1141 pairs, 1024 slots) wrote
     * 638960 bytes past a 573440-byte heap block inside the step itself —
     * before mpe_parallel_narrowphase ever attempted a grow. That single
     * omission was the ASan heap-buffer-overflow and the glibc "free():
     * invalid pointer" abort in the MFS suite. */
    int slot_count = pair_count;
    if (slot_count > 0) {
        if (physics_world_ensure_manifold_capacity (world, slot_count) != 0) {
            /* Growable ceiling reached: the serial compactor handles the
             * tail and counts every collided overflow pair. */
            slot_count = world -> manifold_capacity;
            if (slot_count < 0) { slot_count = 0; }
        }
        if (slot_count > 0) {
            memset (world -> manifolds, 0, (size_t) slot_count * sizeof (collision_data));
            memset (world -> manifold_awake, 0, (size_t) slot_count);
        }
    }
    manifold_count = mpe_parallel_narrowphase (world, pair_count, slot_count, dt, step_cfg);
    /* Sleep-wake revisit fixpoint: pairs skipped above as both-asleep get
     * re-processed if either endpoint woke during the main loop (a woken
     * body with no manifold free-falls the whole tick otherwise). Each
     * pass only touches newly-active skipped pairs; cascades settle pass
     * by pass. Deterministic (pair order, wake-driven, bounded). */
    for (int revisit_pass = 0; revisit_pass < 16; revisit_pass++) {
        bool revisit_woke = false;
        for (int p = 0; p < pair_count; p++) {
            if (!world -> pair_skipped [p]) { continue; }
            int index_a = world -> pair_buffer [p].object_index_a;
            int index_b = world -> pair_buffer [p].object_index_b;
            if ((index_a < 0) || (index_a >= world -> body_count) || (index_b < 0) || (index_b >= world -> body_count)) {
                world -> pair_skipped [p] = 0;
                continue;
            } rigidbody *body_a = &world -> bodies [index_a];
            rigidbody *body_b = &world -> bodies [index_b];
            if ((body_a -> is_sleeping) && (body_b -> is_sleeping)) { continue; }
            bool slept_a = body_a -> is_sleeping;
            bool slept_b = body_b -> is_sleeping;
            physics_world_process_pair (world, index_a, index_b, dt, &manifold_count);
            world -> pair_skipped [p] = 0;
            if ((slept_a && !body_a -> is_sleeping) || (slept_b && !body_b -> is_sleeping)) { revisit_woke = true; }
        }
        if (!revisit_woke) { break; }
    }
    for (int i = 0; i < world -> body_count; i++) {
        rigidbody *rb = &world -> bodies [i];
        if (rb -> static_state) { continue; }
        if (rb -> no_collide) {
            continue; /* render-only proxies: no floor contact */
        }
        /* TRUTH: the infinite solver floor at y=0 is gated by static_plane_enabled.
         * When disabled, the boundary box provides a perfectly-plastic backstop
         * at y=0 with no friction or restitution. When enabled (default for GUI),
         * the floor is a real material contact with friction + restitution. */
        if (world -> static_plane_enabled) {
            /* Sync plane friction from the live per-world config every tick.
             * The persistent body would otherwise keep its initialisation
             * defaults (sphere 0.3/0.2), silently ignoring world.floor_*
             * (0.2/0.1) and live edits — a material lie in every floor
             * contact. The old thread-local proxy synced per call. */
            world -> static_plane_body.friction_static = step_cfg -> world.floor_friction_s;
            world -> static_plane_body.friction_kinetic = step_cfg -> world.floor_friction_k;
            bool was_sleeping = rb -> is_sleeping;
            if (rb -> type == object_composite) {
                /* MIG WP2 2.6: the plane path is the same primitive path
                 * per part; one manifold per touching part keeps each
                 * part's normal/contact frame separate. */
                mpe_composite_floor_batch floor_batch;
                mpe_composite_floor_detect (world, rb, step_cfg, &floor_batch);
                if (floor_batch.count > 0) {
                    if (was_sleeping && floor_batch.deepest > step_cfg -> depenetration.wake_depth_thresh) {
                        rigidbody_wake (rb);
                    } bool appended = false;
                    bool storage_failed = false;
                    for (int fk = 0; fk < floor_batch.count; fk++) {
                        if (manifold_count >= world -> manifold_capacity) {
                            if (storage_failed || physics_world_grow_manifolds (world) != 0) {
                                storage_failed = true;
                                world -> manifold_overflow_count++;
                                continue;
                            }
                        } collision_prepare_solver (world, &floor_batch.manifolds [fk],
                                                    &world -> manifolds [manifold_count], dt);
                        manifold_count++;
                        appended = true;
                    } if (appended && world -> has_contact) { world -> has_contact [i] = 1; }
                }
            } else if (rb -> type == object_hull) {
                /* MIG WP2.6 2.6.5: one hull floor manifold from the
                 * support-vertex walk (physics_world_hull_floor_detect;
                 * it counts the composite.hull refusal itself). */
                collision_data floor_collision = {0};
                if (physics_world_hull_floor_detect (world, rb, 0.0f, &floor_collision)) {
                    if (manifold_count >= world -> manifold_capacity) {
                        if (physics_world_grow_manifolds (world) != 0) {
                            world -> manifold_overflow_count++;
                            continue;
                        }
                    } float deepest = 0.0f;
                    for (int fi = 0; fi < floor_collision.contact_count; fi++) {
                        if (floor_collision.contacts [fi].penetration > deepest) { deepest = floor_collision.contacts [fi].penetration; }
                    }
                    if (was_sleeping && deepest > step_cfg -> depenetration.wake_depth_thresh) { rigidbody_wake (rb); }
                    collision_prepare_solver (world, &floor_collision, &world -> manifolds [manifold_count], dt);
                    manifold_count++;
                    if (world -> has_contact) { world -> has_contact [i] = 1; }
                }
            } else {
                collision_data floor_collision = {0};
                if (collision_static_plane_body (&world -> static_plane_body, rb, 0.0f, &floor_collision, step_cfg)) {
                    if (manifold_count >= world -> manifold_capacity) {
                        if (physics_world_grow_manifolds (world) != 0) {
                            world -> manifold_overflow_count++;
                            continue;
                        }
                    } float deepest = 0.0f;
                    for (int fi = 0; fi < floor_collision.contact_count; fi++) {
                        if (floor_collision.contacts [fi].penetration > deepest) { deepest = floor_collision.contacts [fi].penetration; }
                    }
                    if (was_sleeping && deepest > step_cfg -> depenetration.wake_depth_thresh) { rigidbody_wake (rb); }
                    collision_prepare_solver (world, &floor_collision, &world -> manifolds [manifold_count], dt);
                    manifold_count++;
                    if (world -> has_contact) { world -> has_contact [i] = 1; }
                }
            }
        }
    }
    /* Solver islands: union pairs + joints, then flag each manifold.
     * Fully-sleeping islands skip the iteration/relaxation loops below
     * (exact no-ops: zero velocity, zeroed inverses). */
    islands_build (world, world -> pair_buffer, pair_count);
    mpe_step_flag_manifolds_awake (world, manifold_count);
    collision_manifold_solve_order (world, world -> manifolds, manifold_count, world -> manifold_order);
    vector3 gravity = {0.0f, step_gravity, 0.0f};
    /* Snapshot start-of-tick velocities for exact free-flight. The analytic
     * position/velocity solution must start from v_pre; live velocity after
     * rb_integrate_velocity is v_post (forces already applied).
     * FIX-AUDIT-DESPOT: was >= mpe_max_bodies (always true today, since the
     * pool allocates exactly that — but a future smaller/tighter allocation
     * would silently skip the snapshot and integrate free-flight from
     * v_post, double-applying gravity. Gate on what we actually read. */
    if (world -> tick_v0 && world -> tick_v0_capacity >= world -> body_count) {
        for (int i = 0; i < world -> body_count; i++) {
            world -> tick_v0 [i] = world -> bodies [i].velocity;
        }
    }
    for (int i = 0; i < world -> body_count; i++) {
        rigidbody *rb = &world -> bodies [i];
        if ((rb -> static_state) || (rb -> is_sleeping) || (rb -> kinematic)) { continue; }
        /* DESPOT-2026-10-20 CCD-TOI GRAVITY: sweep-clamped bodies skip the
         * gravity push entirely. The sweep banked gravity over [0,toi] in
         * v_toi, and the post-impulse flight is constant-velocity by
         * engine convention, so [toi,dt] gravity has no owned window.
         * Forces/torques/damping below still integrate the full dt. */
        if (world -> ccd_time_remaining) {
            float rem = world -> ccd_time_remaining [i];
            if ((rem > 0.0f) && (rem < dt)) { continue; }
        }
        /* Gravity bypasses rb_apply_forces_perfect: it is not an external
         * force for the ext_force_this_tick gate (the analytic free-flight
         * path already integrates gravity exactly; only non-gravity forces
         * must force the constrained path). */
        rb -> force_accumulator = simd_add (rb -> force_accumulator, simd_scale (gravity, rb -> mass));
    }
    /* Deterministic retention factors (never libm pow: see det_math.h).
     *
     * DESPOT-2026-09-29: the drag value was passed to det_pow_retention
     * UNCLAMPED. det_pow_retention's contract is base in (0, 1.1]; anything
     * else takes the counted libm path, so a drag of 0 (or negative, or NaN)
     * -- a perfectly reachable state, since a world initialised before
     * mpe_config_init() runs sees a zeroed g_cfg -- silently escaped the
     * deterministic path on EVERY tick, once per world per step. Caught by
     * the new det-fallback assertion in mpe_test_end. Clamp to the contract
     * here, exactly as rb_integrate_position_exact_free_flight already does,
     * so an out-of-range config is corrected rather than desynchronised. */
    float drag_clamped = step_drag;
    if (!isfinite (drag_clamped) || drag_clamped <= 0.0f)
        drag_clamped = 1.0f;
    if (drag_clamped > 1.0f)
        drag_clamped = 1.0f;
    float linear_damping = (float) det_pow_retention ((double) drag_clamped, (double) dt);
    /* TRUTH: scale=1.0 means NO extra rotary damping (retention 1.0), even
     * when drag<1 damps translation. Air barely damps rotation; coupling
     * them (old: pow(drag*scale)) damped spin in vacuum whenever drag<1.
     * FIX-AUDIT: angular scale was hardcoded 0.97 (damped even at drag=1). */
    float ang_base = drag_clamped * step_ang_scale;
    if (!isfinite (ang_base) || ang_base <= 0.0f)
        ang_base = 1.0f;
    if (ang_base > 1.0f)
        ang_base = 1.0f;
    float angular_damping =
        (step_ang_scale >= 1.0f) ? 1.0f : (float) det_pow_retention ((double) ang_base, (double) dt);
    /* Springs before integration (weak-linked canonical entry). */
    if (mpe_springs_apply) { mpe_springs_apply (world, dt); }
    constraint_apply_motors (world, dt); /* MPE_FTC_067 */
    /* Phase-2: foreign forcefield / motor modules (pre-integration). */
    mpe_step_call_foreign_modules (world, dt, true);
    /* DESPOT-2026-10-20 CCD-TOI GRAVITY: a sweep-clamped body arrives here
     * with velocity already advanced to the touch speed (collision_ccd
     * applies v_toi, gravity over [0,toi] banked) and its position
     * teleported to the hit. Integrating gravity over the full dt on top of
     * v_toi double-counts [0,toi] and adds [toi,dt], so the Poisson gate
     * recorded one extra g*dt on every CCD-bounced impact (flat slam:
     * approach read 2.96 vs the physical 2.80 touch speed; bounce exits
     * hot, chains run a bounce long, and marginal sub-threshold impacts
     * turned into bounces). The gravity increment is therefore excluded
     * for clamped bodies at the force_accumulator add site above; this
     * integrate keeps the FULL dt so motors, springs, damping, the
     * torque-free L-conservation rotor advance, and the per-tick
     * quaternion normalisation are preserved verbatim (skipping them -
     * earlier body_dt=0 attempt - stalled MFS motor pivots and left the
     * quaternion off its normalisation fixed point, which the scene
     * round-trip's unconditional renormalisation then flagged). The
     * post-impulse flight stays constant-velocity by engine convention, so
     * [toi,dt] gravity is deliberately unowned. Non-clamped bodies
     * (rem == dt) integrate the full tick with gravity as before. */
    for (int i = 0; i < world -> body_count; i++) {
        rb_integrate_velocity (&world -> bodies [i], dt, linear_damping, angular_damping);
    }
    /* Sleeping bodies keep real mass/inertia (no staticize mutation).
     * The solver treats them as infinite mass via rigidbody_effective_*
     * helpers, so observable state is never corrupted mid-tick and the
     * path is thread-safe. Zero stale velocities/accumulators only. */
    for (int sleep_index = 0; sleep_index < world -> body_count; sleep_index++) {
        rigidbody *sleep_body = &world -> bodies [sleep_index];
        if ((sleep_body -> is_sleeping) && (!sleep_body -> static_state)) {
            sleep_body -> velocity = vector3_zero ();
            sleep_body -> angular_velocity = vector3_zero ();
            sleep_body -> force_accumulator = vector3_zero ();
            sleep_body -> torque_accumulator = vector3_zero ();
        }
    }
    /* Poisson gate must see post-force-integration approach speed (prepare
     * ran pre-gravity, stale by g*dt). Refresh from current velocities
     * BEFORE any iteration touches accumulators. Refreshing after the solve
     * would read the post-solve residual (~0) and gate every bounce off.
     * DESPOT-2026-10-20 CCD-TOI GRAVITY: clamped-tick gravity was excluded
     * at the force_accumulator add site above, so the gate and the Newton
     * bound measure the touch-speed approach instead of touch + g*dt. */
    collision_refresh_impact_velocities (world -> manifolds, manifold_count);
    /* TRUTH: islands were built before motors/forces ran; motor wake flips
     * bodies awake after the flags were computed, leaving joint+contact
     * solves skipped for a tick (broken hinge response). Rebuild cheaply
     * (O(pairs), idempotent) so iteration skips match live sleep state. */
    /* TRUTH: islands were built before motors/forces ran; motor wake flips
     * bodies awake after the flags were computed, leaving joint+contact
     * solves skipped for a tick (broken hinge response). Rebuild cheaply
     * (O(pairs), idempotent) so iteration skips match live sleep state. */
    islands_build (world, world -> pair_buffer, pair_count);
    mpe_step_flag_manifolds_awake (world, manifold_count);
    /* Solver iterations with joint coupling inside the loop. */
    int solver_iterations = step_iterations;
    if (solver_iterations < 1)
        solver_iterations = 1;
    if (solver_iterations > 128)
        solver_iterations = 128;
    /* Joint angle/slide integration once per tick (not per iteration). */
    constraint_pre_step_all (world, dt);
    /* DESPOT-2026-10-14: full multicore solver. Islands are independent
     * connected components (no shared bodies), so each island's manifolds
     * are solved by one thread in sort order. Disjoint writes + fixed
     * within-island order = bitwise identical to serial at any thread count.
     * constraint_solve_all stays serial (O(mpe_max_joints)=1024 scan,
     * measured negligible; joints only connect bodies within one island,
     * so no cross-island dependency exists). */
    for (int iter = 0; iter < solver_iterations; iter++) {
        mpe_parallel_solve_sweep (world, manifold_count, dt, iter, step_cfg, false);
        /* Solve joints inside iteration loop for friction transfer through hinges. */
        constraint_solve_all (world, dt);
    }
    /* DESPOT-2026-10-08 buckling guard: tall stacks (8+ bodies) with
     * residual deep overlap after the budgeted iterations get up to 32
     * extra iterations (capped 128 total visits logic). Measured lever:
     * 10-cube at gravity -17 goes 0.31 m@64 -> 0.00 m@128. Cheap when
     * settled (worst<=slop skips), pays only during buckle transients. */
    if ((world -> body_count >= 8) && (solver_iterations < 128) && (manifold_count > 0)) {
        float worst_pen = 0.0f;
        for (int m = 0; m < manifold_count; m++) {
            for (int c = 0; c < world -> manifolds [m].contact_count; c++) {
                float p = world -> manifolds [m].contacts [c].penetration;
                if (isfinite (p) && (p > worst_pen)) { worst_pen = p; }
            }
        }
            if (worst_pen > step_cfg -> solver.penetration_slop) {
                int extra = 128 - solver_iterations;
                if (extra > 32) { extra = 32; }
                for (int iter = 0; iter < extra; iter++) {
                    mpe_parallel_solve_sweep (world, manifold_count, dt, solver_iterations + iter, step_cfg, false);
                    constraint_solve_all (world, dt);
                }
            }
    }
    /* Axis-drift correction lives at end-of-step (after the final joint
     * pass): the correction is a velocity tweak that must survive into
     * integration. Placed here it was eaten by the post-Poisson joint solve
     * (which kills ALL perp relative velocity, including the correction),
     * so it never acted — see the post-rolling pass below. */
    /* Poisson restitution: one e-over-compression payment per fresh impact,
     * then short relaxation so friction sees post-bounce velocities. */
    /* TRUTH: joints must see post-bounce velocities too. Poisson without a
     * joint relaxation leaves hinges/welds broken for a tick. */
    mpe_step_poisson (world, world -> manifolds, manifold_count, step_cfg);
    constraint_solve_all (world, dt);
    /* DESPOT-2026-10-14: parallel friction-only relaxation sweeps (per-island,
     * disjoint writes, sort order preserved within each island). */
    for (int relax_iter = 0; relax_iter < 2; relax_iter++) {
        mpe_parallel_solve_sweep (world, manifold_count, dt, relax_iter, step_cfg, true);
    }
    /* DESPOT-2026-10-12 (ABUSE-TILT): joints re-equalize after the
     * relaxation passes (see the post-rolling pass below for the mechanism);
     * this one covers the relax->split window, the later one the
     * rolling->integrate window. Both are velocity-only re-equalizations
     * (no drift, no pre-step). */
    constraint_solve_all (world, dt);
    /* Split impulse: positional depenetration with zero velocity change.
     * The velocity solve above is compression-only, so contact impulses
     * (and the Coulomb clamp) stay honest. */
    mpe_step_split (world, world -> manifolds, manifold_count, dt, step_cfg);
    /* TRUTH: split can wake sleepers (deep overlap) that were skipped as
     * both-asleep with no manifold. They would integrate with has_contact=0
     * (false free-flight). Re-process newly-awake skipped pairs now so they
     * get a manifold + has_contact before integration.
     * FIX-AUDIT-DESPOT late-manifold-after-split: those fresh manifolds
     * missed the iteration/Poisson/split passes above (they solve cold next
     * tick = one-tick-late response + a first-tick penetration souvenir).
     * Run one extra resolve+Poisson+split iteration over exactly the late
     * range now (has_contact is already set by process_pair above, so the
     * gravity gate stays honest). Rolling below then sees their normals. */
    int late_manifold_start = manifold_count;
    for (int p = 0; p < pair_count; p++) {
        if (!world -> pair_skipped [p]) { continue; }
        int ia = world -> pair_buffer [p].object_index_a;
        int ib = world -> pair_buffer [p].object_index_b;
        if (ia < 0 || ia >= world -> body_count || ib < 0 || ib >= world -> body_count) {
            world -> pair_skipped [p] = 0;
            continue;
        }
        if (world -> bodies [ia].is_sleeping && world -> bodies [ib].is_sleeping) { continue; }
        physics_world_process_pair (world, ia, ib, dt, &manifold_count);
        world -> pair_skipped [p] = 0;
    }
    if (manifold_count > late_manifold_start) {
        for (int m = late_manifold_start; m < manifold_count; m++) {
            world -> manifold_awake [m] = 1; /* newly-awake by construction */
        }
        for (int m = late_manifold_start; m < manifold_count; m++) {
            mpe_step_resolve (world, &world -> manifolds [m], dt, false, 0, step_cfg);
            mpe_step_resolve (world, &world -> manifolds [m], dt, false, 1, step_cfg);
        }
        mpe_step_poisson (world, &world -> manifolds [late_manifold_start], manifold_count - late_manifold_start,
                          step_cfg);
        mpe_step_split (world, &world -> manifolds [late_manifold_start], manifold_count - late_manifold_start, dt,
                        step_cfg);
    }
    /* Rolling resistance once per tick (uses solved normal impulses). */
    mpe_step_rolling (world, world -> manifolds, manifold_count, dt, step_cfg);
    /* DESPOT-2026-10-12 (ABUSE-TILT root cause): rolling's spin-decay about
     * the contact normal cannot distinguish rigid co-rotation from slip, so
     * on a hinge-driven wheel it strips the co-yaw the joint just gave it
     * (measured post-final relperp 0.000 -> post-rolling 0.92 on a sustained
     * combined drive; the chassis keeps yawing, the wheel does not follow,
     * integration bakes the difference into hinge tilt at the chassis yaw
     * rate). Same single-principle pass as above: re-equalize after every
     * out-of-loop velocity stage, so the joints — not contacts, not rolling —
     * have the final word going into integration. Velocity-only solve, no
     * drift, no pre-step. Hinge-parallel spin rolling legitimately removed
     * is untouched by axis/P2P rows (parallel component is not constrained);
     * co-rotation the joint owns is restored. */
    constraint_solve_all (world, dt);
    /* Axis-drift correction, relocated end-of-step (DESPOT-2026-10-12,
     * ABUSE-TILT): exactly once per tick, after every velocity-touching
     * stage, so the corrective velocity survives into integration. In its
     * old slot (pre-Poisson) the very next joint solve removed the
     * correction as "spin error" and orientation error never decayed
     * (measured: drift injected 1.5 rad/s at 14° tilt, post-joint read
     * 0.001 — full erasure). With joints holding per-tick equality AND
     * drift decaying residual angle, tilt is bounded with recovery:
     * error *= (1-beta) per tick plus only fresh injection. Still never
     * inside the iteration loop (pendulum-9x energy pump rule stands). */
    constraint_correct_axis_drift_all (world, dt);
    /* Sleeping bodies already hold real mass (no staticize was applied),
     * so no restore is needed. Keep velocities pinned at zero. */
    for (int sleep_restore_index = 0; sleep_restore_index < world -> body_count; sleep_restore_index++) {
        rigidbody *sleep_restore_body = &world -> bodies [sleep_restore_index];
        if ((sleep_restore_body -> is_sleeping) && (!sleep_restore_body -> static_state)) {
            sleep_restore_body -> velocity = vector3_zero ();
            sleep_restore_body -> angular_velocity = vector3_zero ();
        }
    } contact_cache_save (world, world -> manifolds, manifold_count);
    /* TRUTH P0-1+P0-3: integrate CCD remainder with exact free-flight
     * for contact-free AND joint-free bodies (analytic gravity+drag solution),
     * original symplectic Euler for constrained bodies.
     * PHYSICS-FIX: joint membership is precomputed once per tick into a
     * body-index bitmap O(J+B). The old per-body scan was O(B*J) (16Kx1K
     * worst case) and, worse, looped ji<count indexing pool[ji], missing
     * active joints past holes left by removals. Full-pool scan + id cache
     * is both faster and correct. */
    static _Thread_local unsigned char joint_membership [mpe_max_bodies];
    {
        int n = world -> body_count;
        if (n > mpe_max_bodies) { n = mpe_max_bodies; }
        /* FIX-AUDIT-DESPOT tail safety: was memset(..., n) — a tick with
         * FEWER bodies than the last tick left stale 1s past n, so removed
         * bodies' indices kept other bodies "jointed" (wrong symplectic
         * path, gravity double-count). The array is thread-local and fixed
         * size: always clear the whole thing. */
        memset (joint_membership, 0, sizeof (joint_membership));
        for (int ji = 0; ji < mpe_max_joints; ji++) {
            if (!world -> revolute_constraints [ji].is_active) { continue; }
            int ia = physics_world_index_by_id (world, world -> revolute_constraints [ji].body_id_a);
            int ib = physics_world_index_by_id (world, world -> revolute_constraints [ji].body_id_b);
            if (ia >= 0 && ia < n) { joint_membership [ia] = 1; }
            if (ib >= 0 && ib < n) { joint_membership [ib] = 1; }
        }
        for (int ji = 0; ji < mpe_max_joints; ji++) {
            if (!world -> spring_joints [ji].is_active) { continue; }
            int ia = physics_world_index_by_id (world, world -> spring_joints [ji].object_id_a);
            int ib = physics_world_index_by_id (world, world -> spring_joints [ji].object_id_b);
            if (ia >= 0 && ia < n) { joint_membership [ia] = 1; }
            if (ib >= 0 && ib < n) { joint_membership [ib] = 1; }
        }
    }
    /* DESPOT-2026-10-14: parallel per-body integration. Disjoint writes per
     * body, order-independent within the integration phase (each body's
     * new state depends only on its own pre-integration state). Bitwise
     * identical to serial at any thread count. */
    mpe_parallel_integrate (world, dt, step_cfg, joint_membership);
    /* World-edge safety net, same as the legacy path: perfectly plastic,
     * fires only past emergency slop (solver owns all normal contact). */
    bool a3_boundary_moved_any = false;
    /* DESPOT-2026-10-03: gated on boundary.safety_net_enabled (default 1, so
     * shipped behaviour is byte-identical). The net is a fail-safe; a
     * fail-safe that is unconditionally on makes "nothing fell through the
     * world" vacuous, which is why the f10/f11 `fallen` counters could never
     * fire. A test now switches it off to measure the contact solver's own
     * ability to hold a body up. See config/mpe_config.h. */
    if (step_cfg -> boundary.safety_net_enabled) {
        for (int i = 0; i < world -> body_count; i++) {
            vector3 a3_pre_boundary_position = world -> bodies [i].position;
            boundary_apply_box_world_cfg (&world -> bodies [i], (vector3) {-250, 0, -250}, (vector3) {250, 500, 250},
                                          step_cfg, world);
            if (vector3_length_squared (simd_sub (world -> bodies [i].position, a3_pre_boundary_position)) >
                0.000001f) {
                a3_boundary_moved_any = true;
            }
        }
    }
    /* Positional depenetration pass (like legacy path). */
    a3_positional_depenetration_pass_dt (world, world -> pair_buffer, &broadphase_pair_count, a3_boundary_moved_any, dt);
    /* Phase-2: foreign post-step modules (loggers, correctors). */
    mpe_step_call_foreign_modules (world, dt, false);
    /* AUDIT: the warm-start cache is deliberately NOT cleared here. Cached
     * contact points are stored in BODY-LOCAL space, which rigid
     * translation (the only thing depenetration/boundary do) preserves
     * exactly — a persistent contact matches the same material point next
     * tick regardless of where the body moved. An earlier revision cleared
     * unconditionally ("stale positions"), which silently disabled warm
     * starting engine-wide (F9 showed hits=0 forever): every tick solved
     * cold, tall stacks crept and collapsed at low iteration counts.
     * Rotation-driven contact migration still misses naturally via the
     * match distance, which is the correct invalidation path. Scene loads
     * (pool invalidation) keep their explicit clears. */
    world -> last_manifold_count = manifold_count;
} /* R3-07: Containment walls.
 *
 * Adds four static cube bodies around the playable area.
 * The walls are placed just outside the half-extents so the
 * playable interior is exactly half_width x half_depth.
 *
 * Wall layout (top view):
 *
 *        north wall
 *   +-----------------+
 *   |                 |
 * w |    playable     | e
 * e |     area        | a
 * s |                 | s
 * t |                 | t
 *   +-----------------+
 *        south wall
 */
int physics_world_add_boundary_walls (physics_world *world, float half_width, float half_depth, float wall_height,
                                      float wall_thickness) {
    if (!world) { return -1; }
    if ((half_width <= 0.0f) || (half_depth <= 0.0f) || (wall_height <= 0.0f) || (wall_thickness <= 0.0f)) { return -1; }
    float hy = wall_height * 0.5f;
    float ht = wall_thickness * 0.5f;
    /* PHYSICS-NOTE: long axes overshoot by the full wall_thickness so
     * corners overlap (no escape gaps). Overlap is intentional
     * containment, not asymmetry: N/S span half_width+thickness in X,
     * E/W span half_depth+thickness in Z, thin axes use half-thickness. */
    /* North wall: +Z side */
    int north = physics_world_add_cube (world, (vector3) {0.0f, hy, half_depth + ht},
                                        (vector3) {half_width + wall_thickness, hy, ht}, 0.0f);
    /* South wall: -Z side */
    int south = physics_world_add_cube (world, (vector3) {0.0f, hy, -(half_depth + ht)},
                                        (vector3) {half_width + wall_thickness, hy, ht}, 0.0f);
    /* East wall: +X side */
    int east = physics_world_add_cube (world, (vector3) {half_width + ht, hy, 0.0f},
                                       (vector3) {ht, hy, half_depth + wall_thickness}, 0.0f);
    /* West wall: -X side */
    int west = physics_world_add_cube (world, (vector3) {-(half_width + ht), hy, 0.0f},
                                       (vector3) {ht, hy, half_depth + wall_thickness}, 0.0f);
    if ((north < 0) || (south < 0) || (east < 0) || (west < 0)) { return -1; }
    return 0;
}
