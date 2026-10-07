/* Canonical explicit-world step pipeline. Body storage, contacts, solver
 * scratch, config, joints, and warm-start cache are owned per world. The GTK
 * callback still has a parallel fixed-step loop in simulation_physics_loop.c;
 * shared pair dispatch does not make those whole pipelines identical.
 * DESPOT-2026-10-08 threading: order-independent per-body phases (sanitize,
 * CCD remainder init) can run MT via mpe_parallel_for below (MPE_THREADS,
 * default 1 = deterministic single-threaded). Pair/solve phases stay
 * single-threaded for bitwise-identical manifold order. */
#include "physics_world.h"
#include "mpe_registry.h"
#include "mpe_loader.h"
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
#include <stdlib.h>
#include <stdio.h>
/* Minimal deterministic thread pool: disjoint index ranges, no shared writes
 * except distinct bodies. Order-independent, so bitwise identical to serial. */
typedef struct {
    physics_world *world;
    int start;
    int end;
} mpe_slice_t;
static void *mpe_sanitize_slice (void *arg) {
    mpe_slice_t *s = (mpe_slice_t *) arg;
    for (int i = s -> start; i < s -> end; i++) {
        rigidbody_sanitize (&s -> world -> bodies [i]);
    } return NULL;
}
static void mpe_parallel_sanitize (physics_world *world) {
    int n = world -> body_count;
    if (n <= 0) { return; }
    const char *env = getenv ("MPE_THREADS");
    int threads = env ? atoi (env) : 1;
    if (threads < 1) { threads = 1; }
    if (threads > 8) { threads = 8; }
    if ((threads <= 1) || (n < 64)) {
        for (int i = 0; i < n; i++) {
            rigidbody_sanitize (&world -> bodies [i]);
        } return;
    } pthread_t tids [8];
    mpe_slice_t slices [8];
    int chunk = (n + threads - 1) / threads;
    int launched = 0;
    for (int t = 0; t < threads; t++) {
        int s = t * chunk;
        int e = s + chunk;
        if (e > n) { e = n; }
        if (s >= e) { break; }
        slices [t].world = world;
        slices [t].start = s;
        slices [t].end = e;
        if (pthread_create (&tids [t], NULL, mpe_sanitize_slice, &slices [t]) != 0) {
            /* Fallback: run slice serially on creation failure. */
            mpe_sanitize_slice (&slices [t]);
            tids [t] = 0;
        } else { launched++; }
    }
    /* Join only launched threads; serial-fallback slices already done. */
    for (int t = 0; t < threads; t++) {
        if ((slices [t].start < slices [t].end) && tids [t]) { pthread_join (tids [t], NULL); }
    } (void) launched;
} /* Canonical spring pass is weakly linked so spring-less headless test
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
int physics_world_attach_module (physics_world *world, const mpe_module_desc_t *desc) {
    if (!world || !desc || desc -> abi != MPE_MODULE_ABI)
        return -1;
    for (int i = 0; i < world -> tick_module_count; i++)
        if (world -> tick_modules [i] == desc)
        return i;
    if (world -> tick_module_count >= MPE_MAX_TICK_MODULES)
        return -1;
    void *st = NULL;
    if (desc -> attach && desc -> attach (world, &st) != 0)
        return -1;
    world -> tick_modules [world -> tick_module_count] = desc;
    world -> tick_module_state [world -> tick_module_count] = st;
    mpe_loader_retain_module (desc);
    return world -> tick_module_count++;
}
int physics_world_detach_module (physics_world *world, const char *name) {
    if (!world || !name)
        return -1;
    for (int i = 0; i < world -> tick_module_count; i++) {
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
            return 0;
        }
    } return -1;
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
bool mpe_shape_dispatch (physics_world *world, rigidbody *a, rigidbody *b, collision_data *out) {
    if (!a || !b || !out)
        return false;
    /* Render-only proxies never collide (semantic backstop for direct
     * dispatch users; broadphase/floor/CCD skip them earlier for perf). */
    if (a -> no_collide || b -> no_collide)
        return false;
    /* Builtins are once-registered at world init; no per-pair call. */
    int ca = (a -> type == object_custom) ? a -> custom_shape : -1;
    int cb = (b -> type == object_custom) ? b -> custom_shape : -1;
    mpe_collide_fn fn = mpe_find_pair_handler ((int) a -> type, (int) b -> type, ca, cb);
    if (fn) {
        if (!fn (a, b, out, world))
            return false;
        if (!a3_sanitize_plugin_manifold (out, a, b))
            return false;
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
        out -> normal_vector = vector3_scaling (tmp.normal_vector, -1.0f);
        out -> object_a = a;
        out -> object_b = b;
        /* Swap body-local frames: contacts were solved in (b,a) space. */
        for (int i = 0; i < out -> contact_count; i++) {
            vector3 t = out -> contacts [i].local_position_a;
            out -> contacts [i].local_position_a = out -> contacts [i].local_position_b;
            out -> contacts [i].local_position_b = t;
            out -> contacts [i].tangent_vector = vector3_scaling (out -> contacts [i].tangent_vector, -1.0f);
            /* tangent2 intentionally NOT negated: t2 = n×t1 is invariant
             * under the double flip, and prepare rebuilds it from
             * position+normal anyway (collision_solver.c). */
            vector3 r = out -> contacts [i].ra;
            out -> contacts [i].ra = out -> contacts [i].rb;
            out -> contacts [i].rb = r;
        } return true;
    } return false;
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
void physics_world_clear (physics_world *world) {
    if (!world) { return; }
    world -> body_count = 0;
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
        /* TRUTH: three-gate wake (any fires).
             * 1. NEW EDGE (pair novelty via the contact cache): either id
             *    pair has no entry from a prior tick. First touch wakes at
             *    ANY speed — this closes the slow-pusher ghost (creeping
             *    stack partner, 0.05 m/s kinematic platform) that pure
             *    velocity gates miss, and unlike per-body flags it is not
             *    blinded by floor contacts every rester holds. Persistent
             *    resting pairs have entries, so settled stacks proceed to
             *    sleep instead of being kept awake forever (which pumped
             *    the F10 10-stack to 13 m/s via never-sleeping micro-motion).
             * 2. FAST OTHER: original velocity gate (backstop for fast bodies
             *    that already held contacts, e.g. debris sliding along a
             *    stack before striking a sleeper).
             * 3. DEEP: overlap beyond wake_depth_thresh, even vs statics. */
        bool a_was_sleeping = body_a -> is_sleeping;
        bool b_was_sleeping = body_b -> is_sleeping;
        if ((a_was_sleeping) && (b_was_sleeping)) {
            /* Both sleeping: nothing to wake (depth backstop below still
                 * applies via the revisit/split/depen paths). */
        } else {
            bool new_edge = !contact_cache_has_pair (world, body_a -> object_id, body_b -> object_id);
            if ((a_was_sleeping) && (!body_b -> static_state) && new_edge) { rigidbody_wake (body_a); }
            if ((b_was_sleeping) && (!body_a -> static_state) && new_edge) { rigidbody_wake (body_b); }
            float wake_lin_sq = mpe_world_cfg (world) -> sleep.wake_linear_thresh_sq;
            float wake_ang_sq = mpe_world_cfg (world) -> sleep.wake_angular_thresh_sq;
            bool a_fast = (!a_was_sleeping) && ((vector3_length_squared (body_a -> velocity) > wake_lin_sq) ||
                                                (vector3_length_squared (body_a -> angular_velocity) > wake_ang_sq));
            bool b_fast = (!b_was_sleeping) && ((vector3_length_squared (body_b -> velocity) > wake_lin_sq) ||
                                                (vector3_length_squared (body_b -> angular_velocity) > wake_ang_sq));
            if ((a_was_sleeping) && (!body_b -> static_state) && b_fast) { rigidbody_wake (body_a); }
            if ((b_was_sleeping) && (!body_a -> static_state) && a_fast) { rigidbody_wake (body_b); }
            /* Wake on significant overlap growth, even against static
                 * geometry: a sleeper ground into a static surface by a new
                 * persistent force (e.g. a spawned overlap, a tilted stack
                 * settling) must rejoin the solve. Resting contact within
                 * slop never reaches this bar. */
            {
                float deepest = 0.0f;
                for (int wi = 0; wi < narrowphase_collision.contact_count; wi++) {
                    if (narrowphase_collision.contacts [wi].penetration > deepest) { deepest = narrowphase_collision.contacts [wi].penetration; }
                }
                if (deepest > mpe_world_cfg (world) -> depenetration.wake_depth_thresh) {
                    if (a_was_sleeping) { rigidbody_wake (body_a); }
                    if (b_was_sleeping) { rigidbody_wake (body_b); }
                }
            } collision_prepare_solver (world, &narrowphase_collision, &world -> manifolds [(*manifold_count_ptr)], dt);
            (*manifold_count_ptr)++;
            /* TRUTH P0-1: contact flag for gravity-exactness gating. */
            if (world -> has_contact) {
                if ((index_a >= 0) && (index_a < world -> body_count)) { world -> has_contact [index_a] = 1; }
                if ((index_b >= 0) && (index_b < world -> body_count)) { world -> has_contact [index_b] = 1; }
            }
        }
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
void physics_world_step (physics_world *world, float dt) {
    if ((!world) || (!world -> bodies) || (!(dt > 0.0f)) || (!isfinite (dt)) || (world -> body_count <= 0)) { return; }
    /* DESPOT-2026-10-07 LIE-05: fixed-dt is caller discipline. Bitwise
     * determinism requires identical dt sequences; variable-dt callers
     * silently desync (damping pow(drag,dt), CCD sweep*dt, rotor |w|*dt/2).
     * The GUI loop is fixed 1/60; headless callers must pin dt=1/60.
     * Non-1/60 dt still steps (no breakage) but is LOUD so a variable-dt
     * harness cannot claim determinism. */
    if ((dt < 0.016666f) || (dt > 0.016668f)) {
        static int s_dt_warned = 0;
        if (!s_dt_warned) {
            s_dt_warned = 1;
            fprintf (stderr, "[mpe] NON-CANONICAL dt=%.6f (canonical 1/60=0.016667); determinism twins must use identical dt\n",
                     (double) dt);
        }
    }
    /* TRUTH: spiral-of-death guard rescales time (dt>0.1s clamped, sim lags
     * wall) instead of substepping. Substepping would preserve time at the
     * cost of unbounded catch-up work; clamping bounds work and keeps every
     * tick at fixed dt (determinism's first requirement). Time dilation
     * under extreme lag is documented behavior, not hidden. */
    if (dt > 0.1f) { dt = 0.1f; }
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
    collision_ccd_sweep_clamp_world (world, dt);
    int pair_count = 0;
    if (world -> body_count >= 2) {
        if (world -> broadphase_if && world -> broadphase_if -> generate)
            pair_count = world -> broadphase_if -> generate (world, world -> pair_buffer, mpe_max_broadphase_pairs, dt,
                                                         world -> broadphase_state);
        else
        pair_count = broadphase_generate_pairing (world, world -> pair_buffer, mpe_max_broadphase_pairs, dt);
    }
    int broadphase_pair_count = pair_count; /* saved for depenetration pass */
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
        static unsigned long s_degraded_ticks = 0;
        s_degraded_ticks++;
        if ((s_degraded_ticks <= 8) || ((s_degraded_ticks % 1000) == 0)) {
            fprintf (stderr, "[mpe] DEGRADED tick %lu: scratch missing (low-mem); solving pairless, islands/ccd/contact flags stale\n",
                     s_degraded_ticks);
        } return;
    } int manifold_count = 0;
    for (int q = 0; q < pair_count; q++) {
        world -> pair_skipped [q] = 0;
    } contact_cache_stats_reset (world);
    for (int p = 0; p < pair_count; p++) {
        int index_a = world -> pair_buffer [p].object_index_a;
        int index_b = world -> pair_buffer [p].object_index_b;
        if ((index_a < 0) || (index_a >= world -> body_count) || (index_b < 0) || (index_b >= world -> body_count)) { continue; }
        rigidbody *body_a = &world -> bodies [index_a];
        rigidbody *body_b = &world -> bodies [index_b];
        if ((body_a -> is_sleeping) && (body_b -> is_sleeping)) {
            world -> pair_skipped [p] = 1;
            continue;
        } physics_world_process_pair (world, index_a, index_b, dt, &manifold_count);
    }
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
    /* Solver islands: union pairs + joints, then flag each manifold.
     * Fully-sleeping islands skip the iteration/relaxation loops below
     * (exact no-ops: zero velocity, zeroed inverses). */
    islands_build (world, world -> pair_buffer, pair_count);
    for (int m = 0; m < manifold_count; m++) {
        rigidbody *ma = world -> manifolds [m].object_a;
        rigidbody *mb = world -> manifolds [m].object_b;
        world -> manifold_awake [m] = (unsigned char) (islands_body_awake (world, ma) || islands_body_awake (world, mb));
    } collision_manifold_solve_order (world, world -> manifolds, manifold_count, world -> manifold_order);
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
        rb_apply_forces_perfect (rb, vector3_scaling (gravity, rb -> mass));
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
    /* Phase-2: foreign forcefield / motor modules (pre-integration).
     * Snapshot the table: a hook may attach/detach (structural change
     * deferred in effect to next tick — this loop runs the snapshot). */
    {
        const mpe_module_desc_t *mods [16];
        void *states [16];
        int nmods = world -> tick_module_count < 16 ? world -> tick_module_count : 16;
        for (int mi = 0; mi < nmods; mi++) {
            mods [mi] = world -> tick_modules [mi];
            states [mi] = world -> tick_module_state [mi];
        }
        for (int mi = 0; mi < nmods; mi++) {
            if (!mods [mi] || !mods [mi] -> pre_step)
                continue;
            /* Re-resolve: a hook earlier in this loop may have detached this
             * one, freeing states[mi]. See a3_module_live_state. A module
             * that legitimately holds NULL state is distinguished from one
             * that is no longer attached, so a NULL-state module detached
             * mid-tick is still skipped. */
            void *live = NULL;
            if (!physics_world_module_live_state (world, mods [mi], &live))
                continue;
            mods [mi] -> pre_step (world, dt, live);
            (void) states;
        }
    }
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
     * would read the post-solve residual (~0) and gate every bounce off. */
    collision_refresh_impact_velocities (world -> manifolds, manifold_count);
    /* TRUTH: islands were built before motors/forces ran; motor wake flips
     * bodies awake after the flags were computed, leaving joint+contact
     * solves skipped for a tick (broken hinge response). Rebuild cheaply
     * (O(pairs), idempotent) so iteration skips match live sleep state. */
    islands_build (world, world -> pair_buffer, pair_count);
    for (int m = 0; m < manifold_count; m++) {
        rigidbody *ma = world -> manifolds [m].object_a;
        rigidbody *mb = world -> manifolds [m].object_b;
        world -> manifold_awake [m] = (unsigned char) (islands_body_awake (world, ma) || islands_body_awake (world, mb));
    }
    /* Solver iterations with joint coupling inside the loop. */
    int solver_iterations = step_iterations;
    if (solver_iterations < 1)
        solver_iterations = 1;
    if (solver_iterations > 128)
        solver_iterations = 128;
    /* Joint angle/slide integration once per tick (not per iteration). */
    constraint_pre_step_all (world, dt);
    for (int iter = 0; iter < solver_iterations; iter++) {
        for (int o = 0; o < manifold_count; o++) {
            int m = world -> manifold_order [o];
            if (!world -> manifold_awake [m])
                continue;
            /* TRUTH: two visits per iteration (128 total at default 64, not
             * 64: the count knob undercounts by design for local coupling).
             * Convergent (no energy), just expensive. */
            mpe_step_resolve (world, &world -> manifolds [m], dt, false, iter, step_cfg);
            mpe_step_resolve (world, &world -> manifolds [m], dt, false, iter + 1, step_cfg);
        }
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
                for (int o = 0; o < manifold_count; o++) {
                    int m = world -> manifold_order [o];
                    if (!world -> manifold_awake [m])
                        continue;
                    mpe_step_resolve (world, &world -> manifolds [m], dt, false, solver_iterations + iter, step_cfg);
                    mpe_step_resolve (world, &world -> manifolds [m], dt, false, solver_iterations + iter + 1, step_cfg);
                } constraint_solve_all (world, dt);
            }
        }
    }
    /* Axis-drift correction: exactly once per tick, never in the loop. */
    constraint_correct_axis_drift_all (world, dt);
    /* Poisson restitution: one e-over-compression payment per fresh impact,
     * then short relaxation so friction sees post-bounce velocities. */
    /* TRUTH: joints must see post-bounce velocities too. Poisson without a
     * joint relaxation leaves hinges/welds broken for a tick. */
    mpe_step_poisson (world, world -> manifolds, manifold_count, step_cfg);
    constraint_solve_all (world, dt);
    for (int relax_iter = 0; relax_iter < 2; relax_iter++) {
        for (int o = 0; o < manifold_count; o++) {
            int m = world -> manifold_order [o];
            if (!world -> manifold_awake [m])
                continue;
            mpe_step_resolve (world, &world -> manifolds [m], dt, true, relax_iter, step_cfg);
        }
    }
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
    for (int i = 0; i < world -> body_count; i++) {
        float step_dt = dt;
        if ((world -> ccd_time_remaining) && (world -> ccd_time_remaining [i] < step_dt) &&
            (world -> ccd_time_remaining [i] > 0.0f)) {
            step_dt = world -> ccd_time_remaining [i];
        } rigidbody *ib = &world -> bodies [i];
        bool has_joint = (i < mpe_max_bodies) ? (joint_membership [i] != 0) : false;
        bool contact_free = ((world -> has_contact) ? (world -> has_contact [i] == 0) : false);
        /* DESPOT-2026-10-07 LIE-04: free_flight must be FALSE unless the
         * tick_v0 restore actually ran. The old default (has_contact NULL
         * -> true) treated post-force v_post as v(0) in the analytic path,
         * double-applying gravity (v error g*dt ~0.08 m/s per tick).
         * Degraded/low-mem ticks now take the safe symplectic path. */
        bool free_flight = contact_free && !has_joint;
        bool have_v0 = (world -> tick_v0 && (world -> tick_v0_capacity >= world -> body_count));
        if (free_flight && !have_v0) { free_flight = false; }
        /* FIX-AUDIT-DESPOT: capacity gate matches the snapshot gate above
         * (>= live body_count, not >= mpe_max_bodies). */
        if (free_flight && have_v0) {
            /* Restore start-of-tick velocity so the analytic solution starts
             * from v_pre (Euler already applied gravity+damping to live). */
            ib -> velocity = world -> tick_v0 [i];
        } rb_integrate_position_exact (ib, step_dt, step_cfg, free_flight);
        /* Exact integration now handles gravity correctly for all drag values.
         * No correction needed. */
        rigidbody_sanitize (&world -> bodies [i]);
    }
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
            boundary_apply_box_cfg (&world -> bodies [i], (vector3) {-250, 0, -250}, (vector3) {250, 500, 250}, step_cfg);
            if (vector3_length_squared (vector3_subtraction (world -> bodies [i].position, a3_pre_boundary_position)) >
                0.000001f) {
                a3_boundary_moved_any = true;
            }
        }
    }
    /* Positional depenetration pass (like legacy path). */
    a3_positional_depenetration_pass_dt (world, world -> pair_buffer, &broadphase_pair_count, a3_boundary_moved_any, dt);
    /* Phase-2: foreign post-step modules (loggers, correctors).
     * Snapshot the table (see pre-step above). */
    {
        const mpe_module_desc_t *mods [16];
        void *states [16];
        int nmods = world -> tick_module_count < 16 ? world -> tick_module_count : 16;
        for (int mi = 0; mi < nmods; mi++) {
            mods [mi] = world -> tick_modules [mi];
            states [mi] = world -> tick_module_state [mi];
        }
        for (int mi = 0; mi < nmods; mi++) {
            if (!mods [mi] || !mods [mi] -> post_step)
                continue;
            void *live = NULL;
            if (!physics_world_module_live_state (world, mods [mi], &live))
                continue;
            mods [mi] -> post_step (world, dt, live);
            (void) states;
        }
    }
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
