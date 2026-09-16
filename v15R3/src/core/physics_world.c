/* MPE_FTC_059C: physics world — full pipeline.
 * Supersedes MPE_FTC_056 (free-body step).
 * Pipeline mirrors the legacy loop in simulation.c, minus:
 *   - sleep staticize hack (sleeping bodies keep real mass here),
 *   - positional depenetration pass,
 *   - joints/constraints (Phase 1).
 * NOTE: the contact warm-start cache is still engine-global. Worlds
 * must seed non-overlapping object_id ranges (see tests/two_world_test.c).
 * A per-world cache is tracked as future work.
 */
#include "physics_world.h"
#include "../physics/collision_mechanics.h"
#include "../physics/broadphase.h"
#include "../physics/constraint.h" /* MPE_FTC_067 */
#include "../physics/islands.h"
#include "../physics/depenetration.h"
#include "../scene/boundary.h"
#include "det_math.h" /* bit-identical damping factors on all IEEE targets */
/* FIX-AUDIT: world spring pass is weakly linked so headless test binaries
 * that omit physics/spring_joint.c (GL dependency) still link; the GUI
 * engine links it and gets real spring forces. Pool comes from the world. */
void apply_spring_forces_world(physics_world *world, rigidbody *bodies, int body_count) __attribute__((weak));
void apply_spring_forces_world_dt(physics_world *world, rigidbody *bodies, int body_count, float dt)
    __attribute__((weak));
#include "../config/mpe_config.h"
#include "../config/mpe_constants.h"
#include <stdlib.h>
#include <math.h>
#include <string.h> /* MPE_FTC_076a */

static physics_world g_physics_world = {.bodies = NULL, .body_count = 0, .body_capacity = 0, .next_object_id = 1};

/* Helper: malloc-or-NULL (leaves member NULL on failure; users degrade
 * gracefully — add fns reject NULL bodies, pairing returns 0, solves skip).
 * Worlds that fail init are safely unusable, never crash-prone. */
void physics_world_init(physics_world *world) {
    if (!world) {
        return;
    }
    det_pin_fp_state();
    memset(world, 0, sizeof(physics_world)); /* MPE_FTC_076a */
    if (!world->bodies) {
        world->bodies = (rigidbody *) malloc((size_t) mpe_max_bodies * sizeof(rigidbody));
        world->body_capacity = mpe_max_bodies;
    }
    if (!world->world_contact_cache) {
        world->world_contact_cache =
            (cached_contact *) malloc((size_t) max_cached_contacts * sizeof(cached_contact)); /* MFS_131A */
    }
    /* Warm-start hash heads: heap, not stack (worlds are often stack-local;
     * an inline 16 KB array risks overflow next to big frames). */
    if (!world->contact_hash_head) {
        world->contact_hash_head = (int32_t *) malloc((size_t) 4096 * sizeof(int32_t));
        if (world->contact_hash_head) {
            for (int i = 0; i < 4096; i++) {
                world->contact_hash_head[i] = -1;
            }
        }
    }
    /* Solver scratch (heap; worlds are frequently stack-local). Sizes are
     * compile-time caps from mpe_constants.h; pages fault on first touch. */
    if (!world->broadphase) {
        world->broadphase = (broadphase_workspace *) calloc(1, sizeof(broadphase_workspace));
        if (world->broadphase) {
            world->broadphase->current_cell_size = 5.0f;
        }
    }
    if (!world->pair_buffer) {
        world->pair_buffer = (broadphase_pair *) malloc((size_t) mpe_max_broadphase_pairs * sizeof(broadphase_pair));
    }
    if (!world->manifolds) {
        world->manifolds = (collision_data *) malloc((size_t) a3_max_manifolds * sizeof(collision_data));
    }
    if (!world->manifold_awake) {
        world->manifold_awake = (unsigned char *) malloc((size_t) a3_max_manifolds * sizeof(unsigned char));
    }
    if (!world->manifold_order) {
        world->manifold_order = (int *) malloc((size_t) a3_max_manifolds * sizeof(int));
    }
    if (!world->manifold_sort_keys) {
        world->manifold_sort_keys = (float *) malloc((size_t) a3_max_manifolds * sizeof(float));
    }
    if (!world->pair_skipped) {
        world->pair_skipped = (unsigned char *) malloc((size_t) mpe_max_broadphase_pairs * sizeof(unsigned char));
    }
    if (!world->island_parent) {
        world->island_parent = (int *) malloc((size_t) mpe_max_bodies * sizeof(int));
    }
    if (!world->island_label) {
        world->island_label = (int *) malloc((size_t) mpe_max_bodies * sizeof(int));
    }
    if (!world->island_awake_flags) {
        world->island_awake_flags = (unsigned char *) malloc((size_t) mpe_max_bodies * sizeof(unsigned char));
    }
    if (!world->ccd_time_remaining) {
        world->ccd_time_remaining = (float *) malloc((size_t) mpe_max_bodies * sizeof(float));
    }
    if (!world->has_contact) {
        world->has_contact = (unsigned char *) malloc((size_t) mpe_max_bodies * sizeof(unsigned char));
    }
    world->body_count = 0;
    if (world->next_object_id == 0) {
        world->next_object_id = 1;
    }
}

static void physics_world_free_ptr(void **slot) {
    if (slot && *slot) {
        free(*slot);
        *slot = NULL;
    }
}

void physics_world_cleanup(physics_world *world) {
    if (!world) {
        return;
    }
    physics_world_free_ptr((void **) &world->bodies);
    physics_world_free_ptr((void **) &world->world_contact_cache);
    physics_world_free_ptr((void **) &world->contact_hash_head);
    broadphase_cleanup(world); /* frees the node pool; struct freed below */
    physics_world_free_ptr((void **) &world->broadphase);
    physics_world_free_ptr((void **) &world->pair_buffer);
    physics_world_free_ptr((void **) &world->manifolds);
    physics_world_free_ptr((void **) &world->manifold_awake);
    physics_world_free_ptr((void **) &world->manifold_order);
    physics_world_free_ptr((void **) &world->manifold_sort_keys);
    physics_world_free_ptr((void **) &world->pair_skipped);
    physics_world_free_ptr((void **) &world->island_parent);
    physics_world_free_ptr((void **) &world->island_label);
    physics_world_free_ptr((void **) &world->island_awake_flags);
    physics_world_free_ptr((void **) &world->ccd_time_remaining);
    physics_world_free_ptr((void **) &world->has_contact);
    world->world_contact_cache_count = 0;
    world->body_count = 0;
    world->body_capacity = 0;
}

int physics_world_add_sphere(physics_world *world, float radius, float mass, vector3 position) {
    if ((!world) || (!world->bodies) || (world->body_count >= world->body_capacity)) {
        return -1;
    }
    rigidbody *rb = &world->bodies[world->body_count];
    rigidbody_initialisation_sphere(rb, radius, mass, position);
    if (world->next_object_id == 0 || world->next_object_id == 0xFFFFFFFFu) {
        world->next_object_id = 1;
    }
    rb->object_id = world->next_object_id++;
    if (world->next_object_id == 0 || world->next_object_id == 0xFFFFFFFFu) {
        world->next_object_id = 1;
    }
    rb->object_generation = 1;
    rigidbody_sanitize(rb);
    return world->body_count++;
}

int physics_world_add_cube(physics_world *world, vector3 position, vector3 half_extensions, float mass) {
    if ((!world) || (!world->bodies) || (world->body_count >= world->body_capacity)) {
        return -1;
    }
    rigidbody *rb = &world->bodies[world->body_count];
    rigidbody_initialisation_cube(rb, position, half_extensions, mass);
    if (world->next_object_id == 0 || world->next_object_id == 0xFFFFFFFFu) {
        world->next_object_id = 1;
    }
    rb->object_id = world->next_object_id++;
    if (world->next_object_id == 0 || world->next_object_id == 0xFFFFFFFFu) {
        world->next_object_id = 1;
    }
    rb->object_generation = 1;
    rigidbody_sanitize(rb);
    return world->body_count++;
}

/* MPE_FTC_091 */
int physics_world_add_cylinder(physics_world *world, float radius, float half_length, float mass,
                             vector3 position) {
    if ((!world) || (!world->bodies) || (world->body_count >= world->body_capacity)) {
        return -1;
    }
    rigidbody *rb = &world->bodies[world->body_count];
    rigidbody_initialisation_cylinder(rb, radius, half_length, mass, position);
    if (world->next_object_id == 0 || world->next_object_id == 0xFFFFFFFFu) {
        world->next_object_id = 1;
    }
    rb->object_id = world->next_object_id++;
    if (world->next_object_id == 0 || world->next_object_id == 0xFFFFFFFFu) {
        world->next_object_id = 1;
    }
    rb->object_generation = 1;
    rigidbody_sanitize(rb);
    return world->body_count++;
}

void physics_world_clear(physics_world *world) {
    if (!world) {
        return;
    }
    world->body_count = 0;
    world->world_contact_cache_count = 0; /* MFS_131A */
    world->manifold_overflow_count = 0;
    world->contact_cache_hits = 0;
    world->contact_cache_misses = 0;
    /* TRUTH: next_object_id monotonic wraps at 4G to 0, colliding with
     * cache sentinel id==0 (no match) + floor 0xFFFFFFFF. Skip 0/0xFFFFFFFF
     * on wrap. clear() does NOT reset IDs (stable across clears would alias
     * old cache entries); instead ensure wrap skips sentinels. */
    if (world->next_object_id == 0 || world->next_object_id == 0xFFFFFFFFu) {
        world->next_object_id = 1;
    }
}

/* One broadphase pair through narrowphase + wake-on-contact + solver
 * prep. Shared by the main pair loop and the sleep-wake revisit pass
 * below (extracted verbatim from the former single loop). */
static void physics_world_process_pair(physics_world *world, int index_a, int index_b, float dt,
                                       int *manifold_count_ptr) {
    if ((index_a < 0) || (index_a >= world->body_count)) {
        return;
    }
    if ((index_b < 0) || (index_b >= world->body_count)) {
        return;
    }
    rigidbody *body_a = &world->bodies[index_a];
    rigidbody *body_b = &world->bodies[index_b];
        collision_data narrowphase_collision = {0};
        bool collided = false;
        if ((body_a->type == object_sphere) && (body_b->type == object_sphere)) {
            collided = collision_dual_sphere(body_a, body_b, &narrowphase_collision);
        } else if ((body_a->type == object_sphere) && (body_b->type == object_cube)) {
            collided = collision_sphere_cube(body_a, body_b, &narrowphase_collision);
        } else if ((body_a->type == object_cube) && (body_b->type == object_sphere)) {
            collided = collision_sphere_cube(body_b, body_a, &narrowphase_collision);
            if (collided) {
                narrowphase_collision.normal_vector = vector3_scaling(narrowphase_collision.normal_vector, -1.0f);
                narrowphase_collision.object_a = body_a;
                narrowphase_collision.object_b = body_b;
            }
        } else if ((body_a->type == object_cube) && (body_b->type == object_cube)) {
            collided = collision_dual_cube(body_a, body_b, &narrowphase_collision);
        } else if ((body_a->type == object_cylinder) && (body_b->type == object_sphere)) { /* MFS_173C_REPAIRED */
            collided = collision_cylinder_sphere(body_a, body_b, &narrowphase_collision);
        } else if ((body_a->type == object_sphere) && (body_b->type == object_cylinder)) {
            collided = collision_cylinder_sphere(body_b, body_a, &narrowphase_collision);
            if (collided) {
                narrowphase_collision.normal_vector = vector3_scaling(narrowphase_collision.normal_vector, -1.0f);
                narrowphase_collision.object_a = body_a;
                narrowphase_collision.object_b = body_b;
            }
        } else if ((body_a->type == object_cylinder) && (body_b->type == object_cube)) {
            collided = collision_cylinder_cube(body_a, body_b, &narrowphase_collision);
        } else if ((body_a->type == object_cube) && (body_b->type == object_cylinder)) {
            collided = collision_cylinder_cube(body_b, body_a, &narrowphase_collision);
            if (collided) {
                narrowphase_collision.normal_vector = vector3_scaling(narrowphase_collision.normal_vector, -1.0f);
                narrowphase_collision.object_a = body_a;
                narrowphase_collision.object_b = body_b;
            }
        } else if ((body_a->type == object_cylinder) && (body_b->type == object_cylinder)) {
            collided = collision_cylinder_cylinder(body_a, body_b, &narrowphase_collision);
        }         if (collided) {
            if ((*manifold_count_ptr) >= a3_max_manifolds) {
                world->manifold_overflow_count++;
                return;
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
            bool a_was_sleeping = body_a->is_sleeping;
            bool b_was_sleeping = body_b->is_sleeping;

            if ((a_was_sleeping) && (b_was_sleeping)) {
                /* Both sleeping: nothing to wake (depth backstop below still
                 * applies via the revisit/split/depen paths). */
            } else {
                bool new_edge = !contact_cache_has_pair(world, body_a->object_id, body_b->object_id);
                if ((a_was_sleeping) && (!body_b->static_state) && new_edge) {
                    rigidbody_wake(body_a);
                }
                if ((b_was_sleeping) && (!body_a->static_state) && new_edge) {
                    rigidbody_wake(body_b);
                }

                float wake_lin_sq = g_cfg.sleep.wake_linear_thresh_sq;
                float wake_ang_sq = g_cfg.sleep.wake_angular_thresh_sq;
                bool a_fast = (!a_was_sleeping) &&
                              ((vector3_length_squared(body_a->velocity) > wake_lin_sq) ||
                               (vector3_length_squared(body_a->angular_velocity) > wake_ang_sq));
                bool b_fast = (!b_was_sleeping) &&
                              ((vector3_length_squared(body_b->velocity) > wake_lin_sq) ||
                               (vector3_length_squared(body_b->angular_velocity) > wake_ang_sq));
                if ((a_was_sleeping) && (!body_b->static_state) && b_fast) {
                    rigidbody_wake(body_a);
                }
                if ((b_was_sleeping) && (!body_a->static_state) && a_fast) {
                    rigidbody_wake(body_b);
                }

                /* Wake on significant overlap growth, even against static
                 * geometry: a sleeper ground into a static surface by a new
                 * persistent force (e.g. a spawned overlap, a tilted stack
                 * settling) must rejoin the solve. Resting contact within
                 * slop never reaches this bar. */
                {
                    float deepest = 0.0f;
                    for (int wi = 0; wi < narrowphase_collision.contact_count; wi++) {
                        if (narrowphase_collision.contacts[wi].penetration > deepest) {
                            deepest = narrowphase_collision.contacts[wi].penetration;
                        }
                    }
                    if (deepest > g_cfg.depenetration.wake_depth_thresh) {
                        if (a_was_sleeping) {
                            rigidbody_wake(body_a);
                        }
                        if (b_was_sleeping) {
                            rigidbody_wake(body_b);
                        }
                    }
                }

                collision_prepare_solver(world, &narrowphase_collision, &world->manifolds[(*manifold_count_ptr)], dt);
                (*manifold_count_ptr)++;
                /* TRUTH P0-1: contact flag for gravity-exactness gating. */
                if (world->has_contact) {
                    if ((index_a >= 0) && (index_a < world->body_count)) {
                        world->has_contact[index_a] = 1;
                    }
                    if ((index_b >= 0) && (index_b < world->body_count)) {
                        world->has_contact[index_b] = 1;
                    }
                }
            }
        }
}

void physics_world_step(physics_world *world, float dt) {
    if ((!world) || (!world->bodies) || (!(dt > 0.0f)) || (!isfinite(dt)) || (world->body_count <= 0)) {
        return;
    }
    if (dt > 0.1f) {
        dt = 0.1f;
    }

    for (int i = 0; i < world->body_count; i++) {
        rigidbody_sanitize(&world->bodies[i]);
    }
    /* TRUTH: snapshot config once per tick. Menu/terminal mutating g_cfg
     * mid-tick (between substeps) would otherwise change behavior halfway
     * through the frame. Hot path uses these locals, never g_cfg directly. */
    const float step_gravity = g_cfg.world.gravity;
    const float step_drag = g_cfg.world.drag;
    const float step_ang_scale = g_cfg.world.angular_damping_scale;
    const int step_iterations = g_cfg.timestep.solver_iterations;
    /* TRUTH P0-1: reset contact flags (set below on every manifold). */
    if (world->has_contact) {
        for (int i = 0; i < world->body_count; i++) {
            world->has_contact[i] = 0;
        }
    }

    /* CCD: clamp fast bodies to their time-of-impact pose before pairing,
     * so discrete narrowphase cannot tunnel past thin geometry.
     * TRUTH P0-3: remainder (dt-toi) recorded per body; post-solve
     * integration advances only the remainder (see below). */
    if (world->ccd_time_remaining) {
        collision_ccd_sweep_clamp_full(world->bodies, world->body_count, dt, world->ccd_time_remaining);
    } else {
        collision_ccd_sweep_clamp(world->bodies, world->body_count, dt);
    }

    int pair_count = 0;
    if (world->body_count >= 2) {
        pair_count =
            broadphase_generate_pairing(world, world->pair_buffer, mpe_max_broadphase_pairs, dt);
    }
    int broadphase_pair_count = pair_count; /* saved for depenetration pass */

    /* Low-memory contract: degraded (pairless) tick instead of a crash when
     * scratch failed to allocate. TRUTH: sort_keys missing left order_out
     * uninitialized -> OOB solve. Check it too. */
    if ((!world->pair_buffer) || (!world->manifolds) || (!world->manifold_awake) || (!world->manifold_order) ||
        (!world->manifold_sort_keys) || (!world->pair_skipped) || (!world->broadphase)) {
        return;
    }
    int manifold_count = 0;
    for (int q = 0; q < pair_count; q++) {
        world->pair_skipped[q] = 0;
    }
    contact_cache_stats_reset(world);
    for (int p = 0; p < pair_count; p++) {
        int index_a = world->pair_buffer[p].object_index_a;
        int index_b = world->pair_buffer[p].object_index_b;
        if ((index_a < 0) || (index_a >= world->body_count) || (index_b < 0) ||
            (index_b >= world->body_count)) {
            continue;
        }
        rigidbody *body_a = &world->bodies[index_a];
        rigidbody *body_b = &world->bodies[index_b];
        if ((body_a->is_sleeping) && (body_b->is_sleeping)) {
            world->pair_skipped[p] = 1;
            continue;
        }
        physics_world_process_pair(world, index_a, index_b, dt, &manifold_count);
    }
    /* Sleep-wake revisit fixpoint: pairs skipped above as both-asleep get
     * re-processed if either endpoint woke during the main loop (a woken
     * body with no manifold free-falls the whole tick otherwise). Each
     * pass only touches newly-active skipped pairs; cascades settle pass
     * by pass. Deterministic (pair order, wake-driven, bounded). */
    for (int revisit_pass = 0; revisit_pass < 16; revisit_pass++) {
        bool revisit_woke = false;
        for (int p = 0; p < pair_count; p++) {
            if (!world->pair_skipped[p]) {
                continue;
            }
            int index_a = world->pair_buffer[p].object_index_a;
            int index_b = world->pair_buffer[p].object_index_b;
            if ((index_a < 0) || (index_a >= world->body_count) || (index_b < 0) ||
                (index_b >= world->body_count)) {
                world->pair_skipped[p] = 0;
                continue;
            }
            rigidbody *body_a = &world->bodies[index_a];
            rigidbody *body_b = &world->bodies[index_b];
            if ((body_a->is_sleeping) && (body_b->is_sleeping)) {
                continue;
            }
            bool slept_a = body_a->is_sleeping;
            bool slept_b = body_b->is_sleeping;
            physics_world_process_pair(world, index_a, index_b, dt, &manifold_count);
            world->pair_skipped[p] = 0;
            if ((slept_a && !body_a->is_sleeping) || (slept_b && !body_b->is_sleeping)) {
                revisit_woke = true;
            }
        }
        if (!revisit_woke) {
            break;
        }
    }

    for (int i = 0; i < world->body_count; i++) {
        rigidbody *rb = &world->bodies[i];
        if (rb->static_state) {
            continue;
        }
        /* TRUTH: sleeping floor contact must still build a manifold (with
         * eff_inv=0 it solves as no-op) so has_contact is set and deep
         * sink wakes. Old skip left shallow sleepers intersecting floor
         * forever with has_contact=0 (false free-flight). */
        bool was_sleeping = rb->is_sleeping;
        collision_data floor_collision = {0};
        if (collision_static_plane_body(rb, 0.0f, &floor_collision)) {
            if (manifold_count >= a3_max_manifolds) {
                world->manifold_overflow_count++;
                continue;
            }
            float deepest = 0.0f;
            for (int fi = 0; fi < floor_collision.contact_count; fi++) {
                if (floor_collision.contacts[fi].penetration > deepest) {
                    deepest = floor_collision.contacts[fi].penetration;
                }
            }
            if (was_sleeping && deepest > g_cfg.depenetration.wake_depth_thresh) {
                rigidbody_wake(rb);
            }
            collision_prepare_solver(world, &floor_collision, &world->manifolds[manifold_count], dt);
            manifold_count++;
            if (world->has_contact) {
                world->has_contact[i] = 1;
            }
        }
    }

    /* Solver islands: union pairs + joints, then flag each manifold.
     * Fully-sleeping islands skip the iteration/relaxation loops below
     * (exact no-ops: zero velocity, zeroed inverses). */
    islands_build(world, world->pair_buffer, pair_count);
    for (int m = 0; m < manifold_count; m++) {
        rigidbody *ma = world->manifolds[m].object_a;
        rigidbody *mb = world->manifolds[m].object_b;
        world->manifold_awake[m] =
            (unsigned char) (islands_body_awake(world, ma) || islands_body_awake(world, mb));
    }
    collision_manifold_solve_order(world, world->manifolds, manifold_count, world->manifold_order);

    vector3 gravity = {0.0f, step_gravity, 0.0f};
    for (int i = 0; i < world->body_count; i++) {
        rigidbody *rb = &world->bodies[i];
        if ((rb->static_state) || (rb->is_sleeping) || (rb->kinematic)) {
            continue;
        }
        rb_apply_forces_perfect(rb, vector3_scaling(gravity, rb->mass));
    }

    /* Deterministic retention factors (never libm pow: see det_math.h). */
    float linear_damping = (float) det_pow_retention((double) step_drag, (double) dt);
    /* FIX-AUDIT: angular scale was hardcoded 0.97 (damped even at drag=1).
     * Now a real config param. */
    float angular_damping =
        (float) det_pow_retention((double) (step_drag * step_ang_scale), (double) dt);
    /* FIX-AUDIT: springs were never applied on the encapsulated path.
     * Apply world-aware spring forces before integration (weak-linked). */
    if (apply_spring_forces_world_dt) {
        apply_spring_forces_world_dt(world, world->bodies, world->body_count, dt);
    } else if (apply_spring_forces_world) {
        apply_spring_forces_world(world, world->bodies, world->body_count);
    }
    constraint_apply_motors(world, dt); /* MPE_FTC_067 */
    for (int i = 0; i < world->body_count; i++) {
        rb_integrate_velocity(&world->bodies[i], dt, linear_damping, angular_damping);
    }

    /* Sleeping bodies keep real mass/inertia (no staticize mutation).
     * The solver treats them as infinite mass via rigidbody_effective_*
     * helpers, so observable state is never corrupted mid-tick and the
     * path is thread-safe. Zero stale velocities/accumulators only. */
    for (int sleep_index = 0; sleep_index < world->body_count; sleep_index++) {
        rigidbody *sleep_body = &world->bodies[sleep_index];
        if ((sleep_body->is_sleeping) && (!sleep_body->static_state)) {
            sleep_body->velocity = vector3_zero();
            sleep_body->angular_velocity = vector3_zero();
            sleep_body->force_accumulator = vector3_zero();
            sleep_body->torque_accumulator = vector3_zero();
        }
    }

    /* TRUTH P0-2: Poisson gate must see post-force-integration approach
     * speed (prepare ran pre-gravity, stale by g*dt). Refresh vn from
     * current velocities before any iteration touches accumulators. */
    collision_refresh_impact_velocities(world->manifolds, manifold_count);

    int solver_iterations = step_iterations;
    if (solver_iterations < 1) {
        solver_iterations = 1;
    }
    if (solver_iterations > 128) {
        solver_iterations = 128;
    }
    /* TRUTH: joint angle/slide integration once per tick (not per iteration). */
    constraint_pre_step_all(world, dt);
    for (int iter = 0; iter < solver_iterations; iter++) {
        for (int o = 0; o < manifold_count; o++) {
            int m = world->manifold_order[o];
            if (!world->manifold_awake[m]) {
                continue;
            }
            /* Local convergence: a manifold's contacts share bodies
             * (strong rotational coupling), so one visit rarely settles
             * them — the residue pollutes the next level up the stack.
             * A second immediate visit converges the local distribution
             * before propagating, buying back the margin deep stacks
             * need at low global iteration counts. Deterministic. */
            collision_resolve_iterative(&world->manifolds[m], dt, false, iter);
            collision_resolve_iterative(&world->manifolds[m], dt, false, iter + 1);
        }
        /* MFS_SOLVER_FIX: solve joints inside the iteration loop so friction
         * impulses properly transfer through revolute constraints to the chassis */
        constraint_solve_all(world, dt);
    }
    /* Positional axis-drift correction: exactly once per tick, never in the
     * loop above (see revolute_joint.c). */
    constraint_correct_axis_drift_all(world, dt);
    /* Poisson restitution: one e-over-compression payment per fresh impact,
     * then short relaxation so friction sees post-bounce velocities. */
    /* TRUTH: joints must see post-bounce velocities too. Poisson without a
     * joint relaxation leaves hinges/welds broken for a tick. */
    collision_apply_poisson_restitution(world->manifolds, manifold_count);
    constraint_solve_all(world, dt);
    for (int relax_iter = 0; relax_iter < 2; relax_iter++) {
        for (int o = 0; o < manifold_count; o++) {
            int m = world->manifold_order[o];
            if (!world->manifold_awake[m]) {
                continue;
            }
            collision_resolve_iterative(&world->manifolds[m], dt, true, relax_iter);
        }
    }
    /* Split impulse: positional depenetration with zero velocity change.
     * The velocity solve above is compression-only, so contact impulses
     * (and the Coulomb clamp) stay honest. */
    collision_apply_split_impulse(world->manifolds, manifold_count, dt);
    /* TRUTH: split can wake sleepers (deep overlap) that were skipped as
     * both-asleep with no manifold. They would integrate with has_contact=0
     * (false free-flight). Re-process newly-awake skipped pairs now so they
     * get a manifold + has_contact before integration. */
    for (int p = 0; p < pair_count; p++) {
        if (!world->pair_skipped[p]) {
            continue;
        }
        int ia = world->pair_buffer[p].object_index_a;
        int ib = world->pair_buffer[p].object_index_b;
        if (ia < 0 || ia >= world->body_count || ib < 0 || ib >= world->body_count) {
            world->pair_skipped[p] = 0;
            continue;
        }
        if (world->bodies[ia].is_sleeping && world->bodies[ib].is_sleeping) {
            continue;
        }
        physics_world_process_pair(world, ia, ib, dt, &manifold_count);
        world->pair_skipped[p] = 0;
    }
    /* Rolling resistance once per tick (uses solved normal impulses). */
    collision_apply_rolling_resistance(world->manifolds, manifold_count, dt);

    /* Sleeping bodies already hold real mass (no staticize was applied),
     * so no restore is needed. Keep velocities pinned at zero. */
    for (int sleep_restore_index = 0; sleep_restore_index < world->body_count; sleep_restore_index++) {
        rigidbody *sleep_restore_body = &world->bodies[sleep_restore_index];
        if ((sleep_restore_body->is_sleeping) && (!sleep_restore_body->static_state)) {
            sleep_restore_body->velocity = vector3_zero();
            sleep_restore_body->angular_velocity = vector3_zero();
        }
    }

    contact_cache_save(world, world->manifolds, manifold_count);

    /* TRUTH P0-1+P0-3: integrate CCD remainder; gravity-exactness for
     * contact-free bodies ONLY (x += v*rem - 1/2*g*rem^2 is the exact
     * constant-force flow, hence exact parabolas; constrained bodies stay
     * symplectic Euler since contact impulses cancel gravity post-solve). */
    float grav_half = -0.5f * step_gravity;
    for (int i = 0; i < world->body_count; i++) {
        float step_dt = dt;
        if ((world->ccd_time_remaining) && (world->ccd_time_remaining[i] < step_dt) &&
            (world->ccd_time_remaining[i] > 0.0f)) {
            step_dt = world->ccd_time_remaining[i];
        }
        rb_integrate_position(&world->bodies[i], step_dt);
        rigidbody *ib = &world->bodies[i];
        bool free_flight = (world->has_contact) ? (world->has_contact[i] == 0) : true;
        if (free_flight && (!ib->static_state) && (!ib->is_sleeping) && (!ib->kinematic) && (step_dt > 0.0f)) {
            ib->position.y += grav_half * step_dt * step_dt;
        }
        rigidbody_sanitize(&world->bodies[i]);
    }

    /* World-edge safety net, same as the legacy path: perfectly plastic,
     * fires only past emergency slop (solver owns all normal contact). */
    bool a3_boundary_moved_any = false;
    for (int i = 0; i < world->body_count; i++) {
        vector3 a3_pre_boundary_position = world->bodies[i].position;
        boundary_apply_box(&world->bodies[i], (vector3){-250, 0, -250}, (vector3){250, 500, 250});
        if (vector3_length_squared(vector3_subtraction(world->bodies[i].position, a3_pre_boundary_position)) > 0.000001f) {
            a3_boundary_moved_any = true;
        }
    }

    /* Positional depenetration pass (like legacy path). */
    a3_positional_depenetration_pass_dt(world, world->pair_buffer, &broadphase_pair_count, a3_boundary_moved_any,
                                        dt);
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
}

physics_world *physics_world_get_primary(void) {
    return &g_physics_world;
}

/* R3-07: Containment walls.
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
int physics_world_add_boundary_walls(physics_world *world,
                                     float half_width,
                                     float half_depth,
                                     float wall_height,
                                     float wall_thickness)
{
    if (!world) {
        return -1;
    }
    if ((half_width <= 0.0f) || (half_depth <= 0.0f) ||
        (wall_height <= 0.0f) || (wall_thickness <= 0.0f)) {
        return -1;
    }

    float hy = wall_height * 0.5f;
    float ht = wall_thickness * 0.5f;

    /* North wall: +Z side */
    int north = physics_world_add_cube(world,
        (vector3){0.0f, hy, half_depth + ht},
        (vector3){half_width + wall_thickness, hy, ht},
        0.0f);

    /* South wall: -Z side */
    int south = physics_world_add_cube(world,
        (vector3){0.0f, hy, -(half_depth + ht)},
        (vector3){half_width + wall_thickness, hy, ht},
        0.0f);

    /* East wall: +X side */
    int east = physics_world_add_cube(world,
        (vector3){half_width + ht, hy, 0.0f},
        (vector3){ht, hy, half_depth + wall_thickness},
        0.0f);

    /* West wall: -X side */
    int west = physics_world_add_cube(world,
        (vector3){-(half_width + ht), hy, 0.0f},
        (vector3){ht, hy, half_depth + wall_thickness},
        0.0f);

    if ((north < 0) || (south < 0) || (east < 0) || (west < 0)) {
        return -1;
    }

    return 0;
}
