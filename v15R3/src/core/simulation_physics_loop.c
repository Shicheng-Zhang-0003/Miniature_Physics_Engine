/* MFS_INCREMENT_SPLIT_2: Fixed-timestep physics loop.
* Extracted from physics_step_increment in simulation.c.
* Owns: the accumulator, broadphase, narrowphase, solver, integration,
*        sleep staticize/restore, boundary, depenetration.
* LEGACY GUI PATH: operates on global obj_per_scene / object_count so the
* GTK render/editor loop keeps working. Canonical stepping is
* physics_world_step() in core/physics_world.c (used by all headless
* tests). Do not add new simulation state here — add it to physics_world.
*/
#include "../mpe_engine.h"
#include "../physics/depenetration.h"
#include "../physics/constraint.h"
#include "../physics/islands.h"
#include "../core/det_math.h"
#include <math.h>

static broadphase_pair persistent_collision_pairs[mpe_max_broadphase_pairs];
/* Per-substep island wake flags, one per manifold (see islands.h). */
static unsigned char legacy_manifold_awake[a3_max_manifolds];
/* Support-first sweep order (see collision_manifold_solve_order). */
static int legacy_manifold_order[a3_max_manifolds];

/* One broadphase pair through narrowphase + wake-on-contact + solver
 * prep (legacy GUI path). Shared by the main pair loop and the sleep-wake
 * revisit pass below; see physics_world.c for the race it closes. */
static void simulation_process_pair(int object_index_a, int object_index_b, float dt,
                                    collision_data *manifolds, int max_manifolds, int *manifold_count_ptr) {
    rigidbody *body_a = &obj_per_scene[object_index_a];
    rigidbody *body_b = &obj_per_scene[object_index_b];
            collision_data narrowphase_collision = {0};
            bool collided = false;
            if (body_a->type == object_sphere && body_b->type == object_sphere)
                collided = collision_dual_sphere(body_a, body_b, &narrowphase_collision);
            else if (body_a->type == object_sphere && body_b->type == object_cube)
                collided = collision_sphere_cube(body_a, body_b, &narrowphase_collision);
            else if (body_a->type == object_cube && body_b->type == object_sphere) {
                collided = collision_sphere_cube(body_b, body_a, &narrowphase_collision);
                narrowphase_collision.normal_vector = vector3_scaling(narrowphase_collision.normal_vector, -1.0f);
                narrowphase_collision.object_a = body_a;
                narrowphase_collision.object_b = body_b;
            } else if (body_a->type == object_cube && body_b->type == object_cube)
                collided = collision_dual_cube(body_a, body_b, &narrowphase_collision);
            /* MFS_173D: Cylinder collision dispatch */
            else if (body_a->type == object_cylinder && body_b->type == object_sphere)
                collided = collision_cylinder_sphere(body_a, body_b, &narrowphase_collision);
            else if (body_a->type == object_sphere && body_b->type == object_cylinder) {
                collided = collision_cylinder_sphere(body_b, body_a, &narrowphase_collision);
                narrowphase_collision.normal_vector = vector3_scaling(narrowphase_collision.normal_vector, -1.0f);
                narrowphase_collision.object_a = body_a;
                narrowphase_collision.object_b = body_b;
            } else if (body_a->type == object_cylinder && body_b->type == object_cube)
                collided = collision_cylinder_cube(body_a, body_b, &narrowphase_collision);
            else if (body_a->type == object_cube && body_b->type == object_cylinder) {
                collided = collision_cylinder_cube(body_b, body_a, &narrowphase_collision);
                narrowphase_collision.normal_vector = vector3_scaling(narrowphase_collision.normal_vector, -1.0f);
                narrowphase_collision.object_a = body_a;
                narrowphase_collision.object_b = body_b;
            } else if (body_a->type == object_cylinder && body_b->type == object_cylinder)
                collided = collision_cylinder_cylinder(body_a, body_b, &narrowphase_collision);
            if (collided) {
                if ((*manifold_count_ptr) < max_manifolds) {
                    bool a3_a_was_sleeping = body_a->is_sleeping;
                    bool a3_b_was_sleeping = body_b->is_sleeping;
                    if (a3_a_was_sleeping && a3_b_was_sleeping) { return; }
                    float a3_wake_linear_threshold_sq = g_cfg.sleep.wake_linear_thresh_sq;
                    float a3_wake_angular_threshold_sq = g_cfg.sleep.wake_angular_thresh_sq;
                    bool a3_a_is_active = (!a3_a_was_sleeping) &&
                        ((vector3_length_squared(body_a->velocity) > a3_wake_linear_threshold_sq) ||
                         (vector3_length_squared(body_a->angular_velocity) > a3_wake_angular_threshold_sq));
                    bool a3_b_is_active = (!a3_b_was_sleeping) &&
                        ((vector3_length_squared(body_b->velocity) > a3_wake_linear_threshold_sq) ||
                         (vector3_length_squared(body_b->angular_velocity) > a3_wake_angular_threshold_sq));
                    if (a3_a_was_sleeping && (!body_b->static_state) && a3_b_is_active) {
                        rigidbody_wake(body_a);
                    }
                    if (a3_b_was_sleeping && (!body_a->static_state) && a3_a_is_active) {
                        rigidbody_wake(body_b);
                    }
                    /* Wake on significant overlap growth, even against static
                     * geometry (see physics_world.c). */
                    {
                        float deepest = 0.0f;
                        for (int wi = 0; wi < narrowphase_collision.contact_count; wi++) {
                            if (narrowphase_collision.contacts[wi].penetration > deepest) {
                                deepest = narrowphase_collision.contacts[wi].penetration;
                            }
                        }
                        if (deepest > g_cfg.depenetration.wake_depth_thresh) {
                            if (a3_a_was_sleeping) {
                                rigidbody_wake(body_a);
                            }
                            if (a3_b_was_sleeping) {
                                rigidbody_wake(body_b);
                            }
                        }
                    }
                    collision_prepare_solver(NULL, &narrowphase_collision, &manifolds[(*manifold_count_ptr)],
                                             dt);
                    (*manifold_count_ptr)++;
                } else {
                    debug_last_manifold_overflow_count++;
                }
            }
}

void simulation_physics_tick(float frame_delta_time) {
    /* FIX-AUDIT: static float accumulator bled across scene loads and lost
     * precision on long runs. Double precision here. */
    static double physics_time_accumulator = 0.0;
    const float fixed_physics_dt = 1.0f / 60.0f;
    /* FIX-AUDIT: max_substeps was hardcoded 5, ignoring
     * g_cfg.timestep.max_substeps (1..20). */
    int max_substeps_per_frame = g_cfg.timestep.max_substeps;
    if (max_substeps_per_frame < 1) {
        max_substeps_per_frame = 1;
    }
    if (max_substeps_per_frame > 20) {
        max_substeps_per_frame = 20;
    }
    if (frame_delta_time < 0.0f) {
        frame_delta_time = 0.0f;
    }
    physics_time_accumulator += (double) frame_delta_time;
    if (physics_time_accumulator > (double) fixed_physics_dt * (double) max_substeps_per_frame) {
        /* Anti-spiral: excess wall-clock time is dropped (sim lags wall). */
        physics_time_accumulator = (double) fixed_physics_dt * (double) max_substeps_per_frame;
    }
    float linear_damping_factor =
        (float) det_pow_retention((double) g_cfg.world.drag, (double) fixed_physics_dt);
    float angular_damping_factor = (float) det_pow_retention(
        (double) (g_cfg.world.drag * g_cfg.world.angular_damping_scale), (double) fixed_physics_dt);
    debug_last_manifold_overflow_count = 0;
    while (physics_time_accumulator >= fixed_physics_dt) {
        /* Sanitize all bodies */
        for (int sanitize_index = 0; sanitize_index < object_count; sanitize_index++) {
            rigidbody_sanitize(&obj_per_scene[sanitize_index]);
        }
        /* CCD: clamp fast bodies to TOI pose (see physics_world.c). */
        collision_ccd_sweep_clamp(obj_per_scene, object_count, fixed_physics_dt);
        /* Broadphase */
        int detected_collision_count = 0;
        detected_collision_count = broadphase_generate_pairing(
            obj_per_scene, object_count, persistent_collision_pairs, mpe_max_broadphase_pairs, fixed_physics_dt);
        debug_last_broadphase_pair_count = detected_collision_count;
        /* Narrowphase + manifold build */
        static collision_data active_manifold[a3_max_manifolds];
        int manifold_count = 0;
        contact_cache_stats_reset();
        apply_force_all_joints();
        /* Gravity */
        for (int object_iterator_index = 0; object_iterator_index < object_count; object_iterator_index++) {
            vector3 constant_gravity_acceleration = {0, g_cfg.world.gravity, 0};
            rigidbody *rigid_body = &obj_per_scene[object_iterator_index];
            if ((rigid_body->is_sleeping) || (rigid_body->kinematic)) { continue; }
            rb_apply_forces_perfect(rigid_body, vector3_scaling(constant_gravity_acceleration, rigid_body->mass));
        }
        /* FIX-AUDIT: revolute motors were never applied on the legacy path.
         * Apply before velocity integration (mirrors world path). */
        constraint_apply_motors(obj_per_scene, object_count, fixed_physics_dt);
        /* Integrate velocity */
        for (int velocity_integration_index = 0; velocity_integration_index < object_count;
             velocity_integration_index++) {
            rb_integrate_velocity(&obj_per_scene[velocity_integration_index], fixed_physics_dt,
                                  linear_damping_factor, angular_damping_factor);
        }
        /* Narrowphase dispatch */
        static unsigned char legacy_pair_skipped[mpe_max_broadphase_pairs];
        for (int q = 0; q < detected_collision_count; q++) {
            legacy_pair_skipped[q] = 0;
        }
        for (int collision_index = 0; collision_index < detected_collision_count; collision_index++) {
            int index_a = persistent_collision_pairs[collision_index].object_index_a;
            int index_b = persistent_collision_pairs[collision_index].object_index_b;
            if ((index_a < 0) || (index_a >= object_count) || (index_b < 0) || (index_b >= object_count)) {
                continue;
            }
            rigidbody *body_a = &obj_per_scene[index_a];
            rigidbody *body_b = &obj_per_scene[index_b];
            if ((body_a->is_sleeping) && (body_b->is_sleeping)) {
                legacy_pair_skipped[collision_index] = 1;
                continue;
            }
            simulation_process_pair(index_a, index_b, fixed_physics_dt, active_manifold, a3_max_manifolds,
                                    &manifold_count);
        }
        /* Sleep-wake revisit fixpoint (see physics_world.c). */
        for (int revisit_pass = 0; revisit_pass < 16; revisit_pass++) {
            bool revisit_woke = false;
            for (int collision_index = 0; collision_index < detected_collision_count; collision_index++) {
                if (!legacy_pair_skipped[collision_index]) {
                    continue;
                }
                int index_a = persistent_collision_pairs[collision_index].object_index_a;
                int index_b = persistent_collision_pairs[collision_index].object_index_b;
                if ((index_a < 0) || (index_a >= object_count) || (index_b < 0) || (index_b >= object_count)) {
                    legacy_pair_skipped[collision_index] = 0;
                    continue;
                }
                rigidbody *body_a = &obj_per_scene[index_a];
                rigidbody *body_b = &obj_per_scene[index_b];
                if ((body_a->is_sleeping) && (body_b->is_sleeping)) {
                    continue;
                }
                bool slept_a = body_a->is_sleeping;
                bool slept_b = body_b->is_sleeping;
                simulation_process_pair(index_a, index_b, fixed_physics_dt, active_manifold, a3_max_manifolds,
                                        &manifold_count);
                legacy_pair_skipped[collision_index] = 0;
                if ((slept_a && !body_a->is_sleeping) || (slept_b && !body_b->is_sleeping)) {
                    revisit_woke = true;
                }
            }
            if (!revisit_woke) {
                break;
            }
        }
        /* Floor collision */
        for (int floor_object_index = 0; floor_object_index < object_count; floor_object_index++) {
            rigidbody *floor_rigid_body = &obj_per_scene[floor_object_index];
            if ((floor_rigid_body->static_state) || (floor_rigid_body->is_sleeping)) { continue; }
            collision_data floor_collision = {0};
            if (collision_static_plane_body(floor_rigid_body, 0.0f, &floor_collision)) {
                if (manifold_count < a3_max_manifolds) {
                    collision_prepare_solver(NULL, &floor_collision, &active_manifold[manifold_count],
                                             fixed_physics_dt);
                    manifold_count++;
                } else {
                    debug_last_manifold_overflow_count++;
                }
            }
        }
        debug_last_manifold_count = manifold_count;
        /* Solver islands (see physics_world.c): skip fully-sleeping islands
         * in the iteration/relaxation loops below. */
        islands_build(obj_per_scene, object_count, persistent_collision_pairs, detected_collision_count);
        for (int island_mark = 0; island_mark < manifold_count; island_mark++) {
            rigidbody *ma = active_manifold[island_mark].object_a;
            rigidbody *mb = active_manifold[island_mark].object_b;
            legacy_manifold_awake[island_mark] =
                (unsigned char) (islands_body_awake(obj_per_scene, ma) || islands_body_awake(obj_per_scene, mb));
        }
        collision_manifold_solve_order(active_manifold, manifold_count, legacy_manifold_order);
        /* Sleep staticize */
        math3 a3_sleep_zero_matrix = {{{0.0f}}};
        for (int sleep_staticize_index = 0; sleep_staticize_index < object_count; sleep_staticize_index++) {
            rigidbody *sleep_staticize_body = &obj_per_scene[sleep_staticize_index];
            if ((sleep_staticize_body->is_sleeping) && (!sleep_staticize_body->static_state)) {
                sleep_staticize_body->velocity = vector3_zero();
                sleep_staticize_body->angular_velocity = vector3_zero();
                sleep_staticize_body->force_accumulator = vector3_zero();
                sleep_staticize_body->torque_accumulator = vector3_zero();
                sleep_staticize_body->inverse_mass = 0.0f;
                sleep_staticize_body->inverse_inertia_system = a3_sleep_zero_matrix;
            }
        }
        /* Solver iterations */
        int solver_iterations = g_cfg.timestep.solver_iterations;
        for (int iter = 0; iter < solver_iterations; iter++) {
            for (int o = 0; o < manifold_count; o++) {
                int m = legacy_manifold_order[o];
                if (!legacy_manifold_awake[m]) {
                    continue;
                }
                /* Local convergence (see physics_world.c): settle the
                 * manifold's coupled contacts before propagating upward. */
                collision_resolve_iterative(&active_manifold[m], fixed_physics_dt, false, iter);
                collision_resolve_iterative(&active_manifold[m], fixed_physics_dt, false, iter + 1);
            }
            /* FIX-AUDIT: revolute constraints were never solved on the
             * legacy path (only in physics_world). Solve inside the
             * iteration loop so contact/joint impulses couple — mirroring
             * physics_world_step(). */
            constraint_solve_all(obj_per_scene, object_count, fixed_physics_dt);
        }
        /* Positional axis-drift correction: exactly once per substep, never
         * in the loop above (see revolute_joint.c). */
        constraint_correct_axis_drift_all(obj_per_scene, object_count, fixed_physics_dt);
        /* Poisson restitution + relaxation (see physics_world.c). */
        collision_apply_poisson_restitution(active_manifold, manifold_count);
        for (int relax_iter = 0; relax_iter < 2; relax_iter++) {
            for (int o = 0; o < manifold_count; o++) {
                int m = legacy_manifold_order[o];
                if (!legacy_manifold_awake[m]) {
                    continue;
                }
                collision_resolve_iterative(&active_manifold[m], fixed_physics_dt, true, relax_iter);
            }
        }
        /* Rolling resistance once per tick (uses solved normal impulses). */
        collision_apply_rolling_resistance(active_manifold, manifold_count, fixed_physics_dt);
        contact_cache_save(NULL, active_manifold, manifold_count); /* MFS_131A: legacy global fallback cache */
        /* Sleep restore */
        for (int sleep_restore_index = 0; sleep_restore_index < object_count; sleep_restore_index++) {
            rigidbody *sleep_restore_body = &obj_per_scene[sleep_restore_index];
            if ((sleep_restore_body->is_sleeping) && (!sleep_restore_body->static_state)) {
                if ((sleep_restore_body->mass > 0.0f) && (isfinite(sleep_restore_body->mass))) {
                    sleep_restore_body->inverse_mass = 1.0f / sleep_restore_body->mass;
                } else {
                    sleep_restore_body->inverse_mass = 0.0f;
                }
                math3 sleep_rotation_matrix = vector4_to_math3(sleep_restore_body->orientation);
                math3 sleep_rotation_transpose = math3_transposition(sleep_rotation_matrix);
                sleep_restore_body->inverse_inertia_system = math3_multiplication(
                    sleep_rotation_matrix,
                    math3_multiplication(sleep_restore_body->inverse_inertia_tensor_local, sleep_rotation_transpose));
                sleep_restore_body->velocity = vector3_zero();
                sleep_restore_body->angular_velocity = vector3_zero();
            }
        }
        /* Integrate position + boundary + depenetration */
        bool a3_boundary_moved_any = false;
        for (int object_iterator_index = 0; object_iterator_index < object_count; object_iterator_index++) {
            rigidbody *rigid_body = &obj_per_scene[object_iterator_index];
            rb_integrate_position(rigid_body, fixed_physics_dt);
            rigidbody_sanitize(rigid_body);
            vector3 a3_pre_boundary_position = rigid_body->position;
            if (!main_inputs.is_debug_mode_active) {
                boundary_apply_box(rigid_body, (vector3){-250, 0, -250}, (vector3){250, 500, 250});
            } else {
                boundary_apply_floor(rigid_body, 0.0f);
            }
            if (vector3_length_squared(vector3_subtraction(rigid_body->position, a3_pre_boundary_position)) > 0.000001f) {
                a3_boundary_moved_any = true;
            }
        }
        a3_positional_depenetration_pass(obj_per_scene, object_count, persistent_collision_pairs,
                                         &detected_collision_count, a3_boundary_moved_any);
        /* AUDIT: no cache clear here (see physics_world.c): body-local
         * contact points survive rigid translation exactly, so the cache
         * stays valid across depenetration/boundary moves. Clearing every
         * substep disabled warm starting engine-wide. */
        physics_time_accumulator -= fixed_physics_dt;
    }
}
