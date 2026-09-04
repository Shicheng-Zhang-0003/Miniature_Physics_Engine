/* MFS_INCREMENT_SPLIT_2: Fixed-timestep physics loop.
* Extracted from physics_step_increment in simulation.c.
* Owns: the accumulator, broadphase, narrowphase, solver, integration,
*        sleep staticize/restore, boundary, depenetration.
* Still operates on global obj_per_scene / object_count (legacy path).
*/
#include "../mpe_engine.h"
#include "../physics/depenetration.h"
#include <math.h>

static broadphase_pair persistent_collision_pairs[mpe_max_broadphase_pairs];

void simulation_physics_tick(float frame_delta_time) {
    static float physics_time_accumulator = 0.0f;
    const float fixed_physics_dt = 1.0f / 60.0f;
    const int max_substeps_per_frame = 5;

    physics_time_accumulator += frame_delta_time;
    if (physics_time_accumulator > fixed_physics_dt * max_substeps_per_frame) {
        physics_time_accumulator = fixed_physics_dt * max_substeps_per_frame;
    }

    float linear_damping_factor = powf(g_cfg.world.drag, fixed_physics_dt);
    float angular_damping_factor = powf(g_cfg.world.drag * 0.97f, fixed_physics_dt);

    debug_last_manifold_overflow_count = 0;

    while (physics_time_accumulator >= fixed_physics_dt) {
        /* Sanitize all bodies */
        for (int sanitize_index = 0; sanitize_index < object_count; sanitize_index++) {
            rigidbody_sanitize(&obj_per_scene[sanitize_index]);
        }

        /* Broadphase */
        int detected_collision_count = 0;
        detected_collision_count = broadphase_generate_pairing(
            obj_per_scene, object_count, persistent_collision_pairs, mpe_max_broadphase_pairs);
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
            if (rigid_body->is_sleeping) { continue; }
            rb_apply_forces_perfect(rigid_body, vector3_scaling(constant_gravity_acceleration, rigid_body->mass));
        }

        /* Integrate velocity */
        for (int velocity_integration_index = 0; velocity_integration_index < object_count;
             velocity_integration_index++) {
            rb_integrate_velocity(&obj_per_scene[velocity_integration_index], fixed_physics_dt,
                                  linear_damping_factor, angular_damping_factor);
        }

        /* Narrowphase dispatch */
        for (int collision_index = 0; collision_index < detected_collision_count; collision_index++) {
            rigidbody *rigid_body_a = &obj_per_scene[persistent_collision_pairs[collision_index].object_index_a];
            rigidbody *rigid_body_b = &obj_per_scene[persistent_collision_pairs[collision_index].object_index_b];

            if ((rigid_body_a->is_sleeping) && (rigid_body_b->is_sleeping)) { continue; }

            collision_data narrowphase_collision = {0};
            bool collided = false;

            if (rigid_body_a->type == object_sphere && rigid_body_b->type == object_sphere)
                collided = collision_dual_sphere(rigid_body_a, rigid_body_b, &narrowphase_collision);
            else if (rigid_body_a->type == object_sphere && rigid_body_b->type == object_cube)
                collided = collision_sphere_cube(rigid_body_a, rigid_body_b, &narrowphase_collision);
            else if (rigid_body_a->type == object_cube && rigid_body_b->type == object_sphere) {
                collided = collision_sphere_cube(rigid_body_b, rigid_body_a, &narrowphase_collision);
                narrowphase_collision.normal_vector = vector3_scaling(narrowphase_collision.normal_vector, -1.0f);
                narrowphase_collision.object_a = rigid_body_a;
                narrowphase_collision.object_b = rigid_body_b;
            } else if (rigid_body_a->type == object_cube && rigid_body_b->type == object_cube)
                collided = collision_dual_cube(rigid_body_a, rigid_body_b, &narrowphase_collision);

            if (collided) {
                if (manifold_count < a3_max_manifolds) {
                    bool a3_a_was_sleeping = rigid_body_a->is_sleeping;
                    bool a3_b_was_sleeping = rigid_body_b->is_sleeping;
                    if (a3_a_was_sleeping && a3_b_was_sleeping) { continue; }
                    float a3_wake_linear_threshold_sq = g_cfg.sleep.wake_linear_thresh_sq;
                    float a3_wake_angular_threshold_sq = g_cfg.sleep.wake_angular_thresh_sq;
                    bool a3_a_is_active = (!a3_a_was_sleeping) &&
                        ((vector3_length_squared(rigid_body_a->velocity) > a3_wake_linear_threshold_sq) ||
                         (vector3_length_squared(rigid_body_a->angular_velocity) > a3_wake_angular_threshold_sq));
                    bool a3_b_is_active = (!a3_b_was_sleeping) &&
                        ((vector3_length_squared(rigid_body_b->velocity) > a3_wake_linear_threshold_sq) ||
                         (vector3_length_squared(rigid_body_b->angular_velocity) > a3_wake_angular_threshold_sq));
                    if (a3_a_was_sleeping && (!rigid_body_b->static_state) && a3_b_is_active) {
                        rigidbody_wake(rigid_body_a);
                    }
                    if (a3_b_was_sleeping && (!rigid_body_a->static_state) && a3_a_is_active) {
                        rigidbody_wake(rigid_body_b);
                    }
                    collision_prepare_solver(&narrowphase_collision, &active_manifold[manifold_count]);
                    manifold_count++;
                } else {
                    debug_last_manifold_overflow_count++;
                }
            }
        }

        /* Floor collision */
        for (int floor_object_index = 0; floor_object_index < object_count; floor_object_index++) {
            rigidbody *floor_rigid_body = &obj_per_scene[floor_object_index];
            if ((floor_rigid_body->static_state) || (floor_rigid_body->is_sleeping)) { continue; }
            collision_data floor_collision = {0};
            if (collision_static_plane_body(floor_rigid_body, 0.0f, &floor_collision)) {
                if (manifold_count < a3_max_manifolds) {
                    collision_prepare_solver(&floor_collision, &active_manifold[manifold_count]);
                    manifold_count++;
                } else {
                    debug_last_manifold_overflow_count++;
                }
            }
        }

        debug_last_manifold_count = manifold_count;

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
            for (int m = 0; m < manifold_count; m++) {
                collision_resolve_iterative(&active_manifold[m]);
            }
        }
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
        a3_positional_depenetration_pass(persistent_collision_pairs, &detected_collision_count, a3_boundary_moved_any);

        physics_time_accumulator -= fixed_physics_dt;
    }
}
