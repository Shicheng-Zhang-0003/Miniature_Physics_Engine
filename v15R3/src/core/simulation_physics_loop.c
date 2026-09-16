/* MFS_INCREMENT_SPLIT_2: Fixed-timestep physics loop.
* Extracted from physics_step_increment in simulation.c.
* Owns: the accumulator, broadphase, narrowphase, solver, integration,
*        sleep staticize/restore, boundary, depenetration.
* LEGACY GUI PATH: operates on global world->bodies / world->body_count so the
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


/* One broadphase pair through narrowphase + wake-on-contact + solver
 * prep (legacy GUI path). Shared by the main pair loop and the sleep-wake
 * revisit pass below; see physics_world.c for the race it closes. */
static void simulation_process_pair(physics_world *world, int object_index_a, int object_index_b, float dt,
                                    collision_data *manifolds, int max_manifolds, int *manifold_count_ptr) {
    rigidbody *body_a = &world->bodies[object_index_a];
    rigidbody *body_b = &world->bodies[object_index_b];
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
                if ((*manifold_count_ptr) >= max_manifolds) {
                    debug_last_manifold_overflow_count++;
                    return;
                }
                    bool a3_a_was_sleeping = body_a->is_sleeping;
                    bool a3_b_was_sleeping = body_b->is_sleeping;
                    if (a3_a_was_sleeping && a3_b_was_sleeping) { return; }
                    /* TRUTH: three-gate wake — new edge, fast other, deep
                     * overlap (see physics_world.c). Persistent resting
                     * contact alone must not veto sleep. */
                    bool a3_new_edge = !contact_cache_has_pair(world, body_a->object_id, body_b->object_id);
                    if ((a3_a_was_sleeping) && (!body_b->static_state) && a3_new_edge) {
                        rigidbody_wake(body_a);
                    }
                    if ((a3_b_was_sleeping) && (!body_a->static_state) && a3_new_edge) {
                        rigidbody_wake(body_b);
                    }
                    float a3_wake_lin = g_cfg.sleep.wake_linear_thresh_sq;
                    float a3_wake_ang = g_cfg.sleep.wake_angular_thresh_sq;
                    bool a3_a_fast = (!a3_a_was_sleeping) &&
                                     ((vector3_length_squared(body_a->velocity) > a3_wake_lin) ||
                                      (vector3_length_squared(body_a->angular_velocity) > a3_wake_ang));
                    bool a3_b_fast = (!a3_b_was_sleeping) &&
                                     ((vector3_length_squared(body_b->velocity) > a3_wake_lin) ||
                                      (vector3_length_squared(body_b->angular_velocity) > a3_wake_ang));
                    if ((a3_a_was_sleeping) && (!body_b->static_state) && a3_b_fast) {
                        rigidbody_wake(body_a);
                    }
                    if ((a3_b_was_sleeping) && (!body_a->static_state) && a3_a_fast) {
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
                    collision_prepare_solver(world, &narrowphase_collision, &manifolds[(*manifold_count_ptr)],
                                             dt);
                    (*manifold_count_ptr)++;
                    /* TRUTH P0-1: contact flag for gravity-exactness gating. */
                    if (world->has_contact) {
                        if ((object_index_a >= 0) && (object_index_a < world->body_count)) {
                            world->has_contact[object_index_a] = 1;
                        }
                        if ((object_index_b >= 0) && (object_index_b < world->body_count)) {
                            world->has_contact[object_index_b] = 1;
                        }
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
    /* All simulation state lives in the primary world; this tick only
     * borrows it (scratch included). Worlds that failed init degrade
     * to skipped ticks, never crashes (low-memory contract). */
    physics_world *world = physics_world_get_primary();
    if ((!world) || (!world->bodies) || (!world->pair_buffer) || (!world->manifolds) ||
        (!world->manifold_awake) || (!world->manifold_order) || (!world->manifold_sort_keys) ||
        (!world->pair_skipped)) {
        return;
    }
    while (physics_time_accumulator >= fixed_physics_dt) {
        /* Sanitize all bodies */
        for (int sanitize_index = 0; sanitize_index < world->body_count; sanitize_index++) {
            rigidbody_sanitize(&world->bodies[sanitize_index]);
        }
        /* CCD: clamp fast bodies to TOI pose (see physics_world.c).
         * TRUTH P0-3: remainder recorded for post-solve integration. */
        if (world->ccd_time_remaining) {
            collision_ccd_sweep_clamp_full(world->bodies, world->body_count, fixed_physics_dt,
                                           world->ccd_time_remaining);
        } else {
            collision_ccd_sweep_clamp(world->bodies, world->body_count, fixed_physics_dt);
        }
        /* Broadphase */
        int detected_collision_count = 0;
        detected_collision_count = broadphase_generate_pairing(world, world->pair_buffer, mpe_max_broadphase_pairs, fixed_physics_dt);
        debug_last_broadphase_pair_count = detected_collision_count;
        /* Narrowphase + manifold build
         * TRUTH: unified order with physics_world_step: CCD->broad->narrow
         * (pre-gravity prepare) -> gravity/springs/motors/integrate -> solve.
         * Old legacy integrated BEFORE narrow, so warm masses/frames lived in
         * a different velocity state than headless. Refresh fixes Poisson
         * gate only, not masses/frames. */
        int manifold_count = 0;
        contact_cache_stats_reset(world);
        /* TRUTH P0-1: reset contact flags (set on every manifold below). */
        if (world->has_contact) {
            for (int hc = 0; hc < world->body_count; hc++) {
                world->has_contact[hc] = 0;
            }
        }
        /* Narrowphase dispatch */
        for (int q = 0; q < detected_collision_count; q++) {
            world->pair_skipped[q] = 0;
        }
        for (int collision_index = 0; collision_index < detected_collision_count; collision_index++) {
            int index_a = world->pair_buffer[collision_index].object_index_a;
            int index_b = world->pair_buffer[collision_index].object_index_b;
            if ((index_a < 0) || (index_a >= world->body_count) || (index_b < 0) || (index_b >= world->body_count)) {
                continue;
            }
            rigidbody *body_a = &world->bodies[index_a];
            rigidbody *body_b = &world->bodies[index_b];
            if ((body_a->is_sleeping) && (body_b->is_sleeping)) {
                world->pair_skipped[collision_index] = 1;
                continue;
            }
            simulation_process_pair(world, index_a, index_b, fixed_physics_dt, world->manifolds, a3_max_manifolds,
                                    &manifold_count);
        }
        /* Sleep-wake revisit fixpoint (see physics_world.c). */
        for (int revisit_pass = 0; revisit_pass < 16; revisit_pass++) {
            bool revisit_woke = false;
            for (int collision_index = 0; collision_index < detected_collision_count; collision_index++) {
                if (!world->pair_skipped[collision_index]) {
                    continue;
                }
                int index_a = world->pair_buffer[collision_index].object_index_a;
                int index_b = world->pair_buffer[collision_index].object_index_b;
                if ((index_a < 0) || (index_a >= world->body_count) || (index_b < 0) || (index_b >= world->body_count)) {
                    world->pair_skipped[collision_index] = 0;
                    continue;
                }
                rigidbody *body_a = &world->bodies[index_a];
                rigidbody *body_b = &world->bodies[index_b];
                if ((body_a->is_sleeping) && (body_b->is_sleeping)) {
                    continue;
                }
                bool slept_a = body_a->is_sleeping;
                bool slept_b = body_b->is_sleeping;
                simulation_process_pair(world, index_a, index_b, fixed_physics_dt, world->manifolds, a3_max_manifolds,
                                        &manifold_count);
                world->pair_skipped[collision_index] = 0;
                if ((slept_a && !body_a->is_sleeping) || (slept_b && !body_b->is_sleeping)) {
                    revisit_woke = true;
                }
            }
            if (!revisit_woke) {
                break;
            }
        }
        /* Floor collision (sleeping included: sets has_contact + deep-wake,
         * solves as no-op via eff_inv=0; see physics_world.c). */
        for (int floor_object_index = 0; floor_object_index < world->body_count; floor_object_index++) {
            rigidbody *floor_rigid_body = &world->bodies[floor_object_index];
            if (floor_rigid_body->static_state) {
                continue;
            }
            bool was_sleeping = floor_rigid_body->is_sleeping;
            collision_data floor_collision = {0};
            if (collision_static_plane_body(floor_rigid_body, 0.0f, &floor_collision)) {
                if (manifold_count < a3_max_manifolds) {
                    float deepest = 0.0f;
                    for (int fi = 0; fi < floor_collision.contact_count; fi++) {
                        if (floor_collision.contacts[fi].penetration > deepest) {
                            deepest = floor_collision.contacts[fi].penetration;
                        }
                    }
                    if (was_sleeping && deepest > g_cfg.depenetration.wake_depth_thresh) {
                        rigidbody_wake(floor_rigid_body);
                    }
                    collision_prepare_solver(world, &floor_collision, &world->manifolds[manifold_count],
                                             fixed_physics_dt);
                    manifold_count++;
                    if (world->has_contact) {
                        world->has_contact[floor_object_index] = 1;
                    }
                } else {
                    debug_last_manifold_overflow_count++;
                }
            }
        }
        debug_last_manifold_count = manifold_count;
        /* Solver islands (see physics_world.c): skip fully-sleeping islands
         * in the iteration/relaxation loops below. */
        islands_build(world, world->pair_buffer, detected_collision_count);
        for (int island_mark = 0; island_mark < manifold_count; island_mark++) {
            rigidbody *ma = world->manifolds[island_mark].object_a;
            rigidbody *mb = world->manifolds[island_mark].object_b;
            world->manifold_awake[island_mark] =
                (unsigned char) (islands_body_awake(world, ma) || islands_body_awake(world, mb));
        }
        collision_manifold_solve_order(world, world->manifolds, manifold_count, world->manifold_order);
        /* TRUTH: forces after narrow (unified with world path). */
        apply_force_all_joints(world);
        for (int gi = 0; gi < world->body_count; gi++) {
            vector3 grav_a = {0, g_cfg.world.gravity, 0};
            rigidbody *rb = &world->bodies[gi];
            if ((rb->is_sleeping) || (rb->kinematic) || (rb->static_state)) {
                continue;
            }
            rb_apply_forces_perfect(rb, vector3_scaling(grav_a, rb->mass));
        }
        constraint_apply_motors(world, fixed_physics_dt);
        for (int vi = 0; vi < world->body_count; vi++) {
            rb_integrate_velocity(&world->bodies[vi], fixed_physics_dt, linear_damping_factor,
                                  angular_damping_factor);
        }
        /* Sleeping bodies keep real mass (no staticize mutation; solver uses
         * effective-mass helpers). Zero stale accumulators only. */
        for (int sleep_staticize_index = 0; sleep_staticize_index < world->body_count; sleep_staticize_index++) {
            rigidbody *sleep_staticize_body = &world->bodies[sleep_staticize_index];
            if ((sleep_staticize_body->is_sleeping) && (!sleep_staticize_body->static_state)) {
                sleep_staticize_body->velocity = vector3_zero();
                sleep_staticize_body->angular_velocity = vector3_zero();
                sleep_staticize_body->force_accumulator = vector3_zero();
                sleep_staticize_body->torque_accumulator = vector3_zero();
            }
        }
        /* TRUTH P0-2: refresh Poisson gate to post-integration velocities. */
        collision_refresh_impact_velocities(world->manifolds, manifold_count);
        /* Solver iterations */
        int solver_iterations = g_cfg.timestep.solver_iterations;
        if (solver_iterations < 1) {
            solver_iterations = 1;
        }
        if (solver_iterations > 128) {
            solver_iterations = 128;
        }
        constraint_pre_step_all(world, fixed_physics_dt);
        for (int iter = 0; iter < solver_iterations; iter++) {
            for (int o = 0; o < manifold_count; o++) {
                int m = world->manifold_order[o];
                if (!world->manifold_awake[m]) {
                    continue;
                }
                /* Local convergence (see physics_world.c): settle the
                 * manifold's coupled contacts before propagating upward. */
                collision_resolve_iterative(&world->manifolds[m], fixed_physics_dt, false, iter);
                collision_resolve_iterative(&world->manifolds[m], fixed_physics_dt, false, iter + 1);
            }
            /* FIX-AUDIT: revolute constraints were never solved on the
             * legacy path (only in physics_world). Solve inside the
             * iteration loop so contact/joint impulses couple — mirroring
             * physics_world_step(). */
            constraint_solve_all(world, fixed_physics_dt);
        }
        /* Positional axis-drift correction: exactly once per substep, never
         * in the loop above (see revolute_joint.c). */
        constraint_correct_axis_drift_all(world, fixed_physics_dt);
        /* Poisson restitution + joint relaxation + friction relaxation
         * (see physics_world.c). */
        collision_apply_poisson_restitution(world->manifolds, manifold_count);
        constraint_solve_all(world, fixed_physics_dt);
        for (int relax_iter = 0; relax_iter < 2; relax_iter++) {
            for (int o = 0; o < manifold_count; o++) {
                int m = world->manifold_order[o];
                if (!world->manifold_awake[m]) {
                    continue;
                }
                collision_resolve_iterative(&world->manifolds[m], fixed_physics_dt, true, relax_iter);
            }
        }
        /* TRUTH: split was missing on legacy (sank to slop). Add it + the
         * post-split wake revisit (see physics_world.c). */
        collision_apply_split_impulse(world->manifolds, manifold_count, fixed_physics_dt);
        for (int p = 0; p < detected_collision_count; p++) {
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
            simulation_process_pair(world, ia, ib, fixed_physics_dt, world->manifolds, a3_max_manifolds,
                                    &manifold_count);
            world->pair_skipped[p] = 0;
        }
        /* Rolling resistance once per tick (uses solved normal impulses). */
        collision_apply_rolling_resistance(world->manifolds, manifold_count, fixed_physics_dt);
        contact_cache_save(world, world->manifolds, manifold_count); /* MFS_131A: legacy global fallback cache */
        /* No sleep restore needed (no staticize was applied). Pin velocities. */
        for (int sleep_restore_index = 0; sleep_restore_index < world->body_count; sleep_restore_index++) {
            rigidbody *sleep_restore_body = &world->bodies[sleep_restore_index];
            if ((sleep_restore_body->is_sleeping) && (!sleep_restore_body->static_state)) {
                sleep_restore_body->velocity = vector3_zero();
                sleep_restore_body->angular_velocity = vector3_zero();
            }
        }
        /* Integrate position + boundary + depenetration
         * TRUTH P0-1+P0-3: CCD remainder + gravity-exact free flight. */
        bool a3_boundary_moved_any = false;
        float grav_half_leg = -0.5f * g_cfg.world.gravity;
        for (int object_iterator_index = 0; object_iterator_index < world->body_count; object_iterator_index++) {
            rigidbody *rigid_body = &world->bodies[object_iterator_index];
            float step_dt = fixed_physics_dt;
            if ((world->ccd_time_remaining) && (world->ccd_time_remaining[object_iterator_index] < step_dt) &&
                (world->ccd_time_remaining[object_iterator_index] > 0.0f)) {
                step_dt = world->ccd_time_remaining[object_iterator_index];
            }
            rb_integrate_position(rigid_body, step_dt);
            bool free_flight = (world->has_contact) ? (world->has_contact[object_iterator_index] == 0) : true;
            if (free_flight && (!rigid_body->static_state) && (!rigid_body->is_sleeping) &&
                (!rigid_body->kinematic) && (step_dt > 0.0f)) {
                rigid_body->position.y += grav_half_leg * step_dt * step_dt;
            }
            rigidbody_sanitize(rigid_body);
            /* TRUTH: unified safety net with world path (box always). Old
             * debug floor-only let bodies escape sideways in debug, diverging
             * from headless. Solver owns normal contact; boundary is plastic. */
            vector3 a3_pre_boundary_position = rigid_body->position;
            boundary_apply_box(rigid_body, (vector3){-250, 0, -250}, (vector3){250, 500, 250});
            if (vector3_length_squared(vector3_subtraction(rigid_body->position, a3_pre_boundary_position)) > 0.000001f) {
                a3_boundary_moved_any = true;
            }
        }
        a3_positional_depenetration_pass_dt(world, world->pair_buffer, &detected_collision_count,
                                            a3_boundary_moved_any, fixed_physics_dt);
        /* AUDIT: no cache clear here (see physics_world.c): body-local
         * contact points survive rigid translation exactly, so the cache
         * stays valid across depenetration/boundary moves. Clearing every
         * substep disabled warm starting engine-wide. */
        physics_time_accumulator -= fixed_physics_dt;
    }
}
