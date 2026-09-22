/* MFS_INCREMENT_SPLIT_2: Fixed-timestep physics loop.
* Extracted from physics_step_increment in simulation.c.
* Owns: the accumulator, broadphase, narrowphase, solver, integration,
*        sleep staticize/restore, boundary, depenetration.
* LEGACY GUI PATH: operates on global world->bodies / world->body_count so the
* GTK render/editor loop keeps working. Canonical stepping is
* physics_world_step() in core/physics_world.c (used by all headless
* tests). Do not add new simulation state here — add it to physics_world.
*/
/* GTK4-PREP: zero GUI headers in core. */
#include "../config/mpe_config.h"
#include "../config/mpe_constants.h"
#include "physics_world.h"
#include "debug_counters.h"
#include "det_math.h"
#include "../physics/depenetration.h"
#include "../physics/constraint.h"
#include "../physics/islands.h"
#include "../physics/broadphase.h"
#include "../physics/collision_mechanics.h"
#include "../physics/spring_joint_types.h"
#include "../scene/boundary.h"
#include "../ui_input/input_state.h"
#include "../ui_input/camera.h"
#include "frame_timer.h"
#include <math.h>
#include <stdbool.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

extern input_status main_inputs;
extern camera main_camera_fov;
extern int selected_object;
extern frame_timer main_timer;
/* Canonical spring pass (weak: spring-less headless links skip it). */
void mpe_springs_apply(physics_world *world, float dt) __attribute__((weak));
/* Forward from simulation.c / overlay / debug_terminal (now headless-clean). */
void simulation_camera_tick(float frame_delta_time);
void simulation_input_dispatch(void *parent_window);
void simulation_menu_dispatch(void *parent_window);
void editor_update_menus(void *parent_window);
void config_menu_update(void *parent_window);
void config_menu_close(void);
int config_menu_is_open(void);
void debug_terminal_sync_mode(void);
bool editor_dialog_is_active(void);
bool physics_halt_tick_update(void);
void overlay_update(void);
void long_run_validation_tick_update(void);


/* Pair pipeline relic removed: the legacy GUI tick shares
 * physics_world_process_pair (registry dispatch + 3-gate wake) with the
 * canonical step so both paths stay identical. */

void simulation_physics_tick(float frame_delta_time) {
    /* FIX-AUDIT: static float accumulator bled across scene loads and lost
     * precision on long runs. Double precision here. */
    static double physics_time_accumulator = 0.0;
    const float fixed_physics_dt = 1.0f / 60.0f;
    /* Legacy GUI path is primary-world-only by design; substep/damping
     * config comes from the primary world's binding (defaults global). */
    const mpe_config_t *leg_cfg = mpe_world_cfg(physics_world_get_primary());
    /* FIX-AUDIT: max_substeps was hardcoded 5, ignoring
     * timestep.max_substeps (1..20). */
    int max_substeps_per_frame = leg_cfg->timestep.max_substeps;
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
        (float) det_pow_retention((double) leg_cfg->world.drag, (double) fixed_physics_dt);
    /* TRUTH: scale=1.0 means no extra rotary damping (matches canonical). */
    float angular_damping_factor = (leg_cfg->world.angular_damping_scale >= 1.0f)
                                       ? 1.0f
                                       : (float) det_pow_retention(
                                             (double) (leg_cfg->world.drag * leg_cfg->world.angular_damping_scale),
                                             (double) fixed_physics_dt);
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
    /* World overflow counter is cumulative until clear; the overlay debug
     * counter keeps per-frame accumulate semantics (reset above). */
    while (physics_time_accumulator >= fixed_physics_dt) {
        int ovfl_sub_mark = world->manifold_overflow_count;
        /* Sanitize all bodies */
        for (int sanitize_index = 0; sanitize_index < world->body_count; sanitize_index++) {
            rigidbody_sanitize(&world->bodies[sanitize_index]);
        }
        /* TRUTH: reset per-tick relative-speed scratch for sleep gating
         * (mirrors physics_world_step; prepare writes the max). Without
         * reset the legacy path accumulates stale maxima forever. */
        for (int rel_index = 0; rel_index < world->body_count; rel_index++) {
            world->bodies[rel_index].max_relative_speed_sq = 0.0f;
        }
        /* CCD: clamp fast bodies to TOI pose (see physics_world.c).
         * TRUTH P0-3: remainder recorded for post-solve integration. */
        collision_ccd_sweep_clamp_world(world, fixed_physics_dt);
        /* Broadphase (per-world backend override supported). */
        int detected_collision_count = 0;
        if (world->broadphase_if && world->broadphase_if->generate) {
            detected_collision_count = world->broadphase_if->generate(world, world->pair_buffer,
                                                                      mpe_max_broadphase_pairs,
                                                                      fixed_physics_dt, NULL);
        } else {
            detected_collision_count =
                broadphase_generate_pairing(world, world->pair_buffer, mpe_max_broadphase_pairs,
                                            fixed_physics_dt);
        }
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
            physics_world_process_pair(world, index_a, index_b, fixed_physics_dt, &manifold_count);
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
                physics_world_process_pair(world, index_a, index_b, fixed_physics_dt, &manifold_count);
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
        if (world->static_plane_enabled) {
            for (int floor_object_index = 0; floor_object_index < world->body_count; floor_object_index++) {
                rigidbody *floor_rigid_body = &world->bodies[floor_object_index];
                if (floor_rigid_body->static_state) {
                    continue;
                }
                bool was_sleeping = floor_rigid_body->is_sleeping;
                collision_data floor_collision = {0};
                if (collision_static_plane_body(&world->static_plane_body, floor_rigid_body, 0.0f, &floor_collision,
                                                mpe_world_cfg(world))) {
                    if (manifold_count < a3_max_manifolds) {
                        float deepest = 0.0f;
                        for (int fi = 0; fi < floor_collision.contact_count; fi++) {
                            if (floor_collision.contacts[fi].penetration > deepest) {
                                deepest = floor_collision.contacts[fi].penetration;
                            }
                        }
                        if (was_sleeping && deepest > mpe_world_cfg(world)->depenetration.wake_depth_thresh) {
                            rigidbody_wake(floor_rigid_body);
                        }
                        collision_prepare_solver(world, &floor_collision, &world->manifolds[manifold_count],
                                                 fixed_physics_dt);
                        manifold_count++;
                        if (world->has_contact) {
                            world->has_contact[floor_object_index] = 1;
                        }
                    } else {
                        world->manifold_overflow_count++;
                    }
                }
            }
        }
        debug_last_manifold_count = manifold_count;
        /* Overlay overflow readout: both pair and floor paths count into
         * the world counter; debug accumulates this substep's delta. */
        debug_last_manifold_overflow_count += world->manifold_overflow_count - ovfl_sub_mark;
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
        /* TRUTH: forces after narrow (unified with world path). Springs via
         * the canonical weak entry (see physics_world.c). */
        if (mpe_springs_apply) {
            mpe_springs_apply(world, fixed_physics_dt);
        }
        for (int gi = 0; gi < world->body_count; gi++) {
            vector3 grav_a = {0, mpe_world_cfg(world)->world.gravity, 0};
            rigidbody *rb = &world->bodies[gi];
            if ((rb->is_sleeping) || (rb->kinematic) || (rb->static_state)) {
                continue;
            }
            rb_apply_forces_perfect(rb, vector3_scaling(grav_a, rb->mass));
        }
        constraint_apply_motors(world, fixed_physics_dt);
        /* Foreign forcefield / motor modules (same hook as world path). */
        for (int mi = 0; mi < world->tick_module_count; mi++) {
            if (world->tick_modules[mi] && world->tick_modules[mi]->pre_step) {
                world->tick_modules[mi]->pre_step(world, fixed_physics_dt, world->tick_module_state[mi]);
            }
        }
        if (world->tick_v0 && world->tick_v0_capacity >= mpe_max_bodies) {
            for (int si = 0; si < world->body_count; si++) {
                world->tick_v0[si] = world->bodies[si].velocity;
            }
        }
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
        /* Solver iterations (per-world config). */
        const mpe_config_t *leg_step_cfg = mpe_world_cfg(world);
        int solver_iterations = leg_step_cfg->timestep.solver_iterations;
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
                if (world->solver_if && world->solver_if->resolve) {
                    world->solver_if->resolve(world, &world->manifolds[m], fixed_physics_dt, false, iter, NULL);
                    world->solver_if->resolve(world, &world->manifolds[m], fixed_physics_dt, false, iter + 1,
                                              NULL);
                } else {
                    collision_resolve_iterative(&world->manifolds[m], fixed_physics_dt, false, iter,
                                                leg_step_cfg);
                    collision_resolve_iterative(&world->manifolds[m], fixed_physics_dt, false, iter + 1,
                                                leg_step_cfg);
                }
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
        if (world->solver_if && world->solver_if->poisson) {
            world->solver_if->poisson(world, world->manifolds, manifold_count, NULL);
        } else {
            collision_apply_poisson_restitution(world->manifolds, manifold_count, leg_step_cfg);
        }
        constraint_solve_all(world, fixed_physics_dt);
        for (int relax_iter = 0; relax_iter < 2; relax_iter++) {
            for (int o = 0; o < manifold_count; o++) {
                int m = world->manifold_order[o];
                if (!world->manifold_awake[m]) {
                    continue;
                }
                if (world->solver_if && world->solver_if->resolve) {
                    world->solver_if->resolve(world, &world->manifolds[m], fixed_physics_dt, true, relax_iter,
                                              NULL);
                } else {
                    collision_resolve_iterative(&world->manifolds[m], fixed_physics_dt, true, relax_iter,
                                                leg_step_cfg);
                }
            }
        }
        /* TRUTH: split was missing on legacy (sank to slop). Add it + the
         * post-split wake revisit (see physics_world.c). */
        if (world->solver_if && world->solver_if->split) {
            world->solver_if->split(world, world->manifolds, manifold_count, fixed_physics_dt, NULL);
        } else {
            collision_apply_split_impulse(world->manifolds, manifold_count, fixed_physics_dt, leg_step_cfg);
        }
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
            physics_world_process_pair(world, ia, ib, fixed_physics_dt, &manifold_count);
            world->pair_skipped[p] = 0;
        }
        /* Rolling resistance once per tick (uses solved normal impulses). */
        if (world->solver_if && world->solver_if->rolling) {
            world->solver_if->rolling(world, world->manifolds, manifold_count, fixed_physics_dt, NULL);
        } else {
            collision_apply_rolling_resistance(world->manifolds, manifold_count, fixed_physics_dt, leg_step_cfg);
        }
        contact_cache_save(world, world->manifolds, manifold_count); /* MFS_131A: legacy global fallback cache */
        /* No sleep restore needed (no staticize was applied). Pin velocities. */
        for (int sleep_restore_index = 0; sleep_restore_index < world->body_count; sleep_restore_index++) {
            rigidbody *sleep_restore_body = &world->bodies[sleep_restore_index];
            if ((sleep_restore_body->is_sleeping) && (!sleep_restore_body->static_state)) {
                sleep_restore_body->velocity = vector3_zero();
                sleep_restore_body->angular_velocity = vector3_zero();
            }
        }
        /* Integrate position + boundary + depenetration.
         * Exact free-flight lives in rb_integrate_position_exact (analytic
         * from v_pre, includes 0.5*g*dt^2). rb_integrate_position is the
         * SAFE default (constrained symplectic on live velocity); free
         * flight must call exact explicitly after restoring v_pre. No
         * manual half-leg anywhere: it would double-count gravity. */
        bool a3_boundary_moved_any = false;
        const mpe_config_t *leg_pos_cfg = mpe_world_cfg(world);
        /* PHYSICS-FIX: legacy path now shares the canonical joint gate.
         * Jointed contact-free bodies are constrained (joint impulses own
         * their velocity); analytic free-flight from v_pre would discard
         * the joint solve. Bitmap precomputed once O(J+B). */
        static _Thread_local unsigned char leg_joint[mpe_max_bodies];
        {
            int n = world->body_count;
            if (n > mpe_max_bodies) {
                n = mpe_max_bodies;
            }
            memset(leg_joint, 0, (size_t) n);
            for (int ji = 0; ji < mpe_max_joints; ji++) {
                if (!world->revolute_constraints[ji].is_active) {
                    continue;
                }
                int ia = physics_world_index_by_id(world, world->revolute_constraints[ji].body_id_a);
                int ib = physics_world_index_by_id(world, world->revolute_constraints[ji].body_id_b);
                if (ia >= 0 && ia < n) {
                    leg_joint[ia] = 1;
                }
                if (ib >= 0 && ib < n) {
                    leg_joint[ib] = 1;
                }
            }
            for (int ji = 0; ji < mpe_max_joints; ji++) {
                if (!world->spring_joints[ji].is_active) {
                    continue;
                }
                int ia = physics_world_index_by_id(world, world->spring_joints[ji].object_id_a);
                int ib = physics_world_index_by_id(world, world->spring_joints[ji].object_id_b);
                if (ia >= 0 && ia < n) {
                    leg_joint[ia] = 1;
                }
                if (ib >= 0 && ib < n) {
                    leg_joint[ib] = 1;
                }
            }
        }
        for (int object_iterator_index = 0; object_iterator_index < world->body_count; object_iterator_index++) {
            rigidbody *rigid_body = &world->bodies[object_iterator_index];
            float step_dt = fixed_physics_dt;
            if ((world->ccd_time_remaining) && (world->ccd_time_remaining[object_iterator_index] < step_dt) &&
                (world->ccd_time_remaining[object_iterator_index] > 0.0f)) {
                step_dt = world->ccd_time_remaining[object_iterator_index];
            }
            /* Contact-free AND joint-free bodies take analytic free-flight
             * from v_pre; constrained bodies keep post-solve velocity
             * (solver/joints own them). Per-world cfg (was &g_cfg global):
             * multi-world gravity/drag diverge from canonical otherwise. */
            bool leg_contact_free =
                (world->has_contact) ? (world->has_contact[object_iterator_index] == 0) : true;
            bool leg_joint_free = (object_iterator_index < mpe_max_bodies)
                ? (leg_joint[object_iterator_index] == 0)
                : false;
            bool leg_free = leg_contact_free && leg_joint_free;
            if (leg_free && world->tick_v0 && world->tick_v0_capacity >= mpe_max_bodies) {
                rigid_body->velocity = world->tick_v0[object_iterator_index];
            }
            rb_integrate_position_exact(rigid_body, step_dt, leg_pos_cfg, leg_free);
            rigidbody_sanitize(rigid_body);
            /* TRUTH: unified safety net with world path (box always). Old
             * debug floor-only let bodies escape sideways in debug, diverging
             * from headless. Solver owns normal contact; boundary is plastic. */
            vector3 a3_pre_boundary_position = rigid_body->position;
            boundary_apply_box_cfg(rigid_body, (vector3){-250, 0, -250}, (vector3){250, 500, 250},
                                   mpe_world_cfg(world));
            if (vector3_length_squared(vector3_subtraction(rigid_body->position, a3_pre_boundary_position)) > 0.000001f) {
                a3_boundary_moved_any = true;
            }
        }
        a3_positional_depenetration_pass_dt(world, world->pair_buffer, &detected_collision_count,
                                            a3_boundary_moved_any, fixed_physics_dt);
        /* Foreign post-step modules (same hook as world path). */
        for (int mi = 0; mi < world->tick_module_count; mi++) {
            if (world->tick_modules[mi] && world->tick_modules[mi]->post_step) {
                world->tick_modules[mi]->post_step(world, fixed_physics_dt, world->tick_module_state[mi]);
            }
        }
        /* AUDIT: no cache clear here (see physics_world.c): body-local
         * contact points survive rigid translation exactly, so the cache
         * stays valid across depenetration/boundary moves. Clearing every
         * substep disabled warm starting engine-wide. */
        physics_time_accumulator -= fixed_physics_dt;
    }
}
