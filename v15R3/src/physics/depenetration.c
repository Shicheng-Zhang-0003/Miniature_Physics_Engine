/* MFS_PHASE_A: positional depenetration extracted from simulation.c.
 * Resolves residual penetration between body pairs after the impulse solve.
 */
#include "depenetration.h"
#include "../mpe_engine.h"
#include "collision_mechanics.h"

bool a3_depenetration_dispatch(rigidbody *rigid_body_a, rigidbody *rigid_body_b,
                                      collision_data *collision_output) {
    if ((rigid_body_a->type == object_sphere) && (rigid_body_b->type == object_sphere)) {
        return collision_dual_sphere(rigid_body_a, rigid_body_b, collision_output);
    }
    if ((rigid_body_a->type == object_sphere) && (rigid_body_b->type == object_cube)) {
        return collision_sphere_cube(rigid_body_a, rigid_body_b, collision_output);
    }
    if ((rigid_body_a->type == object_cube) && (rigid_body_b->type == object_sphere)) {
        bool collided = collision_sphere_cube(rigid_body_b, rigid_body_a, collision_output);
        if (collided) {
            collision_output->normal_vector = vector3_scaling(collision_output->normal_vector, -1.0f);
            collision_output->object_a = rigid_body_a;
            collision_output->object_b = rigid_body_b;
        }
        return collided;
    }
    if ((rigid_body_a->type == object_cube) && (rigid_body_b->type == object_cube)) {
        return collision_dual_cube(rigid_body_a, rigid_body_b, collision_output);
    }
    /* FIX-AUDIT: cylinders were excluded -> no positional pass, only
     * velocity Baumgarte. Add all cylinder branches (normal flip on swap
     * mirrors the sphere/cube swap above). Bisect verified these branches
     * do not affect the wheel-rig settle (floor is a plane path). */
    if ((rigid_body_a->type == object_cylinder) && (rigid_body_b->type == object_sphere)) {
        return collision_cylinder_sphere(rigid_body_a, rigid_body_b, collision_output);
    }
    if ((rigid_body_a->type == object_sphere) && (rigid_body_b->type == object_cylinder)) {
        bool collided = collision_cylinder_sphere(rigid_body_b, rigid_body_a, collision_output);
        if (collided) {
            collision_output->normal_vector = vector3_scaling(collision_output->normal_vector, -1.0f);
            collision_output->object_a = rigid_body_a;
            collision_output->object_b = rigid_body_b;
        }
        return collided;
    }
    if ((rigid_body_a->type == object_cylinder) && (rigid_body_b->type == object_cube)) {
        return collision_cylinder_cube(rigid_body_a, rigid_body_b, collision_output);
    }
    if ((rigid_body_a->type == object_cube) && (rigid_body_b->type == object_cylinder)) {
        bool collided = collision_cylinder_cube(rigid_body_b, rigid_body_a, collision_output);
        if (collided) {
            collision_output->normal_vector = vector3_scaling(collision_output->normal_vector, -1.0f);
            collision_output->object_a = rigid_body_a;
            collision_output->object_b = rigid_body_b;
        }
        return collided;
    }
    if ((rigid_body_a->type == object_cylinder) && (rigid_body_b->type == object_cylinder)) {
        return collision_cylinder_cylinder(rigid_body_a, rigid_body_b, collision_output);
    }
    return false;
}

/* Single depenetration implementation (see header).
 * TRUTH note: sequential with split impulse, NOT double-correction of the
 * same penetration. Split impulse corrects pre-integration penetration at
 * solve time; bodies then MOVE (Verlet remainder integration), creating NEW
 * overlaps; this pass corrects those post-move residuals + boundary shoves.
 * Slop-gated (solver slop, single) + 0.5mm early-out: routine sub-mm
 * residuals pass through untouched. Deep pathology (spawn overlap) resolves
 * over ticks bounded by max_correction (teleport bounded, counted). */
void a3_positional_depenetration_pass(struct physics_world *world, broadphase_pair *pair_buffer,
                                      int *pair_count_pointer, bool rebuild_broadphase) {
    a3_positional_depenetration_pass_dt(world, pair_buffer, pair_count_pointer, rebuild_broadphase, 1.0f / 60.0f);
}

void a3_positional_depenetration_pass_dt(struct physics_world *world, broadphase_pair *pair_buffer,
                                         int *pair_count_pointer, bool rebuild_broadphase, float dt) {
    if ((!world) || (!world->bodies) || (world->body_count < 2) || (!pair_buffer) || (!pair_count_pointer)) {
        return;
    }
    if (!(dt > 0.0f) || !isfinite(dt)) {
        dt = 1.0f / 60.0f;
    }
    rigidbody *bodies = world->bodies;
    int body_count = world->body_count;

    int pair_count = *pair_count_pointer;

    if (rebuild_broadphase) {
        pair_count = broadphase_generate_pairing(world, pair_buffer, mpe_max_broadphase_pairs, dt);
        *pair_count_pointer = pair_count;
    }

    int depenetration_iterations = rebuild_broadphase ? g_cfg.depenetration.rebuild_iterations : 1; /* MPE_TASK_30 */

    for (int dep_iteration = 0; dep_iteration < depenetration_iterations; dep_iteration++) {
        for (int pair_index = 0; pair_index < pair_count; pair_index++) {
            int index_a = pair_buffer[pair_index].object_index_a;
            int index_b = pair_buffer[pair_index].object_index_b;

            if ((index_a < 0) || (index_a >= body_count)) {
                continue;
            }
            if ((index_b < 0) || (index_b >= body_count)) {
                continue;
            }

            rigidbody *body_a = &bodies[index_a];
            rigidbody *body_b = &bodies[index_b];

            collision_data depenetration_collision = {0};

            if (a3_depenetration_dispatch(body_a, body_b, &depenetration_collision)) {
                a3_positional_depenetrate_manifold(&depenetration_collision);
            }
        }

        for (int object_index = 0; object_index < body_count; object_index++) {
            rigidbody *rigid_body = &bodies[object_index];
            if (rigid_body->static_state || rigid_body->kinematic) {
                continue;
            }

            collision_data floor_collision = {0};

            if (collision_static_plane_body(rigid_body, 0.0f, &floor_collision)) {
                a3_positional_depenetrate_manifold(&floor_collision);
            }
        }
    }
}

void a3_positional_depenetrate_manifold(collision_data *manifold) {
    if ((!manifold) || (manifold->contact_count <= 0)) {
        return;
    }

    rigidbody *body_a = manifold->object_a;
    rigidbody *body_b = manifold->object_b;

    if ((!body_a) || (!body_b)) {
        return;
    }

    float normal_length_squared = vector3_length_squared(manifold->normal_vector);
    if ((!isfinite(normal_length_squared)) || (normal_length_squared < 0.000001f)) {
        return;
    }

    float max_depth = 0.0f;
    float depth_sum = 0.0f;
    int depth_count = 0;

    /* TRUTH P0-12: single slop. Solver slop is the sole overlap tolerance;
     * a second depenetration slop (5mm vs solver 10mm) makes the passes
     * fight (limit-cycle jitter). Depenetration honors solver slop. */
    const float penetration_slop = g_cfg.solver.penetration_slop;

    for (int contact_index = 0; contact_index < manifold->contact_count; contact_index++) {
        float depth = manifold->contacts[contact_index].penetration;
        if (depth > max_depth) {
            max_depth = depth;
        }
        if (depth > penetration_slop) {
            depth_sum += depth;
            depth_count++;
        }
    }

    if (max_depth <= 0.0005f) {
        return;
    }

    bool a_sleeping = (body_a->is_sleeping) && (!body_a->static_state);
    bool b_sleeping = (body_b->is_sleeping) && (!body_b->static_state);

    /* Wake sleeping bodies only when the overlap is meaningful. */
    if ((a_sleeping) && (b_sleeping) && (max_depth > g_cfg.depenetration.wake_depth_thresh)) {
        rigidbody_wake(body_a);
        rigidbody_wake(body_b);
        a_sleeping = false;
        b_sleeping = false;
    }

    if ((a_sleeping) && (body_b->static_state) && (max_depth > g_cfg.depenetration.wake_depth_thresh)) {
        rigidbody_wake(body_a);
        a_sleeping = false;
    }

    if ((b_sleeping) && (body_a->static_state) && (max_depth > g_cfg.depenetration.wake_depth_thresh)) {
        rigidbody_wake(body_b);
        b_sleeping = false;
    }

    float inverse_mass_a = rigidbody_effective_inv_mass(body_a);
    float inverse_mass_b = rigidbody_effective_inv_mass(body_b);
    /* TRUTH: sleeping bodies with deep overlap already woken above, so
     * effective (zero for still-sleeping) is correct. Kinematic must use
     * effective (zero), never raw stored inv (nonzero) — old code moved
     * kinematics. */
    if (a_sleeping) {
        inverse_mass_a = 0.0f;
    }
    if (b_sleeping) {
        inverse_mass_b = 0.0f;
    }
    float inverse_mass_sum = inverse_mass_a + inverse_mass_b;

    if (inverse_mass_sum <= 0.0f) {
        return;
    }

    if (depth_count == 0) {
        depth_sum = max_depth;
        depth_count = 1;
    }

    /* FIX-AUDIT: use max_depth (not average) to fully correct the deepest
     * corner per pass. Average under-corrects by (max-avg)*factor, causing
     * leaning stacks. */
    float correction_magnitude =
        (max_depth - penetration_slop) * g_cfg.depenetration.correction_factor / inverse_mass_sum; /* MPE_TASK_30 */

    if (correction_magnitude <= 0.0f) {
        return;
    }
    if (correction_magnitude > g_cfg.depenetration.max_correction) {
        correction_magnitude = g_cfg.depenetration.max_correction;
    }

    vector3 correction_vector = vector3_scaling(manifold->normal_vector, correction_magnitude);

    if (inverse_mass_a > 0.0f) {
        body_a->position = vector3_subtraction(body_a->position, vector3_scaling(correction_vector, inverse_mass_a));
        if (correction_magnitude > 0.01f) {
            rigidbody_wake(body_a);
        }
    }

if (inverse_mass_b > 0.0f) {
        body_b->position = vector3_addition(body_b->position, vector3_scaling(correction_vector, inverse_mass_b));
        if (correction_magnitude > 0.01f) {
            rigidbody_wake(body_b);
        }
    }
}
