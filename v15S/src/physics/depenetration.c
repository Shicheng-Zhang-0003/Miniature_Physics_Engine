/* MFS_PHASE_A: positional depenetration extracted from simulation.c.
 * Resolves residual penetration between body pairs after the impulse solve.
 */
/* GTK4-PREP: zero GUI headers in physics. */
#include "depenetration.h"
#include "collision_mechanics.h"
#include "../core/physics_world.h"
#include "../core/rigidbody.h"
#include "../config/mpe_config.h"
#include "../config/mpe_constants.h"
#include "broadphase.h"
#include <stdbool.h>
#include <math.h>

/* Narrowphase dispatch relic removed: all pair routing goes through the
 * shape registry (mpe_shape_dispatch), so foreign shapes get positional
 * correction too. This symbol remains as a NULL-world wrapper. */
bool a3_depenetration_dispatch(rigidbody *rigid_body_a, rigidbody *rigid_body_b,
                                      collision_data *collision_output) {
    return mpe_shape_dispatch(NULL, rigid_body_a, rigid_body_b, collision_output);
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

    int depenetration_iterations = rebuild_broadphase
        ? mpe_world_cfg(world)->depenetration.rebuild_iterations
        : 1; /* MPE_TASK_30 */

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

            if (mpe_shape_dispatch(world, body_a, body_b, &depenetration_collision)) {
                a3_positional_depenetrate_manifold_w(world, &depenetration_collision);
            }
        }

        for (int object_index = 0; object_index < body_count; object_index++) {
            rigidbody *rigid_body = &bodies[object_index];
            if (rigid_body->static_state || rigid_body->kinematic) {
                continue;
            }

            collision_data floor_collision = {0};

            if (collision_static_plane_body(&world->static_plane_body, rigid_body, 0.0f, &floor_collision, mpe_world_cfg(world))) {
                a3_positional_depenetrate_manifold_w(world, &floor_collision);
            }
        }
    }
}

void a3_positional_depenetrate_manifold(collision_data *manifold) {
    a3_positional_depenetrate_manifold_w(NULL, manifold);
}

void a3_positional_depenetrate_manifold_w(struct physics_world *world, collision_data *manifold) {
    const mpe_config_t *C = (world && world->cfg) ? world->cfg : &g_cfg;
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
    const float penetration_slop = C->solver.penetration_slop;

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
    if ((a_sleeping) && (b_sleeping) && (max_depth > C->depenetration.wake_depth_thresh)) {
        rigidbody_wake(body_a);
        rigidbody_wake(body_b);
        a_sleeping = false;
        b_sleeping = false;
    }

    if ((a_sleeping) && (body_b->static_state) && (max_depth > C->depenetration.wake_depth_thresh)) {
        rigidbody_wake(body_a);
        a_sleeping = false;
    }

    if ((b_sleeping) && (body_a->static_state) && (max_depth > C->depenetration.wake_depth_thresh)) {
        rigidbody_wake(body_b);
        b_sleeping = false;
    }

    /* TRUTH: a DYNAMIC body boring into a sleeper must wake it (deep
     * overlap), or depenetration shoves the dynamic aside while the sleeper
     * sleeps through growing penetration = ghost tunneling. Static and
     * both-sleeping cases above; these cover the asymmetric ones. */
    if ((a_sleeping) && (!b_sleeping) && (max_depth > C->depenetration.wake_depth_thresh)) {
        rigidbody_wake(body_a);
        a_sleeping = false;
    }

    if ((b_sleeping) && (!a_sleeping) && (max_depth > C->depenetration.wake_depth_thresh)) {
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

    /* FIX-AUDIT-DESPOT: the old early-return here skipped correction ENTIRELY
     * when any side slept below the wake threshold. That froze a real overlap
     * in place: the awake body plows into a just-below-threshold sleeper and
     * neither moves (tunneling by inches over seconds). Sleeping sides already
     * contribute effective inv mass 0 above, so the mass-weighted correction
     * below moves ONLY the awake side automatically (if both are effectively
     * locked, inv_sum<=0 returns next). No early return needed. */
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
        (max_depth - penetration_slop) * C->depenetration.correction_factor / inverse_mass_sum; /* MPE_TASK_30 */

    if (correction_magnitude <= 0.0f) {
        return;
    }
    if (correction_magnitude > C->depenetration.max_correction) {
        correction_magnitude = C->depenetration.max_correction;
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
