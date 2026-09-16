/* MPE_FTC_062: revolute (hinge) constraint solver.
 *
 * revolute_solve() enforces per solver iteration (velocity-level only):
 *   1. point-to-point: the two anchors coincide (3 DOF removed), solved
 *      with a 3x3 effective-mass impulse + Baumgarte positional bias.
 *   2. axis alignment: relative angular velocity perpendicular to the
 *      hinge axis is removed (2 DOF removed), leaving spin about the axis.
 * revolute_correct_axis_drift() applies the positional Baumgarte
 * correction that keeps hinge axes aligned (prevents wheel tilt under
 * load). It MUST run exactly once per tick after the iteration loop:
 * the error is positional, so per-iteration application multiplies the
 * correction by the iteration count and pumps energy into the joint.
 * revolute_apply_motor() drives relative spin about the axis toward a
 * target speed by adding torque to the torque accumulator (integrated once
 * per tick), clamped to a max torque. It is intentionally NOT inside the
 * iterative contact loop, so it cannot over-apply.
 *
 * Known simplifications (documented, deferred):
 *   - Jointed bodies are kept awake (FTC robots are always active).
 *   - Angle limits need persistent relative-angle tracking (deferred).
 *   - Single pass per tick; an accumulated-impulse iterative variant is a
 *     future stiffness upgrade.
 */
#include "revolute_joint.h"
#include "../config/mpe_config.h"
#include <math.h>
#include <stdio.h>

static math3 skew_symmetric(vector3 v) {
    math3 m = {{{0.0f}}};
    m.matrix[0][1] = -v.z;
    m.matrix[0][2] = v.y;
    m.matrix[1][0] = v.z;
    m.matrix[1][2] = -v.x;
    m.matrix[2][0] = -v.y;
    m.matrix[2][1] = v.x;
    return m;
}

static math3 math3_addition(math3 a, math3 b) {
    math3 r;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            r.matrix[i][j] = a.matrix[i][j] + b.matrix[i][j];
        }
    }
    return r;
}

void revolute_solve(revolute_params *p, rigidbody *body_a, rigidbody *body_b, float dt) {
    if ((!p) || (!body_a) || (!body_b) || (dt <= 0.0f)) {
        return;
    }
    /* Jointed bodies stay awake so the constraint always acts. */
    if (body_a->is_sleeping) {
        rigidbody_wake(body_a);
    }
    if (body_b->is_sleeping) {
        rigidbody_wake(body_b);
    }
    float inv_mass_a = rigidbody_effective_inv_mass(body_a);
    float inv_mass_b = rigidbody_effective_inv_mass(body_b);
    if ((inv_mass_a <= 0.0f) && (inv_mass_b <= 0.0f)) {
        return;
    }

    vector3 r_a = vector4_rotate_to_vector3(body_a->orientation, p->anchor_a);
    vector3 r_b = vector4_rotate_to_vector3(body_b->orientation, p->anchor_b);

    /* ---- point-to-point ---- */
    vector3 anchor_a_world = vector3_addition(body_a->position, r_a);
    vector3 anchor_b_world = vector3_addition(body_b->position, r_b);
    vector3 position_error = vector3_subtraction(anchor_b_world, anchor_a_world);

    vector3 vel_a_at_anchor = vector3_addition(body_a->velocity, vector3_cross(body_a->angular_velocity, r_a));
    vector3 vel_b_at_anchor = vector3_addition(body_b->velocity, vector3_cross(body_b->angular_velocity, r_b));
    vector3 relative_velocity = vector3_subtraction(vel_b_at_anchor, vel_a_at_anchor);

    /* Live tunable (was hardcoded 0.3): positional correction stiffness. */
    const float baumgarte_beta = g_cfg.joints.revolute_beta;
    /* Clamp the bias SPEED (Catto's stabilized Baumgarte): an uncapped
     * beta/dt turns a large anchor gap into a multi-m/s velocity demand in
     * one tick. Against a contact face the joint then re-injects approach
     * every iteration while the contact re-stops it — accumulated normal
     * impulse grows without bound, inflating Poisson restitution and the
     * friction cone (measured 7x: acc_n 43 from a 6 m/s impact). The cap
     * bounds per-tick energy injection; steady-state mm errors never bind. */
    float bias_speed = baumgarte_beta * vector3_length(position_error) / dt;
    float max_bias_speed = g_cfg.joints.revolute_max_bias;
    vector3 bias;
    if ((bias_speed > max_bias_speed) && (bias_speed > 0.0f)) {
        bias = vector3_scaling(position_error, (baumgarte_beta / dt) * (max_bias_speed / bias_speed));
    } else {
        bias = vector3_scaling(position_error, baumgarte_beta / dt);
    }

    float inv_mass_sum = inv_mass_a + inv_mass_b;
    math3 k = {{{0.0f}}};
    for (int i = 0; i < 3; i++) {
        k.matrix[i][i] = inv_mass_sum;
    }
    math3 skew_a = skew_symmetric(r_a);
    math3 skew_b = skew_symmetric(r_b);
    /* k = inv_mass_sum*I - skew(r_a)*Ia^-1*skew(r_a) - skew(r_b)*Ib^-1*skew(r_b)
     * (the subtracted terms are positive semi-definite, so k stays SPD) */
    math3 term_a = math3_multiplication(skew_a, math3_multiplication(rigidbody_effective_inv_inertia(body_a), skew_a));
    math3 term_b = math3_multiplication(skew_b, math3_multiplication(rigidbody_effective_inv_inertia(body_b), skew_b));
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            k.matrix[i][j] -= term_a.matrix[i][j];
            k.matrix[i][j] -= term_b.matrix[i][j];
        }
    }
    math3 k_inv = math3_inverse(k);
    vector3 rhs = vector3_scaling(vector3_addition(relative_velocity, bias), -1.0f);
    vector3 impulse = math3_multiplication_vector3(k_inv, rhs);

    body_a->velocity = vector3_subtraction(body_a->velocity, vector3_scaling(impulse, inv_mass_a));
    body_b->velocity = vector3_addition(body_b->velocity, vector3_scaling(impulse, inv_mass_b));
    body_a->angular_velocity =
        vector3_subtraction(body_a->angular_velocity,
                            math3_multiplication_vector3(rigidbody_effective_inv_inertia(body_a),
                                                         vector3_cross(r_a, impulse)));
    body_b->angular_velocity =
        vector3_addition(body_b->angular_velocity,
                         math3_multiplication_vector3(rigidbody_effective_inv_inertia(body_b),
                                                      vector3_cross(r_b, impulse)));

    /* ---- axis alignment: kill relative angular velocity off the hinge ---- */
    vector3 axis_world = vector4_rotate_to_vector3(body_a->orientation, vector3_normalisation(p->axis_a));
    vector3 relative_angular = vector3_subtraction(body_b->angular_velocity, body_a->angular_velocity);
    float along_axis = vector3_dot(relative_angular, axis_world);
    vector3 perpendicular_angular = vector3_subtraction(relative_angular, vector3_scaling(axis_world, along_axis));

    math3 angular_mass = math3_addition(rigidbody_effective_inv_inertia(body_a),
                                        rigidbody_effective_inv_inertia(body_b));
    math3 angular_mass_inv = math3_inverse(angular_mass);
    vector3 angular_impulse =
        vector3_scaling(math3_multiplication_vector3(angular_mass_inv, perpendicular_angular), -1.0f);
    body_a->angular_velocity = vector3_subtraction(
        body_a->angular_velocity,
        math3_multiplication_vector3(rigidbody_effective_inv_inertia(body_a), angular_impulse));
    body_b->angular_velocity = vector3_addition(
        body_b->angular_velocity,
        math3_multiplication_vector3(rigidbody_effective_inv_inertia(body_b), angular_impulse));
}

/* Positional axis-drift correction: MUST be called exactly once per tick,
 * AFTER the velocity iteration loop — never inside it. The error term is
 * positional (orientation difference, unchanged by velocity iterations),
 * so per-iteration application multiplies the correction by the iteration
 * count (64x at defaults): a spurious torsional spring that pumps energy
 * and destroys hinge truth (e.g. 9x-too-fast pendulum). */
void revolute_correct_axis_drift(revolute_params *p, rigidbody *body_a, rigidbody *body_b, float dt) {
    if ((!p) || (!body_a) || (!body_b) || (dt <= 0.0f)) {
        return;
    }
    vector3 hinge_b = (vector3_length_squared(p->axis_b) > 1e-12f) ? vector3_normalisation(p->axis_b)
                                                                  : vector3_normalisation(p->axis_a);
    /* ---- axis drift correction: positional Baumgarte to keep hinge axes aligned ---- */
    vector3 axis_a_world = vector4_rotate_to_vector3(body_a->orientation, vector3_normalisation(p->axis_a));
    vector3 axis_b_world = vector4_rotate_to_vector3(body_b->orientation, hinge_b);
    /* Axis error: cross product gives rotation vector needed to align axis_b with axis_a.
     * Magnitude is sin(angle) ≈ angle for small angles. Direction is the rotation axis. */
    vector3 axis_error = vector3_cross(axis_a_world, axis_b_world);
    float axis_error_len_sq = vector3_length_squared(axis_error);
    if (axis_error_len_sq > 0.000001f) {
        /* Baumgarte stabilization: apply angular velocity correction proportional to axis_error */
        const float axis_baumgarte_beta = 0.1f; /* MFS_127: reduced from 0.2 to reduce oscillation */
        vector3 axis_correction = vector3_scaling(axis_error, axis_baumgarte_beta / dt);
        /* Compute effective angular mass for the correction */
        math3 drift_angular_mass = math3_addition(rigidbody_effective_inv_inertia(body_a),
                                                  rigidbody_effective_inv_inertia(body_b));
        math3 drift_angular_mass_inv = math3_inverse(drift_angular_mass);
        vector3 axis_impulse =
            vector3_scaling(math3_multiplication_vector3(drift_angular_mass_inv, axis_correction), -1.0f);
        /* Apply angular impulse to both bodies */
        if (!body_a->static_state) {
            body_a->angular_velocity = vector3_subtraction(
                body_a->angular_velocity,
                math3_multiplication_vector3(rigidbody_effective_inv_inertia(body_a), axis_impulse));
        }
        if (!body_b->static_state) {
            body_b->angular_velocity = vector3_addition(
                body_b->angular_velocity,
                math3_multiplication_vector3(rigidbody_effective_inv_inertia(body_b), axis_impulse));
        }
    }
}

void revolute_apply_motor(revolute_params *p, rigidbody *body_a, rigidbody *body_b, float dt) {
    (void) dt;
    if ((!p) || (!p->motor_enabled) || (!body_a) || (!body_b)) {
        return;
    }
    /* TRUTH gap disclosed: angle limits (limits_enabled/min/max) are STORED
     * but NOT enforced by the velocity solve (needs persistent relative-angle
     * tracking, deferred). A limit-enabled hinge currently behaves as a free
     * hinge. This warns loudly once instead of failing silently. */
    if (p->limits_enabled) {
        static int limit_warn_count = 0;
        if (limit_warn_count < 4) {
            fprintf(stderr,
                    "[revolute] TRUTH gap: angle limits enabled but unenforced "
                    "(free hinge). Persistent angle tracking deferred.\n");
            limit_warn_count++;
        }
    }
    vector3 axis_world = vector4_rotate_to_vector3(body_a->orientation, vector3_normalisation(p->axis_a));
    vector3 relative_angular = vector3_subtraction(body_b->angular_velocity, body_a->angular_velocity);
    float current_speed = vector3_dot(relative_angular, axis_world);
    float speed_error = p->motor_target_speed - current_speed;
    float motor_gain = g_cfg.joints.revolute_motor_gain;
    float desired_torque = speed_error * motor_gain;
    if (desired_torque > p->motor_max_torque) {
        desired_torque = p->motor_max_torque;
    }
    if (desired_torque < -p->motor_max_torque) {
        desired_torque = -p->motor_max_torque;
    }
    vector3 drive_torque = vector3_scaling(axis_world, desired_torque);
    body_a->torque_accumulator = vector3_subtraction(body_a->torque_accumulator, drive_torque);
    body_b->torque_accumulator = vector3_addition(body_b->torque_accumulator, drive_torque);
}

/* Fixed weld: point-to-point (same K-matrix as revolute) plus full angular
 * lock (kill all relative spin, not just off-axis). Deterministic, no bias
 * beyond the shared Baumgarte cap. */
void fixed_solve(fixed_params *p, rigidbody *body_a, rigidbody *body_b, float dt) {
    if ((!p) || (!body_a) || (!body_b) || (dt <= 0.0f)) {
        return;
    }
    if (body_a->is_sleeping) {
        rigidbody_wake(body_a);
    }
    if (body_b->is_sleeping) {
        rigidbody_wake(body_b);
    }
    float inv_a = rigidbody_effective_inv_mass(body_a);
    float inv_b = rigidbody_effective_inv_mass(body_b);
    if ((inv_a <= 0.0f) && (inv_b <= 0.0f)) {
        return;
    }
    vector3 r_a = vector4_rotate_to_vector3(body_a->orientation, p->anchor_a);
    vector3 r_b = vector4_rotate_to_vector3(body_b->orientation, p->anchor_b);
    vector3 world_a = vector3_addition(body_a->position, r_a);
    vector3 world_b = vector3_addition(body_b->position, r_b);
    vector3 err = vector3_subtraction(world_b, world_a);
    vector3 vel_a = vector3_addition(body_a->velocity, vector3_cross(body_a->angular_velocity, r_a));
    vector3 vel_b = vector3_addition(body_b->velocity, vector3_cross(body_b->angular_velocity, r_b));
    vector3 rel = vector3_subtraction(vel_b, vel_a);
    const float beta = g_cfg.joints.revolute_beta;
    float err_len = vector3_length(err);
    vector3 bias = (err_len > 1e-9f)
        ? vector3_scaling(err, fminf(beta / dt, g_cfg.joints.revolute_max_bias / fmaxf(err_len, 1e-9f)))
        : vector3_zero();
    math3 skew_a = skew_symmetric(r_a);
    math3 skew_b = skew_symmetric(r_b);
    math3 k = {{{0.0f}}};
    float inv_sum = inv_a + inv_b;
    for (int i = 0; i < 3; i++) {
        k.matrix[i][i] = inv_sum;
    }
    math3 term_a = math3_multiplication(skew_a, math3_multiplication(rigidbody_effective_inv_inertia(body_a), skew_a));
    math3 term_b = math3_multiplication(skew_b, math3_multiplication(rigidbody_effective_inv_inertia(body_b), skew_b));
    /* k = inv_sum*I - term_a - term_b */
    for (int c = 0; c < 3; c++) {
        for (int r = 0; r < 3; r++) {
            k.matrix[c][r] -= term_a.matrix[c][r] + term_b.matrix[c][r];
        }
    }
    math3 k_inv = math3_inverse(k);
    vector3 rhs = vector3_scaling(vector3_addition(rel, bias), -1.0f);
    vector3 impulse = math3_multiplication_vector3(k_inv, rhs);
    body_a->velocity = vector3_subtraction(body_a->velocity, vector3_scaling(impulse, inv_a));
    body_b->velocity = vector3_addition(body_b->velocity, vector3_scaling(impulse, inv_b));
    body_a->angular_velocity = vector3_subtraction(
        body_a->angular_velocity,
        math3_multiplication_vector3(rigidbody_effective_inv_inertia(body_a), vector3_cross(r_a, impulse)));
    body_b->angular_velocity = vector3_addition(
        body_b->angular_velocity,
        math3_multiplication_vector3(rigidbody_effective_inv_inertia(body_b), vector3_cross(r_b, impulse)));
    /* Full angular lock: remove all relative spin. */
    vector3 rel_w = vector3_subtraction(body_b->angular_velocity, body_a->angular_velocity);
    math3 m = math3_addition(rigidbody_effective_inv_inertia(body_a), rigidbody_effective_inv_inertia(body_b));
    math3 m_inv = math3_inverse(m);
    vector3 ang_imp = vector3_scaling(math3_multiplication_vector3(m_inv, rel_w), -1.0f);
    body_a->angular_velocity = vector3_subtraction(
        body_a->angular_velocity, math3_multiplication_vector3(rigidbody_effective_inv_inertia(body_a), ang_imp));
    body_b->angular_velocity = vector3_addition(
        body_b->angular_velocity, math3_multiplication_vector3(rigidbody_effective_inv_inertia(body_b), ang_imp));
}

/* Distance: 1D constraint along the anchor axis. Preserves free rotation and
 * tangential motion; only the separation error is corrected. */
void distance_solve(distance_params *p, rigidbody *body_a, rigidbody *body_b, float dt) {
    if ((!p) || (!body_a) || (!body_b) || (dt <= 0.0f)) {
        return;
    }
    if (!isfinite(p->rest_length) || p->rest_length < 0.0f) {
        return;
    }
    if (body_a->is_sleeping) {
        rigidbody_wake(body_a);
    }
    if (body_b->is_sleeping) {
        rigidbody_wake(body_b);
    }
    float inv_a = rigidbody_effective_inv_mass(body_a);
    float inv_b = rigidbody_effective_inv_mass(body_b);
    vector3 r_a = vector4_rotate_to_vector3(body_a->orientation, p->anchor_a);
    vector3 r_b = vector4_rotate_to_vector3(body_b->orientation, p->anchor_b);
    vector3 world_a = vector3_addition(body_a->position, r_a);
    vector3 world_b = vector3_addition(body_b->position, r_b);
    vector3 delta = vector3_subtraction(world_b, world_a);
    float dist = vector3_length(delta);
    if (dist < 1e-9f) {
        return;
    }
    vector3 n = vector3_scaling(delta, 1.0f / dist);
    float err = dist - p->rest_length;
    vector3 vel_a = vector3_addition(body_a->velocity, vector3_cross(body_a->angular_velocity, r_a));
    vector3 vel_b = vector3_addition(body_b->velocity, vector3_cross(body_b->angular_velocity, r_b));
    float rel_n = vector3_dot(vector3_subtraction(vel_b, vel_a), n);
    float bias = g_cfg.joints.revolute_beta * err / dt;
    float max_b = g_cfg.joints.revolute_max_bias;
    if (bias > max_b) {
        bias = max_b;
    } else if (bias < -max_b) {
        bias = -max_b;
    }
    vector3 ra_n = vector3_cross(r_a, n);
    vector3 rb_n = vector3_cross(r_b, n);
    float k = inv_a + inv_b +
        vector3_dot(ra_n, math3_multiplication_vector3(rigidbody_effective_inv_inertia(body_a), ra_n)) +
        vector3_dot(rb_n, math3_multiplication_vector3(rigidbody_effective_inv_inertia(body_b), rb_n));
    if (k <= 1e-12f) {
        return;
    }
    float lambda = -(rel_n + bias) / k;
    vector3 impulse = vector3_scaling(n, lambda);
    body_a->velocity = vector3_subtraction(body_a->velocity, vector3_scaling(impulse, inv_a));
    body_b->velocity = vector3_addition(body_b->velocity, vector3_scaling(impulse, inv_b));
    body_a->angular_velocity = vector3_subtraction(
        body_a->angular_velocity,
        math3_multiplication_vector3(rigidbody_effective_inv_inertia(body_a), vector3_cross(r_a, impulse)));
    body_b->angular_velocity = vector3_addition(
        body_b->angular_velocity,
        math3_multiplication_vector3(rigidbody_effective_inv_inertia(body_b), vector3_cross(r_b, impulse)));
}

/* TRUTH: fixed-weld angular positional correction (once per tick, AFTER the
 * velocity loop — never inside, same energy-pump rule as revolute axis
 * drift). Velocity-only lock kills relative spin but lets orientation error
 * integrate (weld flexes over seconds). This Baumgarte drives the relative
 * quaternion error q_err = q_a^-1 * q_b toward identity with angular impulse,
 * proportional to the rotation vector (axis*sin(half-angle) scaled). */
void fixed_correct_angular_drift(fixed_params *p, rigidbody *body_a, rigidbody *body_b, float dt) {
    (void) p;
    if ((!body_a) || (!body_b) || (dt <= 0.0f)) {
        return;
    }
    vector4 q_a_inv =
        (vector4){body_a->orientation.w, -body_a->orientation.x, -body_a->orientation.y, -body_a->orientation.z};
    vector4 q_err = vector4_multiplication(q_a_inv, body_b->orientation);
    q_err = vector4_normalisation(q_err);
    /* Shortest-arc: w<0 flips to the complementary rotation. */
    if (q_err.w < 0.0f) {
        q_err.w = -q_err.w;
        q_err.x = -q_err.x;
        q_err.y = -q_err.y;
        q_err.z = -q_err.z;
    }
    vector3 err_vec = {q_err.x, q_err.y, q_err.z};
    if (vector3_length_squared(err_vec) < 1e-12f) {
        return;
    }
    const float beta = 0.1f;
    /* Correction angular velocity in A-frame, rotated to world via A. */
    vector3 corr_local = vector3_scaling(err_vec, 2.0f * beta / dt);
    vector3 corr_world = vector4_rotate_to_vector3(body_a->orientation, corr_local);
    math3 m = math3_addition(rigidbody_effective_inv_inertia(body_a),
                             rigidbody_effective_inv_inertia(body_b));
    /* Guard near-singular (both infinite mass): nothing to correct. */
    math3 m_inv = math3_inverse(m);
    vector3 impulse = vector3_scaling(math3_multiplication_vector3(m_inv, corr_world), -1.0f);
    if (!body_a->static_state) {
        body_a->angular_velocity = vector3_subtraction(
            body_a->angular_velocity,
            math3_multiplication_vector3(rigidbody_effective_inv_inertia(body_a), impulse));
    }
    if (!body_b->static_state) {
        body_b->angular_velocity = vector3_addition(
            body_b->angular_velocity,
            math3_multiplication_vector3(rigidbody_effective_inv_inertia(body_b), impulse));
    }
}