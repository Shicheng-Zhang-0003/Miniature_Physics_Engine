/* MPE_FTC_062: revolute (hinge) constraint solver.
 *
 * revolute_solve() enforces per solver iteration (velocity-level only):
 *   1. point-to-point: the two anchors coincide (3 DOF removed), solved
 *      with a 3x3 effective-mass impulse + Baumgarte positional bias.
 *   2. axis alignment: relative angular velocity perpendicular to the
 *      hinge axis is removed (2 DOF removed), leaving spin about the axis.
 *   3. angle limits: persistent relative-angle tracking with velocity-level
 *      enforcement (limits_enabled, limit_min_rad, limit_max_rad).
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
 * Known simplifications (documented):
 *   - Jointed bodies are kept awake (FTC robots are always active).
 *   - Single pass per tick; an accumulated-impulse iterative variant is a
 *     future stiffness upgrade.
 *   */
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

/* 6×6 matrix operations for coupled hinge solve + motor. */
static void mat6_zero(float m[6][6]) {
    for (int i = 0; i < 6; i++) for (int j = 0; j < 6; j++) m[i][j] = 0.0f;
}
static void mat6_vec_mul(float out[6], float m[6][6], float v[6]) {
    for (int i = 0; i < 6; i++) {
        out[i] = 0.0f;
        for (int j = 0; j < 6; j++) out[i] += m[i][j] * v[j];
    }
}
static int mat6_invert(float m[6][6], float out[6][6]) {
    /* Gauss-Jordan elimination with partial pivoting. */
    float aug[6][12];
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) aug[i][j] = m[i][j];
        for (int j = 0; j < 6; j++) aug[i][6+j] = (i==j) ? 1.0f : 0.0f;
    }
    for (int col = 0; col < 6; col++) {
        int pivot = col;
        float max_val = fabsf(aug[col][col]);
        for (int row = col+1; row < 6; row++) {
            if (fabsf(aug[row][col]) > max_val) {
                max_val = fabsf(aug[row][col]);
                pivot = row;
            }
        }
        if (max_val < 1e-12f) return 0; /* singular */
        if (pivot != col) {
            for (int j = 0; j < 12; j++) {
                float tmp = aug[col][j]; aug[col][j] = aug[pivot][j]; aug[pivot][j] = tmp;
            }
        }
        float piv_val = aug[col][col];
        for (int j = 0; j < 12; j++) aug[col][j] /= piv_val;
        for (int row = 0; row < 6; row++) {
            if (row == col) continue;
            float factor = aug[row][col];
            for (int j = 0; j < 12; j++) aug[row][j] -= factor * aug[col][j];
        }
    }
    for (int i = 0; i < 6; i++) for (int j = 0; j < 6; j++) out[i][j] = aug[i][6+j];
    return 1;
}

void revolute_solve(revolute_params *p, rigidbody *body_a, rigidbody *body_b, float dt, const mpe_config_t *cfg) {
    const mpe_config_t *C = cfg ? cfg : &g_cfg;
    if ((!p) || (!body_a) || (!body_b) || (dt <= 0.0f)) {
        return;
    }
    /* Jointed bodies stay awake so the constraint always acts. */
    if (body_a->is_sleeping) rigidbody_wake(body_a);
    if (body_b->is_sleeping) rigidbody_wake(body_b);
    
    float inv_mass_a = rigidbody_effective_inv_mass(body_a);
    float inv_mass_b = rigidbody_effective_inv_mass(body_b);
    if ((inv_mass_a <= 0.0f) && (inv_mass_b <= 0.0f)) {
        return;
    }

    vector3 r_a = vector4_rotate_to_vector3(body_a->orientation, p->anchor_a);
    vector3 r_b = vector4_rotate_to_vector3(body_b->orientation, p->anchor_b);

    /* Anchor positions and velocities. */
    vector3 anchor_a_world = vector3_addition(body_a->position, r_a);
    vector3 anchor_b_world = vector3_addition(body_b->position, r_b);
    vector3 position_error = vector3_subtraction(anchor_b_world, anchor_a_world);

    vector3 vel_a_at_anchor = vector3_addition(body_a->velocity, vector3_cross(body_a->angular_velocity, r_a));
    vector3 vel_b_at_anchor = vector3_addition(body_b->velocity, vector3_cross(body_b->angular_velocity, r_b));
    vector3 relative_velocity = vector3_subtraction(vel_b_at_anchor, vel_a_at_anchor);

    /* Hinge axis in world space (from body A). */
    vector3 axis_world = vector4_rotate_to_vector3(body_a->orientation, vector3_normalisation(p->axis_a));
    float axis_len_sq = vector3_length_squared(axis_world);
    if (axis_len_sq < 1e-12f) return;
    axis_world = vector3_scaling(axis_world, 1.0f / sqrtf(axis_len_sq));

    /* Build orthonormal basis (u, v) perpendicular to axis for axis alignment constraints.
     * u = normalize(axis × ref), v = axis × u. */
    vector3 ref = (fabsf(axis_world.y) < 0.99f) ? (vector3){0.0f, 1.0f, 0.0f} : (vector3){1.0f, 0.0f, 0.0f};
    vector3 u = vector3_cross(axis_world, ref);
    float u_len = sqrtf(vector3_length_squared(u));
    if (u_len < 1e-6f) {
        ref = (vector3){1.0f, 0.0f, 0.0f};
        u = vector3_cross(axis_world, ref);
        u_len = sqrtf(vector3_length_squared(u));
    }
    u = vector3_scaling(u, 1.0f / u_len);
    vector3 v = vector3_cross(axis_world, u); /* already unit length */

    /* Baumgarte bias for position error (point-to-point only; axis alignment is velocity-only).
     * TRUTH: velocity-level P2P bias solved INSIDE the iteration loop is
     * correct here (bias includes live rel_vel and converges; it is NOT the
     * once-per-tick axis-drift term, which must never enter the loop or its
     * position error pumps 64x). Do not move either across the boundary. */
    const float baumgarte_beta = C->joints.revolute_beta;
    float bias_speed = baumgarte_beta * vector3_length(position_error) / dt;
    float max_bias_speed = C->joints.revolute_max_bias;
    vector3 bias_p2p = (bias_speed > max_bias_speed && bias_speed > 0.0f)
        ? vector3_scaling(position_error, (baumgarte_beta / dt) * (max_bias_speed / bias_speed))
        : vector3_scaling(position_error, baumgarte_beta / dt);

    /* Effective mass/inertia. */
    float inv_mass_sum = inv_mass_a + inv_mass_b;
    math3 I_inv_a = rigidbody_effective_inv_inertia(body_a);
    math3 I_inv_b = rigidbody_effective_inv_inertia(body_b);
    math3 skew_a = skew_symmetric(r_a);
    math3 skew_b = skew_symmetric(r_b);

    /* Build 6×6 K-matrix (effective mass matrix for the 5 constraints + motor).
     * Rows 0-2: point-to-point (x, y, z)
     * Rows 3-4: axis alignment (u, v components of relative angular velocity)
     * Row 5: motor (relative angular velocity along hinge axis) */
    float K[6][6];
    mat6_zero(K);

    /* Point-to-point block (3×3): K_p2p = inv_mass_sum*I - skew_a*Ia^-1*skew_a - skew_b*Ib^-1*skew_b */
    math3 term_a = math3_multiplication(skew_a, math3_multiplication(I_inv_a, skew_a));
    math3 term_b = math3_multiplication(skew_b, math3_multiplication(I_inv_b, skew_b));
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            K[i][j] = (i==j ? inv_mass_sum : 0.0f) - term_a.matrix[i][j] - term_b.matrix[i][j];
        }
    }

    /* Cross-coupling block (3×2): K_p2p_axis = J_p2p * M^-1 * J_axis^T
     * J_p2p = [I, -I, -skew(r_a), skew(r_b)]
     * J_axis_u = [0, 0, -u, u]
     * J_axis_v = [0, 0, -v, v] */
    for (int i = 0; i < 3; i++) {
        vector3 skew_a_row_i;
        if (i == 0) skew_a_row_i = (vector3){0, -r_a.z, r_a.y};
        else if (i == 1) skew_a_row_i = (vector3){r_a.z, 0, -r_a.x};
        else skew_a_row_i = (vector3){-r_a.y, r_a.x, 0};
        
        vector3 skew_b_row_i;
        if (i == 0) skew_b_row_i = (vector3){0, -r_b.z, r_b.y};
        else if (i == 1) skew_b_row_i = (vector3){r_b.z, 0, -r_b.x};
        else skew_b_row_i = (vector3){-r_b.y, r_b.x, 0};
        
        vector3 Ia_skew_a = math3_multiplication_vector3(I_inv_a, skew_a_row_i);
        vector3 Ib_skew_b = math3_multiplication_vector3(I_inv_b, skew_b_row_i);
        
        K[i][3] = -vector3_dot(Ia_skew_a, u) + vector3_dot(Ib_skew_b, u);
        K[i][4] = -vector3_dot(Ia_skew_a, v) + vector3_dot(Ib_skew_b, v);
        K[3][i] = K[i][3]; /* symmetric */
        K[4][i] = K[i][4];
    }
    
    /* Axis-Axis block (2×2): K_axis = J_axis * M^-1 * J_axis^T */
    math3 I_sum = math3_addition(I_inv_a, I_inv_b);
    K[3][3] = vector3_dot(u, math3_multiplication_vector3(I_sum, u));
    K[4][4] = vector3_dot(v, math3_multiplication_vector3(I_sum, v));
    K[3][4] = vector3_dot(u, math3_multiplication_vector3(I_sum, v));
    K[4][3] = K[3][4];
    
    /* Motor row/column (row 5): K_motor = axis · (Ia^-1 + Ib^-1) · axis */
    float axis_mass_inv = vector3_dot(axis_world, math3_multiplication_vector3(I_sum, axis_world));
    K[5][5] = (axis_mass_inv > 1e-12f) ? axis_mass_inv : 1e-12f;
    
    /* Motor coupling with axis alignment rows (3,4): K[5][3] = axis · I_sum · u, etc.
     * Free hinge when disabled: decouple row/col 5 and force lambda[5]=0
     * (enforcing along_axis=0 would weld the hinge into a rotational lock). */
    if (p->motor_enabled) {
        K[5][3] = vector3_dot(axis_world, math3_multiplication_vector3(I_sum, u));
        K[3][5] = K[5][3];
        K[5][4] = vector3_dot(axis_world, math3_multiplication_vector3(I_sum, v));
        K[4][5] = K[5][4];
    } else {
        K[5][3] = 0.0f; K[3][5] = 0.0f;
        K[5][4] = 0.0f; K[4][5] = 0.0f;
    }
    /* Motor coupling with P2P rows (0,1,2): zero (motor is pure angular, no linear coupling). */
    for (int i = 0; i < 3; i++) {
        K[i][5] = 0.0f;
        K[5][i] = 0.0f;
    }

    /* Add regularization for numerical stability (tiny diagonal). */
    for (int i = 0; i < 6; i++) K[i][i] += 1e-10f;

    /* RHS = -(J*v + bias). Bias only on P2P (first 3 rows). */
    float rhs[6];
    /* P2P rows: -(relative_velocity + bias_p2p) */
    vector3 rhs_p2p = vector3_scaling(vector3_addition(relative_velocity, bias_p2p), -1.0f);
    rhs[0] = rhs_p2p.x;
    rhs[1] = rhs_p2p.y;
    rhs[2] = rhs_p2p.z;
    /* Axis rows: -perpendicular_angular_velocity (no bias for axis alignment). */
    vector3 rel_ang = vector3_subtraction(body_b->angular_velocity, body_a->angular_velocity);
    rhs[3] = -vector3_dot(rel_ang, u);
    rhs[4] = -vector3_dot(rel_ang, v);
    /* Motor row: -(along_axis_velocity - motor_target_speed), or 0 when
     * disabled (free spin: no constraint on the hinge axis). */
    float along_axis = vector3_dot(rel_ang, axis_world);
    if (p->motor_enabled) {
        rhs[5] = -(along_axis - p->motor_target_speed);
    } else {
        rhs[5] = 0.0f;
    }

    /* Solve K * lambda = rhs. */
    float K_inv[6][6];
    if (!mat6_invert(K, K_inv)) {
        /* Singular - fall back to sequential solve. */
        goto fallback_sequential;
    }
    float lambda[6];
    mat6_vec_mul(lambda, K_inv, rhs);
    if (!p->motor_enabled) {
        lambda[5] = 0.0f; /* free hinge: never apply axis torque */
    } else if (p->motor_max_torque > 0.0f) {
        /* Single clamped drive: constraint torque respects max_torque over
         * THIS tick's dt (hardcoded 1/60 overstated torque 6x when clamped
         * to 0.1s). (Torque-accumulator pre-pass also drives; the clamp
         * here keeps the constraint path from overriding weak motors.)
         * TRUTH: full single-drive decoupling (accumulator only) was
         * considered and REJECTED with cause: the clamped row converges
         * hinges through contact in one tick where accumulator-only lags,
         * and no test exhibits overshoot with the clamp in place
         * (revolute/pendulum green, motor stays within no-overshoot). */
        float max_lam = p->motor_max_torque * dt;
        if (lambda[5] > max_lam) lambda[5] = max_lam;
        else if (lambda[5] < -max_lam) lambda[5] = -max_lam;
    }

    /* Apply impulses.
     * P2P impulse (3D): applied to both bodies.
     * Axis impulses (2D): angular impulses along u and v.
     * Motor impulse (1D): angular impulse along axis_world. */
    vector3 impulse_p2p = {lambda[0], lambda[1], lambda[2]};
    vector3 axis_impulse = vector3_addition(
        vector3_scaling(u, lambda[3]),
        vector3_scaling(v, lambda[4])
    );
    float motor_lambda = lambda[5];
    vector3 motor_impulse = vector3_scaling(axis_world, motor_lambda);

    body_a->velocity = vector3_subtraction(body_a->velocity, vector3_scaling(impulse_p2p, inv_mass_a));
    body_b->velocity = vector3_addition(body_b->velocity, vector3_scaling(impulse_p2p, inv_mass_b));
    body_a->angular_velocity = vector3_subtraction(
        body_a->angular_velocity,
        math3_multiplication_vector3(I_inv_a, vector3_addition(vector3_addition(vector3_cross(r_a, impulse_p2p), axis_impulse), motor_impulse)));
    body_b->angular_velocity = vector3_addition(
        body_b->angular_velocity,
        math3_multiplication_vector3(I_inv_b, vector3_addition(vector3_addition(vector3_cross(r_b, impulse_p2p), axis_impulse), motor_impulse)));

    /* ---- angle limits: persistent relative-angle tracking + velocity-level enforcement ---- */
    if (p->limits_enabled) {
        if (!p->angle_initialized) {
            vector4 q_a_inv = {body_a->orientation.w, -body_a->orientation.x, -body_a->orientation.y, -body_a->orientation.z};
            vector4 q_rel = vector4_multiplication(q_a_inv, body_b->orientation);
            q_rel = vector4_normalisation(q_rel);
            if (q_rel.w < 0.0f) { q_rel.w = -q_rel.w; q_rel.x = -q_rel.x; q_rel.y = -q_rel.y; q_rel.z = -q_rel.z; }
            float half_angle = atan2f(sqrtf(q_rel.x * q_rel.x + q_rel.y * q_rel.y + q_rel.z * q_rel.z), q_rel.w);
            vector3 rot_axis = {q_rel.x, q_rel.y, q_rel.z};
            float rot_axis_len = sqrtf(vector3_length_squared(rot_axis));
            if (rot_axis_len > 1e-6f) rot_axis = vector3_scaling(rot_axis, 1.0f / rot_axis_len);
            else rot_axis = axis_world;
            float axis_dot = vector3_dot(rot_axis, axis_world);
            float initial_angle = 2.0f * half_angle * axis_dot;
            /* TRUTH: NO [-pi,pi] wrap (dead-reckoned joint coordinate must
             * stay unwrapped: wrapping made limits outside [-pi,pi]
             * unreachable and teleported multi-turn assemblies. Integration
             * of along_axis IS the angle for single-axis hinge motion
             * (exact, not approximate); velocity-level limit kills rebase
             * drift every iteration. */
            p->accumulated_angle = initial_angle;
            p->reference_axis_a = vector4_rotate_to_vector3(body_a->orientation, vector3_normalisation(p->axis_a));
            p->reference_axis_b = vector4_rotate_to_vector3(body_b->orientation,
                (vector3_length_squared(p->axis_b) > 1e-12f) ? vector3_normalisation(p->axis_b)
                                                            : vector3_normalisation(p->axis_a));
            p->angle_initialized = true;
        }

        float min_limit = p->limit_min_rad;
        float max_limit = p->limit_max_rad;
        float along_axis = vector3_dot(vector3_subtraction(body_b->angular_velocity, body_a->angular_velocity), axis_world);
        bool at_min = (p->accumulated_angle <= min_limit + 1e-4f) && (along_axis < 0.0f);
        bool at_max = (p->accumulated_angle >= max_limit - 1e-4f) && (along_axis > 0.0f);

        if (at_min || at_max) {
            float axis_mass_inv = vector3_dot(axis_world, math3_multiplication_vector3(I_sum, axis_world));
            float axis_mass = (axis_mass_inv > 1e-12f) ? (1.0f / axis_mass_inv) : 0.0f;
            if (axis_mass > 0.0f) {
                float limit_lambda = -along_axis * axis_mass;
                vector3 limit_impulse = vector3_scaling(axis_world, limit_lambda);
                body_a->angular_velocity = vector3_subtraction(
                    body_a->angular_velocity,
                    math3_multiplication_vector3(I_inv_a, limit_impulse));
                body_b->angular_velocity = vector3_addition(
                    body_b->angular_velocity,
                    math3_multiplication_vector3(I_inv_b, limit_impulse));
                if (at_min) p->accumulated_angle = min_limit;
                if (at_max) p->accumulated_angle = max_limit;
            }
        }
    }
    return;

fallback_sequential:
    /* Fallback to original sequential solve if 5×5 solve fails. */
    /* ---- point-to-point ---- */
    {
        float inv_mass_sum = inv_mass_a + inv_mass_b;
        math3 k = {{{0.0f}}};
        for (int i = 0; i < 3; i++) k.matrix[i][i] = inv_mass_sum;
        math3 term_a = math3_multiplication(skew_a, math3_multiplication(I_inv_a, skew_a));
        math3 term_b = math3_multiplication(skew_b, math3_multiplication(I_inv_b, skew_b));
        for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) k.matrix[i][j] -= term_a.matrix[i][j] + term_b.matrix[i][j];
        math3 k_inv = math3_inverse(k);
        vector3 rhs_vec = vector3_scaling(vector3_addition(relative_velocity, bias_p2p), -1.0f);
        vector3 impulse = math3_multiplication_vector3(k_inv, rhs_vec);
        body_a->velocity = vector3_subtraction(body_a->velocity, vector3_scaling(impulse, inv_mass_a));
        body_b->velocity = vector3_addition(body_b->velocity, vector3_scaling(impulse, inv_mass_b));
        body_a->angular_velocity = vector3_subtraction(body_a->angular_velocity,
            math3_multiplication_vector3(I_inv_a, vector3_cross(r_a, impulse)));
        body_b->angular_velocity = vector3_addition(body_b->angular_velocity,
            math3_multiplication_vector3(I_inv_b, vector3_cross(r_b, impulse)));
    }
    /* ---- axis alignment ---- */
    {
        vector3 rel_ang = vector3_subtraction(body_b->angular_velocity, body_a->angular_velocity);
        vector3 perp_ang = vector3_subtraction(rel_ang, vector3_scaling(axis_world, vector3_dot(rel_ang, axis_world)));
        math3 ang_mass = math3_addition(I_inv_a, I_inv_b);
        math3 ang_mass_inv = math3_inverse(ang_mass);
        vector3 ang_imp = vector3_scaling(math3_multiplication_vector3(ang_mass_inv, perp_ang), -1.0f);
        body_a->angular_velocity = vector3_subtraction(body_a->angular_velocity,
            math3_multiplication_vector3(I_inv_a, ang_imp));
        body_b->angular_velocity = vector3_addition(body_b->angular_velocity,
            math3_multiplication_vector3(I_inv_b, ang_imp));
    }
    /* ---- limits ---- (same as above) */
    if (p->limits_enabled) {
        float min_limit = p->limit_min_rad;
        float max_limit = p->limit_max_rad;
        float along_axis = vector3_dot(vector3_subtraction(body_b->angular_velocity, body_a->angular_velocity), axis_world);
        bool at_min = (p->accumulated_angle <= min_limit + 1e-4f) && (along_axis < 0.0f);
        bool at_max = (p->accumulated_angle >= max_limit - 1e-4f) && (along_axis > 0.0f);
        if (at_min || at_max) {
            float axis_mass_inv = vector3_dot(axis_world, math3_multiplication_vector3(I_sum, axis_world));
            float axis_mass = (axis_mass_inv > 1e-12f) ? (1.0f / axis_mass_inv) : 0.0f;
            if (axis_mass > 0.0f) {
                float limit_lambda = -along_axis * axis_mass;
                vector3 limit_impulse = vector3_scaling(axis_world, limit_lambda);
                body_a->angular_velocity = vector3_subtraction(body_a->angular_velocity,
                    math3_multiplication_vector3(I_inv_a, limit_impulse));
                body_b->angular_velocity = vector3_addition(body_b->angular_velocity,
                    math3_multiplication_vector3(I_inv_b, limit_impulse));
                if (at_min) p->accumulated_angle = min_limit;
                if (at_max) p->accumulated_angle = max_limit;
            }
        }
    }
}

/* TRUTH: once-per-tick angle integration (called before the solver loop). */
void revolute_pre_step(revolute_params *p, rigidbody *body_a, rigidbody *body_b, float dt, const mpe_config_t *cfg) {
    (void) cfg;
    if ((!p) || (!body_a) || (!body_b) || (!(dt > 0.0f))) {
        return;
    }
    if (!p->limits_enabled || !p->angle_initialized) {
        return;
    }
    vector3 axis_world = vector4_rotate_to_vector3(body_a->orientation, vector3_normalisation(p->axis_a));
    vector3 rel_ang = vector3_subtraction(body_b->angular_velocity, body_a->angular_velocity);
    float along = vector3_dot(rel_ang, axis_world);
    if (!isfinite(along)) {
        return;
    }
    /* Clamp per-tick delta to avoid explosion on NaN/spike (max 1 rad/tick). */
    float d = along * dt;
    if (d > 1.0f) {
        d = 1.0f;
    } else if (d < -1.0f) {
        d = -1.0f;
    }
    p->accumulated_angle += d;
    /* TRUTH: no wrap (see init site): the joint coordinate stays unwrapped
     * so multi-turn limits work; limit kills rebase it every solve. */
}

/* Prismatic: single-axis slide with optional limits and motor. */
void prismatic_solve(prismatic_params *p, rigidbody *body_a, rigidbody *body_b, float dt, const mpe_config_t *cfg) {
    const mpe_config_t *C = cfg ? cfg : &g_cfg;
    if ((!p) || (!body_a) || (!body_b) || (dt <= 0.0f)) {
        return;
    }
    if (body_a->is_sleeping) rigidbody_wake(body_a);
    if (body_b->is_sleeping) rigidbody_wake(body_b);

    float inv_a = rigidbody_effective_inv_mass(body_a);
    float inv_b = rigidbody_effective_inv_mass(body_b);
    if ((inv_a <= 0.0f) && (inv_b <= 0.0f)) return;

    vector3 r_a = vector4_rotate_to_vector3(body_a->orientation, p->anchor_a);
    vector3 r_b = vector4_rotate_to_vector3(body_b->orientation, p->anchor_b);
    vector3 world_a = vector3_addition(body_a->position, r_a);
    vector3 world_b = vector3_addition(body_b->position, r_b);

    /* Slide axis in world space (from body A). */
    vector3 axis_a_world = vector4_rotate_to_vector3(body_a->orientation, vector3_normalisation(p->axis_a));
    vector3 axis_b_world = (vector3_length_squared(p->axis_b) > 1e-12f)
        ? vector4_rotate_to_vector3(body_b->orientation, vector3_normalisation(p->axis_b))
        : axis_a_world;

    /* Relative velocity along slide axis. */
    vector3 vel_a = vector3_addition(body_a->velocity, vector3_cross(body_a->angular_velocity, r_a));
    vector3 vel_b = vector3_addition(body_b->velocity, vector3_cross(body_b->angular_velocity, r_b));
    vector3 rel_vel = vector3_subtraction(vel_b, vel_a);
    float rel_n = vector3_dot(rel_vel, axis_a_world);

    /* Current position along axis (measured, not dead-reckoned). */
    vector3 delta = vector3_subtraction(world_b, world_a);
    float current_pos = vector3_dot(delta, axis_a_world);

    /* Initialize position tracking once (reference = initial slide pos). */
    if (!p->position_initialized) {
        p->accumulated_position = current_pos;
        p->reference_axis_a = axis_a_world;
        p->reference_axis_b = axis_b_world;
        p->position_initialized = true;
    }

    /* ---- Perpendicular constraint: full 2-D (not 1-D slip dir).
     * TRUTH: old code killed only instantaneous perp_vel direction; when
     * |perp|~0 it applied zero and orthogonal drift grew free (wobble).
     * Build orthonormal basis (u,v) ⊥ axis and solve both. */
    {
        vector3 ref = (fabsf(axis_a_world.y) < 0.99f) ? (vector3){0.0f, 1.0f, 0.0f} : (vector3){1.0f, 0.0f, 0.0f};
        vector3 bu = vector3_subtraction(ref, vector3_scaling(axis_a_world, vector3_dot(ref, axis_a_world)));
        float bu_len_sq = vector3_length_squared(bu);
        if (bu_len_sq > 1e-12f) {
            bu = vector3_scaling(bu, 1.0f / sqrtf(bu_len_sq));
            vector3 bv = vector3_cross(axis_a_world, bu);
            vector3 basis[2] = {bu, bv};
            for (int bi = 0; bi < 2; bi++) {
                vector3 dir = basis[bi];
                float rel_d = vector3_dot(rel_vel, dir);
                /* Positional bias: keep anchors coincident off-axis. */
                float err_d = vector3_dot(delta, dir);
                float bias_d = C->joints.revolute_beta * err_d / dt;
                float max_b = C->joints.revolute_max_bias;
                if (bias_d > max_b) {
                    bias_d = max_b;
                } else if (bias_d < -max_b) {
                    bias_d = -max_b;
                }
                vector3 ra_d = vector3_cross(r_a, dir);
                vector3 rb_d = vector3_cross(r_b, dir);
                float k_d = inv_a + inv_b +
                            vector3_dot(ra_d, math3_multiplication_vector3(rigidbody_effective_inv_inertia(body_a),
                                                                          ra_d)) +
                            vector3_dot(rb_d, math3_multiplication_vector3(rigidbody_effective_inv_inertia(body_b),
                                                                          rb_d));
                if (k_d > 1e-12f) {
                    float lambda_d = -(rel_d + bias_d) / k_d;
                    vector3 imp = vector3_scaling(dir, lambda_d);
                    body_a->velocity = vector3_subtraction(body_a->velocity, vector3_scaling(imp, inv_a));
                    body_b->velocity = vector3_addition(body_b->velocity, vector3_scaling(imp, inv_b));
                    body_a->angular_velocity = vector3_subtraction(
                        body_a->angular_velocity,
                        math3_multiplication_vector3(rigidbody_effective_inv_inertia(body_a),
                                                     vector3_cross(r_a, imp)));
                    body_b->angular_velocity = vector3_addition(
                        body_b->angular_velocity,
                        math3_multiplication_vector3(rigidbody_effective_inv_inertia(body_b),
                                                     vector3_cross(r_b, imp)));
                }
            }
        }
    }

    /* TRUTH: no axial equality — a slider slides freely. Old
     * err=current-accumulated (≈0 by construction) made the lock vacuous
     * when limits were off (free by accident) and double-paid rel_n when
     * limits were on (axis block + limit block). Only limits constrain. */

    /* ---- Limits enforcement (measured position, single pay) ---- */
    if (p->limits_enabled) {
        /* TRUTH: limits key off measured current_pos, not dead-reckoned
         * accumulated (which drifted 64x/tick). Accumulated tracks for API
         * readout via pre_step; enforcement uses measurement. */
        bool at_min = (current_pos <= p->limit_min + 1e-4f) && (rel_n < 0.0f);
        bool at_max = (current_pos >= p->limit_max - 1e-4f) && (rel_n > 0.0f);
        if (at_min || at_max) {
            vector3 ra_axis = vector3_cross(r_a, axis_a_world);
            vector3 rb_axis = vector3_cross(r_b, axis_a_world);
            float k_axis = inv_a + inv_b +
                           vector3_dot(ra_axis,
                                       math3_multiplication_vector3(rigidbody_effective_inv_inertia(body_a), ra_axis)) +
                           vector3_dot(rb_axis,
                                       math3_multiplication_vector3(rigidbody_effective_inv_inertia(body_b), rb_axis));
            if (k_axis > 1e-12f) {
                float lambda_limit = -rel_n / k_axis;
                vector3 limit_impulse = vector3_scaling(axis_a_world, lambda_limit);
                body_a->velocity = vector3_subtraction(body_a->velocity, vector3_scaling(limit_impulse, inv_a));
                body_b->velocity = vector3_addition(body_b->velocity, vector3_scaling(limit_impulse, inv_b));
                body_a->angular_velocity = vector3_subtraction(
                    body_a->angular_velocity,
                    math3_multiplication_vector3(rigidbody_effective_inv_inertia(body_a),
                                                 vector3_cross(r_a, limit_impulse)));
                body_b->angular_velocity = vector3_addition(
                    body_b->angular_velocity,
                    math3_multiplication_vector3(rigidbody_effective_inv_inertia(body_b),
                                                 vector3_cross(r_b, limit_impulse)));
            }
        }
    }
}

/* TRUTH: once-per-tick slide tracking (called before the solver loop). */
void prismatic_pre_step(prismatic_params *p, rigidbody *body_a, rigidbody *body_b, float dt, const mpe_config_t *cfg) {
    (void) cfg;
    if ((!p) || (!body_a) || (!body_b) || (!(dt > 0.0f))) {
        return;
    }
    vector3 r_a = vector4_rotate_to_vector3(body_a->orientation, p->anchor_a);
    vector3 r_b = vector4_rotate_to_vector3(body_b->orientation, p->anchor_b);
    vector3 world_a = vector3_addition(body_a->position, r_a);
    vector3 world_b = vector3_addition(body_b->position, r_b);
    vector3 axis_w = vector4_rotate_to_vector3(body_a->orientation, vector3_normalisation(p->axis_a));
    float cur = vector3_dot(vector3_subtraction(world_b, world_a), axis_w);
    if (!isfinite(cur)) {
        return;
    }
    if (!p->position_initialized) {
        p->accumulated_position = cur;
        p->position_initialized = true;
    } else {
        /* Track for readout; enforcement uses measured cur (see solve). */
        vector3 vel_a = vector3_addition(body_a->velocity, vector3_cross(body_a->angular_velocity, r_a));
        vector3 vel_b = vector3_addition(body_b->velocity, vector3_cross(body_b->angular_velocity, r_b));
        float rel = vector3_dot(vector3_subtraction(vel_b, vel_a), axis_w);
        if (isfinite(rel)) {
            float d = rel * dt;
            if (d > 1.0f) {
                d = 1.0f;
            } else if (d < -1.0f) {
                d = -1.0f;
            }
            p->accumulated_position += d;
        }
    }
}

/* Rope: inequality distance constraint (pulls only, no push). */
void rope_solve(rope_params *p, rigidbody *body_a, rigidbody *body_b, float dt, const mpe_config_t *cfg) {
    const mpe_config_t *C = cfg ? cfg : &g_cfg;
    if ((!p) || (!body_a) || (!body_b) || (dt <= 0.0f)) return;
    if (!isfinite(p->rest_length) || p->rest_length < 0.0f) return;
    if (body_a->is_sleeping) rigidbody_wake(body_a);
    if (body_b->is_sleeping) rigidbody_wake(body_b);

    float inv_a = rigidbody_effective_inv_mass(body_a);
    float inv_b = rigidbody_effective_inv_mass(body_b);
    vector3 r_a = vector4_rotate_to_vector3(body_a->orientation, p->anchor_a);
    vector3 r_b = vector4_rotate_to_vector3(body_b->orientation, p->anchor_b);
    vector3 world_a = vector3_addition(body_a->position, r_a);
    vector3 world_b = vector3_addition(body_b->position, r_b);
    vector3 delta = vector3_subtraction(world_b, world_a);
    float dist = vector3_length(delta);

    if (dist < 1e-9f) return;

    /* Only pull when stretched beyond rest_length (inequality). */
    if (dist <= p->rest_length) return;

    vector3 n = vector3_scaling(delta, 1.0f / dist);
    float err = dist - p->rest_length;

    vector3 vel_a = vector3_addition(body_a->velocity, vector3_cross(body_a->angular_velocity, r_a));
    vector3 vel_b = vector3_addition(body_b->velocity, vector3_cross(body_b->angular_velocity, r_b));
    float rel_n = vector3_dot(vector3_subtraction(vel_b, vel_a), n);

    float bias = C->joints.revolute_beta * err / dt;
    float max_b = C->joints.revolute_max_bias;
    if (bias > max_b) bias = max_b;
    else if (bias < -max_b) bias = -max_b;

    vector3 ra_n = vector3_cross(r_a, n);
    vector3 rb_n = vector3_cross(r_b, n);
    float k = inv_a + inv_b +
        vector3_dot(ra_n, math3_multiplication_vector3(rigidbody_effective_inv_inertia(body_a), ra_n)) +
        vector3_dot(rb_n, math3_multiplication_vector3(rigidbody_effective_inv_inertia(body_b), rb_n));
    if (k <= 1e-12f) return;

    float lambda = -(rel_n + bias) / k;
    /* TRUTH: n=(B-A)/dist, impulse=n*lambda, B+=, A-=. Stretched+separating
     * (err>0, rel_n>0) needs lambda<0 (pull B toward A). Old clamp killed
     * exactly pull (lambda<0) and kept push — rope was a strut. Keep pull
     * (negative), kill push (positive). */
    if (lambda > 0.0f) {
        lambda = 0.0f; /* Only pull, never push. */
    }

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

/* Prismatic motor: adds drive force to the force accumulator (call once per tick). */
void prismatic_apply_motor(prismatic_params *p, rigidbody *body_a, rigidbody *body_b, float dt, const mpe_config_t *cfg) {
    const mpe_config_t *C = cfg ? cfg : &g_cfg;
    if ((!p) || (!p->motor_enabled) || (!body_a) || (!body_b) || (!(dt > 0.0f))) {
        return;
    }
    if (!isfinite(p->motor_target_speed) || !isfinite(p->motor_max_force) || p->motor_max_force <= 0.0f) {
        return;
    }
    /* TRUTH: respect slide limits. */
    if (p->limits_enabled && p->position_initialized) {
        vector3 axw0 = vector4_rotate_to_vector3(body_a->orientation, vector3_normalisation(p->axis_a));
        vector3 r_a0 = vector4_rotate_to_vector3(body_a->orientation, p->anchor_a);
        vector3 r_b0 = vector4_rotate_to_vector3(body_b->orientation, p->anchor_b);
        float cur0 =
            vector3_dot(vector3_subtraction(vector3_addition(body_b->position, r_b0),
                                            vector3_addition(body_a->position, r_a0)),
                        axw0);
        if ((cur0 <= p->limit_min + 1e-4f && p->motor_target_speed < 0.0f) ||
            (cur0 >= p->limit_max - 1e-4f && p->motor_target_speed > 0.0f)) {
            return;
        }
    }
    vector3 axis_world = vector4_rotate_to_vector3(body_a->orientation, vector3_normalisation(p->axis_a));
    if (vector3_length_squared(axis_world) < 1e-12f) {
        return;
    }
    axis_world = vector3_normalisation(axis_world);
    vector3 vel_a = vector3_addition(body_a->velocity, vector3_cross(body_a->angular_velocity,
        vector4_rotate_to_vector3(body_a->orientation, p->anchor_a)));
    vector3 vel_b = vector3_addition(body_b->velocity, vector3_cross(body_b->angular_velocity,
        vector4_rotate_to_vector3(body_b->orientation, p->anchor_b)));
    float current_speed = vector3_dot(vector3_subtraction(vel_b, vel_a), axis_world);
    float speed_error = p->motor_target_speed - current_speed;
    float motor_gain = C->joints.revolute_motor_gain;
    float desired_force = speed_error * motor_gain;
    if (desired_force > p->motor_max_force) {
        desired_force = p->motor_max_force;
    }
    if (desired_force < -p->motor_max_force) {
        desired_force = -p->motor_max_force;
    }
    vector3 drive_force = vector3_scaling(axis_world, desired_force);
    body_a->force_accumulator = vector3_subtraction(body_a->force_accumulator, drive_force);
    body_b->force_accumulator = vector3_addition(body_b->force_accumulator, drive_force);
}

/* Fixed weld: point-to-point (same K-matrix as revolute) plus full angular
 * lock (kill all relative spin, not just off-axis). Deterministic, no bias
 * beyond the shared Baumgarte cap. */
void fixed_solve(fixed_params *p, rigidbody *body_a, rigidbody *body_b, float dt, const mpe_config_t *cfg) {
    const mpe_config_t *C = cfg ? cfg : &g_cfg;
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
    const float beta = C->joints.revolute_beta;
    float err_len = vector3_length(err);
    vector3 bias = (err_len > 1e-9f)
        ? vector3_scaling(err, fminf(beta / dt, C->joints.revolute_max_bias / fmaxf(err_len, 1e-9f)))
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

void revolute_apply_motor(revolute_params *p, rigidbody *body_a, rigidbody *body_b, float dt, const mpe_config_t *cfg) {
    const mpe_config_t *C = cfg ? cfg : &g_cfg;
    (void) dt;
    if ((!p) || (!p->motor_enabled) || (!body_a) || (!body_b)) {
        return;
    }
    if (!isfinite(p->motor_target_speed) || !isfinite(p->motor_max_torque) || p->motor_max_torque <= 0.0f) {
        return;
    }
    /* TRUTH: respect angle limits — driving into a limit fights the limit
     * impulse every tick (jitter + energy). Refuse to drive past stops. */
    if (p->limits_enabled && p->angle_initialized) {
        float along_probe;
        {
            vector3 axw = vector4_rotate_to_vector3(body_a->orientation, vector3_normalisation(p->axis_a));
            vector3 relw = vector3_subtraction(body_b->angular_velocity, body_a->angular_velocity);
            along_probe = vector3_dot(relw, axw);
        }
        bool at_min = (p->accumulated_angle <= p->limit_min_rad + 1e-4f) && (p->motor_target_speed < 0.0f);
        bool at_max = (p->accumulated_angle >= p->limit_max_rad - 1e-4f) && (p->motor_target_speed > 0.0f);
        if ((at_min && along_probe <= 0.0f) || (at_max && along_probe >= 0.0f)) {
            /* At stop and driving further out: hold, don't push. Allow
             * driving back inward (opposite sign passes through). */
            if ((at_min && p->motor_target_speed < 0.0f) || (at_max && p->motor_target_speed > 0.0f)) {
                return;
            }
        }
    }
    vector3 axis_world = vector4_rotate_to_vector3(body_a->orientation, vector3_normalisation(p->axis_a));
    if (vector3_length_squared(axis_world) < 1e-12f) {
        return;
    }
    axis_world = vector3_normalisation(axis_world);
    vector3 relative_angular = vector3_subtraction(body_b->angular_velocity, body_a->angular_velocity);
    float current_speed = vector3_dot(relative_angular, axis_world);
    if (!isfinite(current_speed)) {
        return;
    }
    float speed_error = p->motor_target_speed - current_speed;
    if (!isfinite(speed_error)) {
        return;
    }
    float motor_gain = C->joints.revolute_motor_gain;
    if (!isfinite(motor_gain) || motor_gain < 0.0f) {
        motor_gain = 8.0f;
    }
    /* TRUTH: P-only with shared gain goes explicit-Euler unstable at
     * gain*dt/I >> 2 (gain=100, I=0.1, dt=1/60 => 16). Clamp torque so the
     * per-tick velocity change cannot exceed the remaining error (no
     * overshoot): |Δw| <= |err|. Δw = torque*dt*inv_eff. */
    if (motor_gain > 50.0f) {
        motor_gain = 50.0f;
    }
    float desired_torque = speed_error * motor_gain;
    if (desired_torque > p->motor_max_torque) {
        desired_torque = p->motor_max_torque;
    }
    if (desired_torque < -p->motor_max_torque) {
        desired_torque = -p->motor_max_torque;
    }
    /* Stability: no overshoot in one tick. */
    {
        float inv_sum = vector3_dot(axis_world,
                                    math3_multiplication_vector3(math3_addition(rigidbody_effective_inv_inertia(body_a),
                                                                                rigidbody_effective_inv_inertia(body_b)),
                                                                 axis_world));
        if (inv_sum > 1e-12f && dt > 0.0f) {
            float max_no_overshoot = fabsf(speed_error) / (dt * inv_sum);
            if (desired_torque > max_no_overshoot) {
                desired_torque = max_no_overshoot;
            } else if (desired_torque < -max_no_overshoot) {
                desired_torque = -max_no_overshoot;
            }
        }
    }
    if (!isfinite(desired_torque)) {
        return;
    }
    vector3 drive_torque = vector3_scaling(axis_world, desired_torque);
    body_a->torque_accumulator = vector3_subtraction(body_a->torque_accumulator, drive_torque);
    body_b->torque_accumulator = vector3_addition(body_b->torque_accumulator, drive_torque);
}

/* Distance: 1D constraint along the anchor axis. Preserves free rotation and
 * tangential motion; only the separation error is corrected. */
void distance_solve(distance_params *p, rigidbody *body_a, rigidbody *body_b, float dt, const mpe_config_t *cfg) {
    const mpe_config_t *C = cfg ? cfg : &g_cfg;
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
    float bias = C->joints.revolute_beta * err / dt;
    float max_b = C->joints.revolute_max_bias;
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
void fixed_correct_angular_drift(fixed_params *p, rigidbody *body_a, rigidbody *body_b, float dt, const mpe_config_t *cfg) {
    (void) cfg;
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

/* Positional axis-drift correction: MUST be called exactly once per tick,
 * AFTER the velocity iteration loop — never inside it. The error term is
 * positional (orientation difference, unchanged by velocity iterations),
 * so per-iteration application multiplies the correction by the iteration
 * count (64x at defaults): a spurious torsional spring that pumps energy
 * and destroys hinge truth (e.g. 9x-too-fast pendulum). */
void revolute_correct_axis_drift(revolute_params *p, rigidbody *body_a, rigidbody *body_b, float dt, const mpe_config_t *cfg) {
    (void) cfg;
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