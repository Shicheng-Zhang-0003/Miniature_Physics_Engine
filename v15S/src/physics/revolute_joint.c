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
#include "../core/det_math.h"
#include <math.h>
#include <stdio.h>

static math3 skew_symmetric (vector3 v) {
    math3 m = {{{0.0f}}};
    m.matrix[0][1] = -v.z;
    m.matrix[0][2] = v.y;
    m.matrix[1][0] = v.z;
    m.matrix[1][2] = -v.x;
    m.matrix[2][0] = -v.y;
    m.matrix[2][1] = v.x;
    return m;
}

static math3 math3_addition (math3 a, math3 b) {
    math3 r;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            r.matrix[i][j] = a.matrix[i][j] + b.matrix[i][j];
        }
    }
    return r;
}

/* 6x6 matrix operations for coupled hinge solve + motor.
 * TRUTH: K spans invM (~1e-6..1e4) plus Iinv*r^2 (up to ~1e15 for tiny
 * masses with long anchors). float (23-bit, ~1e7) cannot hold cond(K)>1e10:
 * pivot test is meaningless and lambda is garbage. Solve in double with
 * row/column equilibration (D=sqrt(diag), Ks=D^-1 K D^-1). */
/* Positive-definiteness probe for the symmetric K.
 *
 * K = J*M^-1*J^T is POSITIVE SEMIDEFINITE by construction for any Jacobian
 * J and any SPD M^-1 (a Lagrangian Gram matrix), and positive definite when
 * the constraint set is non-redundant — which it is here (3 independent p2p
 * rows, 2 independent axis rows, 1 motor row).
 *
 * DESPOT-2026-09-29: the K sign error below produced an INDEFINITE K, and
 * mat6_invert happily returned a plausible-looking inverse of it: the
 * equilibrated Gauss-Jordan never checks definiteness, so a wrong K became
 * garbage impulses with no diagnostic anywhere. Cholesky is the correct test
 * and costs nothing at 6x6. Returns 0 on a non-PD (or non-finite) K so the
 * caller can fall through to the sequential solver and report, rather than
 * applying nonsense.
 *
 * Only the symmetric part is tested; K is built symmetric by assignment, and
 * using (A+A^T)/2 keeps the probe honest if that ever stops being true. */
static int mat6_is_positive_definite (double m[6][6]) {
    double L[6][6];
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++)
            L[i][j] = 0.5 * (m[i][j] + m[j][i]);
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j <= i; j++) {
            double sum = L[i][j];
            for (int k = 0; k < j; k++)
                sum -= L[i][k] * L[j][k];
            if (i == j) {
                /* K carries an absolute scale spanning invM..Iinv*r^2 and a
                 * 1e-10 regulariser; test the pivot against a scale-relative
                 * floor rather than an absolute one, or a legitimately tiny
                 * but well-conditioned row reads as singular. */
                double scale = 0.0;
                for (int c = 0; c < 6; c++)
                    scale += L[i][c] * L[i][c];
                scale = (scale > 0.0) ? scale : 1.0;
                if (!(sum > 1e-14 * scale))
                    return 0;
                L[i][i] = sqrt (sum);
            } else {
                if (!(L[j][j] > 0.0))
                    return 0;
                L[i][j] = sum / L[j][j];
            }
            if (!isfinite (L[i][j]))
                return 0;
        }
    }
    return 1;
}
static void mat6_zero (double m[6][6]) {
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++)
            m[i][j] = 0.0;
}
static void mat6_vec_mul_d (double out[6], double m[6][6], double v[6]) {
    for (int i = 0; i < 6; i++) {
        out[i] = 0.0;
        for (int j = 0; j < 6; j++)
            out[i] += m[i][j] * v[j];
    }
}
static int mat6_invert (double m[6][6], double out[6][6]) {
    /* Equilibrate, then Gauss-Jordan with partial pivoting in double. */
    double d[6];
    for (int i = 0; i < 6; i++) {
        double dg = m[i][i];
        if (!(dg > 0.0)) {
            dg = 0.0;
            for (int j = 0; j < 6; j++) {
                double v = m[i][j];
                if (isfinite (v)) {
                    dg += v * v;
                }
            }
            dg = (dg > 0.0) ? sqrt (dg) : 1.0;
            d[i] = dg;
        } else {
            d[i] = sqrt (dg);
        }
        if (!(d[i] > 1e-18) || !isfinite (d[i])) {
            d[i] = 1.0;
        }
    }
    double aug[6][12];
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            double s = d[i] * d[j];
            aug[i][j] = (s > 0.0) ? m[i][j] / s : m[i][j];
        }
        for (int j = 0; j < 6; j++)
            aug[i][6 + j] = (i == j) ? 1.0 : 0.0;
    }
    for (int col = 0; col < 6; col++) {
        int pivot = col;
        double max_val = fabs (aug[col][col]);
        for (int row = col + 1; row < 6; row++) {
            if (fabs (aug[row][col]) > max_val) {
                max_val = fabs (aug[row][col]);
                pivot = row;
            }
        }
        if (!(max_val > 1e-18) || !isfinite (max_val))
            return 0; /* singular */
        if (pivot != col) {
            for (int j = 0; j < 12; j++) {
                double tmp = aug[col][j];
                aug[col][j] = aug[pivot][j];
                aug[pivot][j] = tmp;
            }
        }
        double piv_val = aug[col][col];
        for (int j = 0; j < 12; j++)
            aug[col][j] /= piv_val;
        for (int row = 0; row < 6; row++) {
            if (row == col)
                continue;
            double factor = aug[row][col];
            for (int j = 0; j < 12; j++)
                aug[row][j] -= factor * aug[col][j];
        }
    }
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            double s = d[i] * d[j];
            out[i][j] = aug[i][6 + j] / s;
        }
    }
    return 1;
}

/* DESPOT-2026-09-29: shared sleep guard for every joint solver.
 *
 * The joint solvers used to wake BOTH partners unconditionally, on every
 * iteration, on the stated grounds that "a sleeping hinge partner would
 * freeze the constraint". The measured consequence: a single box on a floor
 * sleeps at tick 47, but two boxes joined by a PASSIVE revolute never sleep at
 * all -- 0/400 ticks. That is permanent solver cost, sleep churn, and it
 * defeats the island skip in constraint.c.
 *
 * The fix is safe because islands_build() already merges jointed bodies into
 * one island: a jointed pair is either both awake or both asleep. When both
 * are asleep the constraint is satisfied by construction (velocities are
 * pinned to zero and the island is skipped), so the joint has nothing to do.
 *
 * `active_drive` must be true when the joint is genuinely doing work -- a
 * motor with a non-zero target, or a limit currently engaged. Those cases
 * still force a wake, because a driven joint must not be frozen by sleep.
 * The motor's own force path (rb_apply_forces_*) wakes via the torque
 * accumulator, so this is belt-and-braces rather than the only mechanism. */
static int a3_joint_solve_may_skip (int sleeping_a, int sleeping_b, int active_drive) {
    if (active_drive) {
        return 0;
    }
    return sleeping_a && sleeping_b;
}

static int a3_revolute_active_drive (const revolute_params *p) {
    if (p->motor_enabled && fabsf (p->motor_target_speed) > 1e-9f) {
        return 1;
    }
    return 0;
}

void revolute_solve (revolute_params *p, rigidbody *body_a, rigidbody *body_b, float dt, const mpe_config_t *cfg) {
    const mpe_config_t *C = cfg ? cfg : &g_cfg;
    if ((!p) || (!body_a) || (!body_b) || (dt <= 0.0f)) {
        return;
    }
    /* FIX-AUDIT-DESPOT: jointed bodies are force-woken here INTENTIONALLY
     * (FTC robots are always active; a sleeping hinge partner would freeze
     * the constraint). Sleep is still allowed when the joint is passive
     * (no motor, no limits): the island/sleep pass may put both partners
     * down and they stay down until contact/spring/motor wakes them — the
     * drift corrections above are gated on effective inv mass so they never
     * kick a sleeper back awake by themselves. */
    /* Jointed bodies stay awake so the constraint always acts. */
    if (a3_joint_solve_may_skip (body_a->is_sleeping, body_b->is_sleeping, a3_revolute_active_drive (p))) {
        return;
    }
    if (body_a->is_sleeping)
        rigidbody_wake (body_a);
    if (body_b->is_sleeping)
        rigidbody_wake (body_b);

    float inv_mass_a = rigidbody_effective_inv_mass (body_a);
    float inv_mass_b = rigidbody_effective_inv_mass (body_b);
    if ((inv_mass_a <= 0.0f) && (inv_mass_b <= 0.0f)) {
        return;
    }

    vector3 r_a = vector4_rotate_to_vector3 (body_a->orientation, p->anchor_a);
    vector3 r_b = vector4_rotate_to_vector3 (body_b->orientation, p->anchor_b);

    /* Anchor positions and velocities. */
    vector3 anchor_a_world = vector3_addition (body_a->position, r_a);
    vector3 anchor_b_world = vector3_addition (body_b->position, r_b);
    vector3 position_error = vector3_subtraction (anchor_b_world, anchor_a_world);

    vector3 vel_a_at_anchor = vector3_addition (body_a->velocity, vector3_cross (body_a->angular_velocity, r_a));
    vector3 vel_b_at_anchor = vector3_addition (body_b->velocity, vector3_cross (body_b->angular_velocity, r_b));
    vector3 relative_velocity = vector3_subtraction (vel_b_at_anchor, vel_a_at_anchor);

    /* Hinge axis in world space (from body A). */
    vector3 axis_world = vector4_rotate_to_vector3 (body_a->orientation, vector3_normalisation (p->axis_a));
    float axis_len_sq = vector3_length_squared (axis_world);
    if (axis_len_sq < 1e-12f)
        return;
    axis_world = vector3_scaling (axis_world, 1.0f / sqrtf (axis_len_sq));

    /* Build orthonormal basis (u, v) perpendicular to axis for axis alignment constraints.
     * u = normalize(axis × ref), v = axis × u. */
    vector3 ref = (fabsf (axis_world.y) < 0.99f) ? (vector3){0.0f, 1.0f, 0.0f} : (vector3){1.0f, 0.0f, 0.0f};
    vector3 u = vector3_cross (axis_world, ref);
    float u_len = sqrtf (vector3_length_squared (u));
    if (u_len < 1e-6f) {
        ref = (vector3){1.0f, 0.0f, 0.0f};
        u = vector3_cross (axis_world, ref);
        u_len = sqrtf (vector3_length_squared (u));
    }
    u = vector3_scaling (u, 1.0f / u_len);
    vector3 v = vector3_cross (axis_world, u); /* already unit length */

    /* Baumgarte bias for position error (point-to-point only; axis alignment is velocity-only).
     * TRUTH: velocity-level P2P bias solved INSIDE the iteration loop is
     * correct here (bias includes live rel_vel and converges; it is NOT the
     * once-per-tick axis-drift term, which must never enter the loop or its
     * position error pumps 64x). Do not move either across the boundary.
     * DESPOT-2026-10-04 (in-loop axis bias TRIED AND REVERTED, twice): the
     * external authorities all feed positional error back per-solve (Box2D
     * position pass, Bullet hinge-row ERP), so an axis bias mirroring the
     * P2P pattern was implemented and measured — at beta 0.3 it bounded
     * harsh-drive axle walk (7° vs 180° flip) but moved the MFS tank pivot
     * -19% and killed reverse-rotate outright; at beta 0.1 it flipped
     * forward-rotate's SIGN and halved the pivot. Non-monotonic in beta =
     * the pivot regime is chaotic-sensitive to joint formulation, so no
     * bolt-on bias ships without a full drive re-baseline. The harsh-abuse
     * tilt walk stays an open engine-side frontier (see KNOWN_FAILURES). */
    const float baumgarte_beta = C->joints.revolute_beta;
    float bias_speed = baumgarte_beta * vector3_length (position_error) / dt;
    float max_bias_speed = C->joints.revolute_max_bias;
    vector3 bias_p2p = (bias_speed > max_bias_speed && bias_speed > 0.0f)
                           ? vector3_scaling (position_error, (baumgarte_beta / dt) * (max_bias_speed / bias_speed))
                           : vector3_scaling (position_error, baumgarte_beta / dt);

    /* Effective mass/inertia. */
    math3 I_inv_a = rigidbody_effective_inv_inertia (body_a);
    math3 I_inv_b = rigidbody_effective_inv_inertia (body_b);
    math3 I_sum = math3_addition (I_inv_a, I_inv_b);

    /* Build 6×6 K-matrix (effective mass matrix for the 5 constraints + motor).
     * Rows 0-2: point-to-point (x, y, z)
     * Rows 3-4: axis alignment (u, v components of relative angular velocity)
     * Row 5: motor (relative angular velocity along hinge axis) */
    double K[6][6];
    mat6_zero (K);

    /* ---------- K = J*M^-1*J^T, assembled from the Jacobian ----------
     *
     * DESPOT-2026-09-29 REWRITE. K used to be hand-written block by block
     * with each sign derived by hand. Two sign errors lived in those
     * derivations for the whole life of the joint, and BOTH were invisible
     * because every test anchored the joint at a body centre (where the
     * cross-coupling block is identically zero).
     *
     * This version builds the 6x12 Jacobian J explicitly and forms
     * K = J*M^-1*J^T mechanically, so K cannot disagree with the impulses
     * that are applied. The Jacobian is fixed by the impulse application
     * below, which is the only thing that defines it:
     *
     *   v_a -= p2p/m_a ;  w_a -= IA^-1 ( r_a x p2p + lam_u*u + lam_v*v + lam_m*axis )
     *   v_b += p2p/m_b ;  w_b += IB^-1 ( r_b x p2p + lam_u*u + lam_v*v + lam_m*axis )
     *
     * Reading J^T off that (P = M^-1 J^T lambda, state order
     * [v_a | w_a | v_b | w_b]) and using skew symmetry (S is skew, so
     * COLUMN i of S equals -ROW i of S -- this transpose is exactly what the
     * old derivation dropped) gives:
     *
     *   J_p2p row i = [ -e_i ,  +(S_a row i) , +e_i , -(S_b row i) ]
     *   J_axis_u    = [  0   ,      -u       ,  0   ,    +u      ]
     *   J_axis_v    = [  0   ,      -v       ,  0   ,    +v      ]
     *   J_motor     = [  0   ,    -axis      ,  0   ,   +axis    ]
     *
     * with M^-1 = diag( (1/m_a)I3 , IA^-1 , (1/m_b)I3 , IB^-1 ).
     *
     * Consequences of using the mechanical form:
     *  - the p2p/axis cross block is -(Au + Bu), BOTH terms negative. The
     *    old code had (-Au + Bu); an intermediate fix used (+Au + Bu).
     *    Only (-Au - Bu) matches the measured constraint-space map.
     *  - the motor row is NOT decoupled from the p2p rows. The old
     *    "motor is pure angular, no linear coupling" comment was wrong:
     *    measured K[0][5] = -0.45, K[1][5] = -0.04. The p2p Jacobian has
     *    angular columns, so it couples to any purely-angular constraint.
     *    Zeroing those entries was only harmless while the motor was off
     *    (lambda[5] forced to 0); with the motor driven it corrupted the
     *    solve. Now every entry is real.
     *
     * Measured proof: tests/mpe_suite_c.c mpe_t_revolute_matrix builds the
     * true 6x6 map by applying a unit lambda in each column with the impulse
     * application above and comparing against this K, entry by entry. */
    {
        /* Each constraint row is stored as FOUR 3-vectors: the coefficients it
         * places on body A's linear and angular velocity, and body B's. K is
         * then a plain bilinear form, with no block-index arithmetic to get
         * wrong (the previous J[6][12] layout was off by two columns in the
         * body-B angular block, which is exactly the sort of silent sign /
         * index error this rewrite exists to prevent).
         *
         * State order is [v_a | w_a | v_b | w_b]; M^-1 is
         * diag( (1/m_a) I , IA^-1 , (1/m_b) I , IB^-1 ). */
        vector3 na[6], nwa[6], nb[6], nwb[6];
        for (int r2 = 0; r2 < 6; r2++) {
            na[r2] = vector3_zero ();
            nwa[r2] = vector3_zero ();
            nb[r2] = vector3_zero ();
            nwb[r2] = vector3_zero ();
        }
        vector3 rows_a[3], rows_b[3];
        rows_a[0] = (vector3){0.0f, -r_a.z, r_a.y};
        rows_a[1] = (vector3){r_a.z, 0.0f, -r_a.x};
        rows_a[2] = (vector3){-r_a.y, r_a.x, 0.0f};
        rows_b[0] = (vector3){0.0f, -r_b.z, r_b.y};
        rows_b[1] = (vector3){r_b.z, 0.0f, -r_b.x};
        rows_b[2] = (vector3){-r_b.y, r_b.x, 0.0f};

        for (int i = 0; i < 3; i++) {
            /* p2p row i: na = -e_i, nwa = +(S_a row i), nb = +e_i, nwb = -(S_b row i) */
            float e[3] = {0.0f, 0.0f, 0.0f};
            e[i] = 1.0f;
            na[i] = vector3_scaling ((vector3){e[0], e[1], e[2]}, -1.0f);
            nwa[i] = rows_a[i];
            nb[i] = (vector3){e[0], e[1], e[2]};
            nwb[i] = vector3_scaling (rows_b[i], -1.0f);
        }
        nwa[3] = vector3_scaling (u, -1.0f);
        nwb[3] = u;
        nwa[4] = vector3_scaling (v, -1.0f);
        nwb[4] = v;
        nwa[5] = vector3_scaling (axis_world, -1.0f);
        nwb[5] = axis_world;

        for (int r2 = 0; r2 < 6; r2++) {
            for (int c = 0; c < 6; c++) {
                double acc = (double) vector3_dot (na[r2], na[c]) * (double) inv_mass_a +
                             (double) vector3_dot (nwa[r2], math3_multiplication_vector3 (I_inv_a, nwa[c])) +
                             (double) vector3_dot (nb[r2], nb[c]) * (double) inv_mass_b +
                             (double) vector3_dot (nwb[r2], math3_multiplication_vector3 (I_inv_b, nwb[c]));
                K[r2][c] = acc;
            }
        }
    }

    /* DESPOT-2026-09-29: with the motor DISABLED the along-axis relative
     * angular velocity is genuinely unconstrained (a free hinge), so the
     * constraint set is the 5 rows/cols {p2p, axis_u, axis_v} and row/col 5
     * must be REMOVED from the system. Zeroing it here (rather than after the
     * solve) is what keeps "force lambda[5] = 0" consistent with the matrix:
     * the couplings are real, so nulling lambda[5] post-solve would leave the
     * p2p rows short by exactly K[0..2][5]*lambda5 -- measured 1.7e-2
     * residual when the zeroing was done after the solve.
     * The old code zeroed the motor<->p2p couplings UNCONDITIONALLY, calling
     * it "no linear coupling"; that comment was wrong (measured K[0][5] =
     * -0.45) and it silently corrupted every motor-driven solve. */
    if (!p->motor_enabled) {
        for (int i = 0; i < 6; i++) {
            K[i][5] = 0.0;
            K[5][i] = 0.0;
        }
        K[5][5] = 1.0; /* decoupled placeholder; never read (lambda[5] = 0) */
    }

    /* Add regularization for numerical stability (tiny diagonal). */
    for (int i = 0; i < 6; i++)
        K[i][i] += 1e-10;

    /* DESPOT-2026-09-29: K is a J*M^-1*J^T Gram matrix, so it is PD by
     * construction. Prove it here rather than trusting it: a non-PD K means
     * the Jacobian and the impulse application have gone out of sync, and
     * inverting it anyway yields plausible garbage impulses (that is exactly
     * how the cross-block sign error stayed invisible). Fall through to the
     * sequential solver, which is slower but stays correct. */
    if (!mat6_is_positive_definite (K)) {
        goto fallback_sequential;
    }

    /* RHS = -(J*v + bias). Bias only on P2P (first 3 rows). */
    double rhs[6];
    /* P2P rows: -(relative_velocity + bias_p2p) */
    vector3 rhs_p2p = vector3_scaling (vector3_addition (relative_velocity, bias_p2p), -1.0f);
    rhs[0] = rhs_p2p.x;
    rhs[1] = rhs_p2p.y;
    rhs[2] = rhs_p2p.z;
    /* Axis rows: -perpendicular_angular_velocity (no bias for axis alignment). */
    vector3 rel_ang = vector3_subtraction (body_b->angular_velocity, body_a->angular_velocity);
    rhs[3] = -vector3_dot (rel_ang, u);
    rhs[4] = -vector3_dot (rel_ang, v);
    /* Motor row: -(along_axis_velocity - motor_target_speed), or 0 when
     * disabled (free spin: no constraint on the hinge axis). */
    float along_axis = vector3_dot (rel_ang, axis_world);
    if (p->motor_enabled) {
        rhs[5] = -(along_axis - p->motor_target_speed);
    } else {
        rhs[5] = 0.0f;
    }

    /* Solve K * lambda = rhs (double). */
    double K_inv[6][6];
    if (!mat6_invert (K, K_inv)) {
        /* Singular - fall back to sequential solve. */
        goto fallback_sequential;
    }
    double lambda[6];
    mat6_vec_mul_d (lambda, K_inv, rhs);
    if (!p->motor_enabled) {
        lambda[5] = 0.0f; /* free hinge: never apply axis torque */
    } else if (p->motor_max_torque > 0.0f) {
        /* Single-budget split drive (DESPOT-2026-10-01): torque reaches the
         * bodies through TWO paths — the accumulator feedforward
         * (revolute_apply_motor, integrated as torque*dt) AND this clamped
         * constraint row. Old code clamped EACH to max_torque, so the sum
         * drove up to 2x commanded torque. Each path now owns half the
         * budget; the sum respects motor_max_torque while keeping the
         * one-tick contact convergence the clamped row provides. */
        float max_lam = 0.5f * p->motor_max_torque * dt;
        if (lambda[5] > max_lam)
            lambda[5] = max_lam;
        else if (lambda[5] < -max_lam)
            lambda[5] = -max_lam;
    }

    /* Apply impulses.
     * P2P impulse (3D): applied to both bodies.
     * Axis impulses (2D): angular impulses along u and v.
     * Motor impulse (1D): angular impulse along axis_world. */
    vector3 impulse_p2p = {(float) lambda[0], (float) lambda[1], (float) lambda[2]};
    vector3 axis_impulse =
        vector3_addition (vector3_scaling (u, (float) lambda[3]), vector3_scaling (v, (float) lambda[4]));
    float motor_lambda = (float) lambda[5];
    vector3 motor_impulse = vector3_scaling (axis_world, motor_lambda);

    body_a->velocity = vector3_subtraction (body_a->velocity, vector3_scaling (impulse_p2p, inv_mass_a));
    body_b->velocity = vector3_addition (body_b->velocity, vector3_scaling (impulse_p2p, inv_mass_b));
    body_a->angular_velocity = vector3_subtraction (
        body_a->angular_velocity,
        math3_multiplication_vector3 (
            I_inv_a,
            vector3_addition (vector3_addition (vector3_cross (r_a, impulse_p2p), axis_impulse), motor_impulse)));
    body_b->angular_velocity = vector3_addition (
        body_b->angular_velocity,
        math3_multiplication_vector3 (
            I_inv_b,
            vector3_addition (vector3_addition (vector3_cross (r_b, impulse_p2p), axis_impulse), motor_impulse)));

    /* ---- angle limits: persistent relative-angle tracking + velocity-level enforcement ---- */
    if (p->limits_enabled) {
        if (!p->angle_initialized) {
            /* DESPOT-2026-10-01: atan2f/sqrtf are libm transcendentals, not
             * IEEE-exact ops. This runs once per joint-limits-enable (cold
             * path, not per-tick), so mark the trig fallback for honesty
             * rather than pretending determinism. Per-tick angle uses
             * dead-reckoned integration below (exact mults/adds only). */
            det_mark_fallback_trig ();
            vector4 q_a_inv = {body_a->orientation.w, -body_a->orientation.x, -body_a->orientation.y,
                               -body_a->orientation.z};
            vector4 q_rel = vector4_multiplication (q_a_inv, body_b->orientation);
            q_rel = vector4_normalisation (q_rel);
            if (q_rel.w < 0.0f) {
                q_rel.w = -q_rel.w;
                q_rel.x = -q_rel.x;
                q_rel.y = -q_rel.y;
                q_rel.z = -q_rel.z;
            }
            float half_angle = atan2f (sqrtf (q_rel.x * q_rel.x + q_rel.y * q_rel.y + q_rel.z * q_rel.z), q_rel.w);
            vector3 rot_axis = {q_rel.x, q_rel.y, q_rel.z};
            float rot_axis_len = sqrtf (vector3_length_squared (rot_axis));
            if (rot_axis_len > 1e-6f)
                rot_axis = vector3_scaling (rot_axis, 1.0f / rot_axis_len);
            else
                rot_axis = axis_world;
            float axis_dot = vector3_dot (rot_axis, axis_world);
            float initial_angle = 2.0f * half_angle * axis_dot;
            /* TRUTH: NO [-pi,pi] wrap (dead-reckoned joint coordinate must
             * stay unwrapped: wrapping made limits outside [-pi,pi]
             * unreachable and teleported multi-turn assemblies. Integration
             * of along_axis IS the angle for single-axis hinge motion
             * (exact, not approximate); velocity-level limit kills rebase
             * drift every iteration. */
            p->accumulated_angle = initial_angle;
            p->reference_axis_a = vector4_rotate_to_vector3 (body_a->orientation, vector3_normalisation (p->axis_a));
            p->reference_axis_b = vector4_rotate_to_vector3 (
                body_b->orientation, (vector3_length_squared (p->axis_b) > 1e-12f) ? vector3_normalisation (p->axis_b)
                                                                                   : vector3_normalisation (p->axis_a));
            p->angle_initialized = true;
        }

        float min_limit = p->limit_min_rad;
        float max_limit = p->limit_max_rad;
        float along_axis =
            vector3_dot (vector3_subtraction (body_b->angular_velocity, body_a->angular_velocity), axis_world);
        bool at_min = (p->accumulated_angle <= min_limit + 1e-4f) && (along_axis < 0.0f);
        bool at_max = (p->accumulated_angle >= max_limit - 1e-4f) && (along_axis > 0.0f);

        if (at_min || at_max) {
            float axis_mass_inv = vector3_dot (axis_world, math3_multiplication_vector3 (I_sum, axis_world));
            float axis_mass = (axis_mass_inv > 1e-12f) ? (1.0f / axis_mass_inv) : 0.0f;
            if (axis_mass > 0.0f) {
                float limit_lambda = -along_axis * axis_mass;
                vector3 limit_impulse = vector3_scaling (axis_world, limit_lambda);
                body_a->angular_velocity = vector3_subtraction (body_a->angular_velocity,
                                                                math3_multiplication_vector3 (I_inv_a, limit_impulse));
                body_b->angular_velocity =
                    vector3_addition (body_b->angular_velocity, math3_multiplication_vector3 (I_inv_b, limit_impulse));
                /* DESPOT-2026-10-01: NO accumulated_angle = limit clamp here.
                 * That clamp existed to stop dead-reckoning windup into the
                 * stop, but it PINNED the books at the limit while the true
                 * twist sat 0.085 past it (measured) — enforcement blind
                 * forever after, since acc == min reads as "holding". With
                 * the clamp gone the books keep integrating live velocity and
                 * can never freeze-false; whatever drift violent motion
                 * causes stays visible to enforcement instead of being paved
                 * over. The velocity kill above is the enforcement. */
            }
        }
    }
    return;

fallback_sequential:
    /* Fallback to original sequential solve if the 6x6 solve fails. */
    /* ---- point-to-point ---- */
    {
        /* The sequential path solves the p2p rows on their own (no axis or
         * motor coupling), so it needs only the 3x3 diagonal block. K's
         * cross/motor blocks are built mechanically above and are not used
         * here. */
        math3 skew_a = skew_symmetric (r_a);
        math3 skew_b = skew_symmetric (r_b);
        float inv_mass_sum = inv_mass_a + inv_mass_b;
        math3 k = {{{0.0f}}};
        for (int i = 0; i < 3; i++)
            k.matrix[i][i] = inv_mass_sum;
        math3 term_a = math3_multiplication (skew_a, math3_multiplication (I_inv_a, skew_a));
        math3 term_b = math3_multiplication (skew_b, math3_multiplication (I_inv_b, skew_b));
        /* SUBTRACT rotational terms (S*M*S is negative-semidefinite; a
         * J*M^-1*J^T Gram block must be PSD). */
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                k.matrix[i][j] -= term_a.matrix[i][j] + term_b.matrix[i][j];
        math3 k_inv = math3_inverse (k);
        vector3 rhs_vec = vector3_scaling (vector3_addition (relative_velocity, bias_p2p), -1.0f);
        vector3 impulse = math3_multiplication_vector3 (k_inv, rhs_vec);
        body_a->velocity = vector3_subtraction (body_a->velocity, vector3_scaling (impulse, inv_mass_a));
        body_b->velocity = vector3_addition (body_b->velocity, vector3_scaling (impulse, inv_mass_b));
        body_a->angular_velocity = vector3_subtraction (
            body_a->angular_velocity, math3_multiplication_vector3 (I_inv_a, vector3_cross (r_a, impulse)));
        body_b->angular_velocity = vector3_addition (
            body_b->angular_velocity, math3_multiplication_vector3 (I_inv_b, vector3_cross (r_b, impulse)));
    }
    /* ---- axis alignment ---- */
    {
        vector3 rel_ang = vector3_subtraction (body_b->angular_velocity, body_a->angular_velocity);
        vector3 perp_ang =
            vector3_subtraction (rel_ang, vector3_scaling (axis_world, vector3_dot (rel_ang, axis_world)));
        math3 ang_mass = math3_addition (I_inv_a, I_inv_b);
        math3 ang_mass_inv = math3_inverse (ang_mass);
        vector3 ang_imp = vector3_scaling (math3_multiplication_vector3 (ang_mass_inv, perp_ang), -1.0f);
        body_a->angular_velocity =
            vector3_subtraction (body_a->angular_velocity, math3_multiplication_vector3 (I_inv_a, ang_imp));
        body_b->angular_velocity =
            vector3_addition (body_b->angular_velocity, math3_multiplication_vector3 (I_inv_b, ang_imp));
    }
    /* ---- limits ---- (same as above) */
    if (p->limits_enabled) {
        float min_limit = p->limit_min_rad;
        float max_limit = p->limit_max_rad;
        float along_axis =
            vector3_dot (vector3_subtraction (body_b->angular_velocity, body_a->angular_velocity), axis_world);
        bool at_min = (p->accumulated_angle <= min_limit + 1e-4f) && (along_axis < 0.0f);
        bool at_max = (p->accumulated_angle >= max_limit - 1e-4f) && (along_axis > 0.0f);
        if (at_min || at_max) {
            float axis_mass_inv = vector3_dot (axis_world, math3_multiplication_vector3 (I_sum, axis_world));
            float axis_mass = (axis_mass_inv > 1e-12f) ? (1.0f / axis_mass_inv) : 0.0f;
            if (axis_mass > 0.0f) {
                float limit_lambda = -along_axis * axis_mass;
                vector3 limit_impulse = vector3_scaling (axis_world, limit_lambda);
                body_a->angular_velocity = vector3_subtraction (body_a->angular_velocity,
                                                                math3_multiplication_vector3 (I_inv_a, limit_impulse));
                body_b->angular_velocity =
                    vector3_addition (body_b->angular_velocity, math3_multiplication_vector3 (I_inv_b, limit_impulse));
                /* DESPOT-2026-10-01: no books clamp (see 6x6 path above). */
            }
        }
    }
}

/* TRUTH: once-per-tick angle integration (called before the solver loop).
 * DESPOT-2026-10-01: a measured-twist replacement (quaternion-delta
 * accumulation, exact ops) was tried here and REVERTED. It tracked settled
 * stops better (0.04 vs 0.085 residual) but was worse in transients, added a
 * 0.01 rad / 2 min bias under free multi-turn spin, and rests on a
 * swing-twist decomposition that is frame-ambiguous under large swing — the
 * torture probe cannot tell method error from definition ambiguity. Dead
 * reckoning's projection ISOLATES the twist rate exactly (swing never
 * contaminates a velocity projection), so in-envelope behaviour (steady
 * motor drives, pendulum swings: measured 0.0000 violation) is exact, and
 * the only error source is in-loop velocity changes the pre-loop sample
 * misses — i.e. exactly when the solver is starved. What ships instead is
 * the blindness fix below: the old per-iteration `accumulated = limit`
 * clamps FROZE the books at the stop while the true twist sat 0.085 past it
 * (enforcement blind forever after). With the clamps gone the books keep
 * integrating live velocity and can never freeze-false; residual drift under
 * violent motion is honest and documented, not hidden. */
void revolute_pre_step (revolute_params *p, rigidbody *body_a, rigidbody *body_b, float dt, const mpe_config_t *cfg) {
    (void) cfg;
    if ((!p) || (!body_a) || (!body_b) || (!(dt > 0.0f))) {
        return;
    }
    if (!p->limits_enabled || !p->angle_initialized) {
        return;
    }
    vector3 axis_world = vector4_rotate_to_vector3 (body_a->orientation, vector3_normalisation (p->axis_a));
    vector3 rel_ang = vector3_subtraction (body_b->angular_velocity, body_a->angular_velocity);
    float along = vector3_dot (rel_ang, axis_world);
    if (!isfinite (along)) {
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
     * so multi-turn limits work. (The per-solve limit clamps that used to
     * rebase the books here were removed: see the 6x6 path note.) */
}

/* Prismatic: single-axis slide with optional limits and motor. */
void prismatic_solve (prismatic_params *p, rigidbody *body_a, rigidbody *body_b, float dt, const mpe_config_t *cfg) {
    const mpe_config_t *C = cfg ? cfg : &g_cfg;
    if ((!p) || (!body_a) || (!body_b) || (dt <= 0.0f)) {
        return;
    }
    if (a3_joint_solve_may_skip (body_a->is_sleeping, body_b->is_sleeping, 0)) {
        return;
    }
    if (body_a->is_sleeping)
        rigidbody_wake (body_a);
    if (body_b->is_sleeping)
        rigidbody_wake (body_b);

    float inv_a = rigidbody_effective_inv_mass (body_a);
    float inv_b = rigidbody_effective_inv_mass (body_b);
    if ((inv_a <= 0.0f) && (inv_b <= 0.0f))
        return;

    vector3 r_a = vector4_rotate_to_vector3 (body_a->orientation, p->anchor_a);
    vector3 r_b = vector4_rotate_to_vector3 (body_b->orientation, p->anchor_b);
    vector3 world_a = vector3_addition (body_a->position, r_a);
    vector3 world_b = vector3_addition (body_b->position, r_b);

    /* Slide axis in world space (from body A). */
    vector3 axis_a_world = vector4_rotate_to_vector3 (body_a->orientation, vector3_normalisation (p->axis_a));
    vector3 axis_b_world = (vector3_length_squared (p->axis_b) > 1e-12f)
                               ? vector4_rotate_to_vector3 (body_b->orientation, vector3_normalisation (p->axis_b))
                               : axis_a_world;

    /* Relative velocity along slide axis. */
    vector3 vel_a = vector3_addition (body_a->velocity, vector3_cross (body_a->angular_velocity, r_a));
    vector3 vel_b = vector3_addition (body_b->velocity, vector3_cross (body_b->angular_velocity, r_b));
    vector3 rel_vel = vector3_subtraction (vel_b, vel_a);
    float rel_n = vector3_dot (rel_vel, axis_a_world);

    /* Current position along axis (measured, not dead-reckoned). */
    vector3 delta = vector3_subtraction (world_b, world_a);
    float current_pos = vector3_dot (delta, axis_a_world);

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
        vector3 ref = (fabsf (axis_a_world.y) < 0.99f) ? (vector3){0.0f, 1.0f, 0.0f} : (vector3){1.0f, 0.0f, 0.0f};
        vector3 bu = vector3_subtraction (ref, vector3_scaling (axis_a_world, vector3_dot (ref, axis_a_world)));
        float bu_len_sq = vector3_length_squared (bu);
        if (bu_len_sq > 1e-12f) {
            bu = vector3_scaling (bu, 1.0f / sqrtf (bu_len_sq));
            vector3 bv = vector3_cross (axis_a_world, bu);
            vector3 basis[2] = {bu, bv};
            for (int bi = 0; bi < 2; bi++) {
                vector3 dir = basis[bi];
                float rel_d = vector3_dot (rel_vel, dir);
                /* Positional bias: keep anchors coincident off-axis. */
                float err_d = vector3_dot (delta, dir);
                float bias_d = C->joints.revolute_beta * err_d / dt;
                float max_b = C->joints.revolute_max_bias;
                if (bias_d > max_b) {
                    bias_d = max_b;
                } else if (bias_d < -max_b) {
                    bias_d = -max_b;
                }
                vector3 ra_d = vector3_cross (r_a, dir);
                vector3 rb_d = vector3_cross (r_b, dir);
                float k_d =
                    inv_a + inv_b +
                    vector3_dot (ra_d, math3_multiplication_vector3 (rigidbody_effective_inv_inertia (body_a), ra_d)) +
                    vector3_dot (rb_d, math3_multiplication_vector3 (rigidbody_effective_inv_inertia (body_b), rb_d));
                if (k_d > 1e-12f) {
                    float lambda_d = -(rel_d + bias_d) / k_d;
                    vector3 imp = vector3_scaling (dir, lambda_d);
                    body_a->velocity = vector3_subtraction (body_a->velocity, vector3_scaling (imp, inv_a));
                    body_b->velocity = vector3_addition (body_b->velocity, vector3_scaling (imp, inv_b));
                    body_a->angular_velocity =
                        vector3_subtraction (body_a->angular_velocity,
                                             math3_multiplication_vector3 (rigidbody_effective_inv_inertia (body_a),
                                                                           vector3_cross (r_a, imp)));
                    body_b->angular_velocity =
                        vector3_addition (body_b->angular_velocity,
                                          math3_multiplication_vector3 (rigidbody_effective_inv_inertia (body_b),
                                                                        vector3_cross (r_b, imp)));
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
            vector3 ra_axis = vector3_cross (r_a, axis_a_world);
            vector3 rb_axis = vector3_cross (r_b, axis_a_world);
            float k_axis =
                inv_a + inv_b +
                vector3_dot (ra_axis,
                             math3_multiplication_vector3 (rigidbody_effective_inv_inertia (body_a), ra_axis)) +
                vector3_dot (rb_axis, math3_multiplication_vector3 (rigidbody_effective_inv_inertia (body_b), rb_axis));
            if (k_axis > 1e-12f) {
                float lambda_limit = -rel_n / k_axis;
                vector3 limit_impulse = vector3_scaling (axis_a_world, lambda_limit);
                body_a->velocity = vector3_subtraction (body_a->velocity, vector3_scaling (limit_impulse, inv_a));
                body_b->velocity = vector3_addition (body_b->velocity, vector3_scaling (limit_impulse, inv_b));
                body_a->angular_velocity = vector3_subtraction (
                    body_a->angular_velocity, math3_multiplication_vector3 (rigidbody_effective_inv_inertia (body_a),
                                                                            vector3_cross (r_a, limit_impulse)));
                body_b->angular_velocity = vector3_addition (
                    body_b->angular_velocity, math3_multiplication_vector3 (rigidbody_effective_inv_inertia (body_b),
                                                                            vector3_cross (r_b, limit_impulse)));
            }
        }
    }
}

/* TRUTH: once-per-tick slide tracking (called before the solver loop). */
void prismatic_pre_step (prismatic_params *p, rigidbody *body_a, rigidbody *body_b, float dt, const mpe_config_t *cfg) {
    (void) cfg;
    if ((!p) || (!body_a) || (!body_b) || (!(dt > 0.0f))) {
        return;
    }
    vector3 r_a = vector4_rotate_to_vector3 (body_a->orientation, p->anchor_a);
    vector3 r_b = vector4_rotate_to_vector3 (body_b->orientation, p->anchor_b);
    vector3 world_a = vector3_addition (body_a->position, r_a);
    vector3 world_b = vector3_addition (body_b->position, r_b);
    vector3 axis_w = vector4_rotate_to_vector3 (body_a->orientation, vector3_normalisation (p->axis_a));
    float cur = vector3_dot (vector3_subtraction (world_b, world_a), axis_w);
    if (!isfinite (cur)) {
        return;
    }
    if (!p->position_initialized) {
        p->accumulated_position = cur;
        p->position_initialized = true;
    } else {
        /* Track for readout; enforcement uses measured cur (see solve). */
        vector3 vel_a = vector3_addition (body_a->velocity, vector3_cross (body_a->angular_velocity, r_a));
        vector3 vel_b = vector3_addition (body_b->velocity, vector3_cross (body_b->angular_velocity, r_b));
        float rel = vector3_dot (vector3_subtraction (vel_b, vel_a), axis_w);
        if (isfinite (rel)) {
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
void rope_solve (rope_params *p, rigidbody *body_a, rigidbody *body_b, float dt, const mpe_config_t *cfg) {
    const mpe_config_t *C = cfg ? cfg : &g_cfg;
    if ((!p) || (!body_a) || (!body_b) || (dt <= 0.0f))
        return;
    if (!isfinite (p->rest_length) || p->rest_length < 0.0f)
        return;
    if (a3_joint_solve_may_skip (body_a->is_sleeping, body_b->is_sleeping, 0)) {
        return;
    }
    if (body_a->is_sleeping)
        rigidbody_wake (body_a);
    if (body_b->is_sleeping)
        rigidbody_wake (body_b);

    float inv_a = rigidbody_effective_inv_mass (body_a);
    float inv_b = rigidbody_effective_inv_mass (body_b);
    vector3 r_a = vector4_rotate_to_vector3 (body_a->orientation, p->anchor_a);
    vector3 r_b = vector4_rotate_to_vector3 (body_b->orientation, p->anchor_b);
    vector3 world_a = vector3_addition (body_a->position, r_a);
    vector3 world_b = vector3_addition (body_b->position, r_b);
    vector3 delta = vector3_subtraction (world_b, world_a);
    float dist = vector3_length (delta);

    if (dist < 1e-9f)
        return;

    /* Only pull when stretched beyond rest_length (inequality). */
    if (dist <= p->rest_length)
        return;

    vector3 n = vector3_scaling (delta, 1.0f / dist);
    float err = dist - p->rest_length;

    vector3 vel_a = vector3_addition (body_a->velocity, vector3_cross (body_a->angular_velocity, r_a));
    vector3 vel_b = vector3_addition (body_b->velocity, vector3_cross (body_b->angular_velocity, r_b));
    float rel_n = vector3_dot (vector3_subtraction (vel_b, vel_a), n);

    float bias = C->joints.revolute_beta * err / dt;
    float max_b = C->joints.revolute_max_bias;
    if (bias > max_b)
        bias = max_b;
    else if (bias < -max_b)
        bias = -max_b;

    vector3 ra_n = vector3_cross (r_a, n);
    vector3 rb_n = vector3_cross (r_b, n);
    float k = inv_a + inv_b +
              vector3_dot (ra_n, math3_multiplication_vector3 (rigidbody_effective_inv_inertia (body_a), ra_n)) +
              vector3_dot (rb_n, math3_multiplication_vector3 (rigidbody_effective_inv_inertia (body_b), rb_n));
    if (k <= 1e-12f)
        return;

    float lambda = -(rel_n + bias) / k;
    /* TRUTH: n=(B-A)/dist, impulse=n*lambda, B+=, A-=. Stretched+separating
     * (err>0, rel_n>0) needs lambda<0 (pull B toward A). Old clamp killed
     * exactly pull (lambda<0) and kept push — rope was a strut. Keep pull
     * (negative), kill push (positive). */
    if (lambda > 0.0f) {
        lambda = 0.0f; /* Only pull, never push. */
    }

    vector3 impulse = vector3_scaling (n, lambda);
    body_a->velocity = vector3_subtraction (body_a->velocity, vector3_scaling (impulse, inv_a));
    body_b->velocity = vector3_addition (body_b->velocity, vector3_scaling (impulse, inv_b));
    body_a->angular_velocity = vector3_subtraction (
        body_a->angular_velocity,
        math3_multiplication_vector3 (rigidbody_effective_inv_inertia (body_a), vector3_cross (r_a, impulse)));
    body_b->angular_velocity = vector3_addition (
        body_b->angular_velocity,
        math3_multiplication_vector3 (rigidbody_effective_inv_inertia (body_b), vector3_cross (r_b, impulse)));
}

/* Prismatic motor: adds drive force to the force accumulator (call once per tick). */
void prismatic_apply_motor (prismatic_params *p, rigidbody *body_a, rigidbody *body_b, float dt,
                            const mpe_config_t *cfg) {
    const mpe_config_t *C = cfg ? cfg : &g_cfg;
    if ((!p) || (!p->motor_enabled) || (!body_a) || (!body_b) || (!(dt > 0.0f))) {
        return;
    }
    if (!isfinite (p->motor_target_speed) || !isfinite (p->motor_max_force) || p->motor_max_force <= 0.0f) {
        return;
    }
    /* TRUTH: respect slide limits. */
    if (p->limits_enabled && p->position_initialized) {
        vector3 axw0 = vector4_rotate_to_vector3 (body_a->orientation, vector3_normalisation (p->axis_a));
        vector3 r_a0 = vector4_rotate_to_vector3 (body_a->orientation, p->anchor_a);
        vector3 r_b0 = vector4_rotate_to_vector3 (body_b->orientation, p->anchor_b);
        float cur0 = vector3_dot (
            vector3_subtraction (vector3_addition (body_b->position, r_b0), vector3_addition (body_a->position, r_a0)),
            axw0);
        if ((cur0 <= p->limit_min + 1e-4f && p->motor_target_speed < 0.0f) ||
            (cur0 >= p->limit_max - 1e-4f && p->motor_target_speed > 0.0f)) {
            return;
        }
    }
    vector3 axis_world = vector4_rotate_to_vector3 (body_a->orientation, vector3_normalisation (p->axis_a));
    if (vector3_length_squared (axis_world) < 1e-12f) {
        return;
    }
    axis_world = vector3_normalisation (axis_world);
    vector3 vel_a = vector3_addition (
        body_a->velocity,
        vector3_cross (body_a->angular_velocity, vector4_rotate_to_vector3 (body_a->orientation, p->anchor_a)));
    vector3 vel_b = vector3_addition (
        body_b->velocity,
        vector3_cross (body_b->angular_velocity, vector4_rotate_to_vector3 (body_b->orientation, p->anchor_b)));
    float current_speed = vector3_dot (vector3_subtraction (vel_b, vel_a), axis_world);
    float speed_error = p->motor_target_speed - current_speed;
    float motor_gain = C->joints.revolute_motor_gain;
    float desired_force = speed_error * motor_gain;
    if (desired_force > p->motor_max_force) {
        desired_force = p->motor_max_force;
    }
    if (desired_force < -p->motor_max_force) {
        desired_force = -p->motor_max_force;
    }
    vector3 drive_force = vector3_scaling (axis_world, desired_force);
    body_a->force_accumulator = vector3_subtraction (body_a->force_accumulator, drive_force);
    body_b->force_accumulator = vector3_addition (body_b->force_accumulator, drive_force);
}

/* Fixed weld: point-to-point (same K-matrix as revolute) plus full angular
 * lock (kill all relative spin, not just off-axis). Deterministic, no bias
 * beyond the shared Baumgarte cap. */
void fixed_solve (fixed_params *p, rigidbody *body_a, rigidbody *body_b, float dt, const mpe_config_t *cfg) {
    const mpe_config_t *C = cfg ? cfg : &g_cfg;
    if ((!p) || (!body_a) || (!body_b) || (dt <= 0.0f)) {
        return;
    }
    if (a3_joint_solve_may_skip (body_a->is_sleeping, body_b->is_sleeping, 0)) {
        return;
    }
    if (body_a->is_sleeping) {
        rigidbody_wake (body_a);
    }
    if (body_b->is_sleeping) {
        rigidbody_wake (body_b);
    }
    float inv_a = rigidbody_effective_inv_mass (body_a);
    float inv_b = rigidbody_effective_inv_mass (body_b);
    if ((inv_a <= 0.0f) && (inv_b <= 0.0f)) {
        return;
    }
    vector3 r_a = vector4_rotate_to_vector3 (body_a->orientation, p->anchor_a);
    vector3 r_b = vector4_rotate_to_vector3 (body_b->orientation, p->anchor_b);
    vector3 world_a = vector3_addition (body_a->position, r_a);
    vector3 world_b = vector3_addition (body_b->position, r_b);
    vector3 err = vector3_subtraction (world_b, world_a);
    vector3 vel_a = vector3_addition (body_a->velocity, vector3_cross (body_a->angular_velocity, r_a));
    vector3 vel_b = vector3_addition (body_b->velocity, vector3_cross (body_b->angular_velocity, r_b));
    vector3 rel = vector3_subtraction (vel_b, vel_a);
    const float beta = C->joints.revolute_beta;
    float err_len = vector3_length (err);
    vector3 bias = (err_len > 1e-9f)
                       ? vector3_scaling (err, fminf (beta / dt, C->joints.revolute_max_bias / fmaxf (err_len, 1e-9f)))
                       : vector3_zero ();
    math3 skew_a = skew_symmetric (r_a);
    math3 skew_b = skew_symmetric (r_b);
    math3 k = {{{0.0f}}};
    float inv_sum = inv_a + inv_b;
    for (int i = 0; i < 3; i++) {
        k.matrix[i][i] = inv_sum;
    }
    math3 term_a =
        math3_multiplication (skew_a, math3_multiplication (rigidbody_effective_inv_inertia (body_a), skew_a));
    math3 term_b =
        math3_multiplication (skew_b, math3_multiplication (rigidbody_effective_inv_inertia (body_b), skew_b));
    /* k = inv_sum*I - term_a - term_b */
    for (int c = 0; c < 3; c++) {
        for (int r = 0; r < 3; r++) {
            k.matrix[c][r] -= term_a.matrix[c][r] + term_b.matrix[c][r];
        }
    }
    math3 k_inv = math3_inverse (k);
    vector3 rhs = vector3_scaling (vector3_addition (rel, bias), -1.0f);
    vector3 impulse = math3_multiplication_vector3 (k_inv, rhs);
    body_a->velocity = vector3_subtraction (body_a->velocity, vector3_scaling (impulse, inv_a));
    body_b->velocity = vector3_addition (body_b->velocity, vector3_scaling (impulse, inv_b));
    body_a->angular_velocity = vector3_subtraction (
        body_a->angular_velocity,
        math3_multiplication_vector3 (rigidbody_effective_inv_inertia (body_a), vector3_cross (r_a, impulse)));
    body_b->angular_velocity = vector3_addition (
        body_b->angular_velocity,
        math3_multiplication_vector3 (rigidbody_effective_inv_inertia (body_b), vector3_cross (r_b, impulse)));
    /* Full angular lock: remove all relative spin. */
    vector3 rel_w = vector3_subtraction (body_b->angular_velocity, body_a->angular_velocity);
    math3 m = math3_addition (rigidbody_effective_inv_inertia (body_a), rigidbody_effective_inv_inertia (body_b));
    math3 m_inv = math3_inverse (m);
    vector3 ang_imp = vector3_scaling (math3_multiplication_vector3 (m_inv, rel_w), -1.0f);
    body_a->angular_velocity = vector3_subtraction (
        body_a->angular_velocity, math3_multiplication_vector3 (rigidbody_effective_inv_inertia (body_a), ang_imp));
    body_b->angular_velocity = vector3_addition (
        body_b->angular_velocity, math3_multiplication_vector3 (rigidbody_effective_inv_inertia (body_b), ang_imp));
}

void revolute_apply_motor (revolute_params *p, rigidbody *body_a, rigidbody *body_b, float dt,
                           const mpe_config_t *cfg) {
    const mpe_config_t *C = cfg ? cfg : &g_cfg;
    (void) dt;
    if ((!p) || (!p->motor_enabled) || (!body_a) || (!body_b)) {
        return;
    }
    if (!isfinite (p->motor_target_speed) || !isfinite (p->motor_max_torque) || p->motor_max_torque <= 0.0f) {
        return;
    }
    /* TRUTH: respect angle limits — driving into a limit fights the limit
     * impulse every tick (jitter + energy). Refuse to drive past stops. */
    if (p->limits_enabled && p->angle_initialized) {
        float along_probe;
        {
            vector3 axw = vector4_rotate_to_vector3 (body_a->orientation, vector3_normalisation (p->axis_a));
            vector3 relw = vector3_subtraction (body_b->angular_velocity, body_a->angular_velocity);
            along_probe = vector3_dot (relw, axw);
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
    vector3 axis_world = vector4_rotate_to_vector3 (body_a->orientation, vector3_normalisation (p->axis_a));
    if (vector3_length_squared (axis_world) < 1e-12f) {
        return;
    }
    axis_world = vector3_normalisation (axis_world);
    vector3 relative_angular = vector3_subtraction (body_b->angular_velocity, body_a->angular_velocity);
    float current_speed = vector3_dot (relative_angular, axis_world);
    if (!isfinite (current_speed)) {
        return;
    }
    float speed_error = p->motor_target_speed - current_speed;
    if (!isfinite (speed_error)) {
        return;
    }
    float motor_gain = C->joints.revolute_motor_gain;
    if (!isfinite (motor_gain) || motor_gain < 0.0f) {
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
    /* Half-budget here, half in the constraint row (see note above). */
    float half_max = 0.5f * p->motor_max_torque;
    if (desired_torque > half_max) {
        desired_torque = half_max;
    }
    if (desired_torque < -half_max) {
        desired_torque = -half_max;
    }
    /* Stability: no overshoot in one tick. */
    {
        float inv_sum = vector3_dot (
            axis_world, math3_multiplication_vector3 (math3_addition (rigidbody_effective_inv_inertia (body_a),
                                                                      rigidbody_effective_inv_inertia (body_b)),
                                                      axis_world));
        if (inv_sum > 1e-12f && dt > 0.0f) {
            float max_no_overshoot = fabsf (speed_error) / (dt * inv_sum);
            if (desired_torque > max_no_overshoot) {
                desired_torque = max_no_overshoot;
            } else if (desired_torque < -max_no_overshoot) {
                desired_torque = -max_no_overshoot;
            }
        }
    }
    if (!isfinite (desired_torque)) {
        return;
    }
    vector3 drive_torque = vector3_scaling (axis_world, desired_torque);
    body_a->torque_accumulator = vector3_subtraction (body_a->torque_accumulator, drive_torque);
    body_b->torque_accumulator = vector3_addition (body_b->torque_accumulator, drive_torque);
}

/* Distance: 1D constraint along the anchor axis. Preserves free rotation and
 * tangential motion; only the separation error is corrected. */
void distance_solve (distance_params *p, rigidbody *body_a, rigidbody *body_b, float dt, const mpe_config_t *cfg) {
    const mpe_config_t *C = cfg ? cfg : &g_cfg;
    if ((!p) || (!body_a) || (!body_b) || (dt <= 0.0f)) {
        return;
    }
    if (!isfinite (p->rest_length) || p->rest_length < 0.0f) {
        return;
    }
    if (a3_joint_solve_may_skip (body_a->is_sleeping, body_b->is_sleeping, 0)) {
        return;
    }
    if (body_a->is_sleeping) {
        rigidbody_wake (body_a);
    }
    if (body_b->is_sleeping) {
        rigidbody_wake (body_b);
    }
    float inv_a = rigidbody_effective_inv_mass (body_a);
    float inv_b = rigidbody_effective_inv_mass (body_b);
    vector3 r_a = vector4_rotate_to_vector3 (body_a->orientation, p->anchor_a);
    vector3 r_b = vector4_rotate_to_vector3 (body_b->orientation, p->anchor_b);
    vector3 world_a = vector3_addition (body_a->position, r_a);
    vector3 world_b = vector3_addition (body_b->position, r_b);
    vector3 delta = vector3_subtraction (world_b, world_a);
    float dist = vector3_length (delta);
    if (dist < 1e-9f) {
        return;
    }
    vector3 n = vector3_scaling (delta, 1.0f / dist);
    float err = dist - p->rest_length;
    vector3 vel_a = vector3_addition (body_a->velocity, vector3_cross (body_a->angular_velocity, r_a));
    vector3 vel_b = vector3_addition (body_b->velocity, vector3_cross (body_b->angular_velocity, r_b));
    float rel_n = vector3_dot (vector3_subtraction (vel_b, vel_a), n);
    float bias = C->joints.revolute_beta * err / dt;
    float max_b = C->joints.revolute_max_bias;
    if (bias > max_b) {
        bias = max_b;
    } else if (bias < -max_b) {
        bias = -max_b;
    }
    vector3 ra_n = vector3_cross (r_a, n);
    vector3 rb_n = vector3_cross (r_b, n);
    float k = inv_a + inv_b +
              vector3_dot (ra_n, math3_multiplication_vector3 (rigidbody_effective_inv_inertia (body_a), ra_n)) +
              vector3_dot (rb_n, math3_multiplication_vector3 (rigidbody_effective_inv_inertia (body_b), rb_n));
    if (k <= 1e-12f) {
        return;
    }
    float lambda = -(rel_n + bias) / k;
    vector3 impulse = vector3_scaling (n, lambda);
    body_a->velocity = vector3_subtraction (body_a->velocity, vector3_scaling (impulse, inv_a));
    body_b->velocity = vector3_addition (body_b->velocity, vector3_scaling (impulse, inv_b));
    body_a->angular_velocity = vector3_subtraction (
        body_a->angular_velocity,
        math3_multiplication_vector3 (rigidbody_effective_inv_inertia (body_a), vector3_cross (r_a, impulse)));
    body_b->angular_velocity = vector3_addition (
        body_b->angular_velocity,
        math3_multiplication_vector3 (rigidbody_effective_inv_inertia (body_b), vector3_cross (r_b, impulse)));
}

/* TRUTH: fixed-weld angular positional correction (once per tick, AFTER the
 * velocity loop — never inside, same energy-pump rule as revolute axis
 * drift). Velocity-only lock kills relative spin but lets orientation error
 * integrate (weld flexes over seconds). This Baumgarte drives the relative
 * quaternion error q_err = q_a^-1 * q_b toward identity with angular impulse,
 * proportional to the rotation vector (axis*sin(half-angle) scaled). */
void fixed_correct_angular_drift (fixed_params *p, rigidbody *body_a, rigidbody *body_b, float dt,
                                  const mpe_config_t *cfg) {
    (void) cfg;
    (void) p;
    if ((!body_a) || (!body_b) || (dt <= 0.0f)) {
        return;
    }
    vector4 q_a_inv =
        (vector4){body_a->orientation.w, -body_a->orientation.x, -body_a->orientation.y, -body_a->orientation.z};
    vector4 q_err = vector4_multiplication (q_a_inv, body_b->orientation);
    q_err = vector4_normalisation (q_err);
    /* Shortest-arc: w<0 flips to the complementary rotation. */
    if (q_err.w < 0.0f) {
        q_err.w = -q_err.w;
        q_err.x = -q_err.x;
        q_err.y = -q_err.y;
        q_err.z = -q_err.z;
    }
    vector3 err_vec = {q_err.x, q_err.y, q_err.z};
    if (vector3_length_squared (err_vec) < 1e-12f) {
        return;
    }
    const float beta = 0.1f;
    /* Correction angular velocity in A-frame, rotated to world via A. */
    vector3 corr_local = vector3_scaling (err_vec, 2.0f * beta / dt);
    vector3 corr_world = vector4_rotate_to_vector3 (body_a->orientation, corr_local);
    math3 m = math3_addition (rigidbody_effective_inv_inertia (body_a), rigidbody_effective_inv_inertia (body_b));
    /* Guard near-singular (both infinite mass): nothing to correct. */
    math3 m_inv = math3_inverse (m);
    vector3 impulse = vector3_scaling (math3_multiplication_vector3 (m_inv, corr_world), -1.0f);
    /* FIX-AUDIT-DESPOT: gate on EFFECTIVE inv inertia (>0), not just
     * !static_state. static_state misses sleeping/kinematic (effective 0):
     * the old gate applied drift impulses to bodies the velocity solve
     * treats as immovable, waking sleepers every tick and fighting
     * prescribed kinematic motion. Zero-effective sides skip. */
    if (rigidbody_effective_inv_mass (body_a) > 0.0f) {
        body_a->angular_velocity = vector3_subtraction (
            body_a->angular_velocity, math3_multiplication_vector3 (rigidbody_effective_inv_inertia (body_a), impulse));
    }
    if (rigidbody_effective_inv_mass (body_b) > 0.0f) {
        body_b->angular_velocity = vector3_addition (
            body_b->angular_velocity, math3_multiplication_vector3 (rigidbody_effective_inv_inertia (body_b), impulse));
    }
}

/* Positional axis-drift correction: MUST be called exactly once per tick,
 * AFTER the velocity iteration loop — never inside it. The error term is
 * positional (orientation difference, unchanged by velocity iterations),
 * so per-iteration application multiplies the correction by the iteration
 * count (64x at defaults): a spurious torsional spring that pumps energy
 * and destroys hinge truth (e.g. 9x-too-fast pendulum). */
void revolute_correct_axis_drift (revolute_params *p, rigidbody *body_a, rigidbody *body_b, float dt,
                                  const mpe_config_t *cfg) {
    (void) cfg;
    if ((!p) || (!body_a) || (!body_b) || (dt <= 0.0f)) {
        return;
    }
    vector3 hinge_b = (vector3_length_squared (p->axis_b) > 1e-12f) ? vector3_normalisation (p->axis_b)
                                                                    : vector3_normalisation (p->axis_a);
    /* ---- axis drift correction: positional Baumgarte to keep hinge axes aligned ---- */
    vector3 axis_a_world = vector4_rotate_to_vector3 (body_a->orientation, vector3_normalisation (p->axis_a));
    vector3 axis_b_world = vector4_rotate_to_vector3 (body_b->orientation, hinge_b);
    /* Axis error: cross product gives rotation vector needed to align axis_b with axis_a.
     * Magnitude is sin(angle) ≈ angle for small angles. Direction is the rotation axis. */
    vector3 axis_error = vector3_cross (axis_a_world, axis_b_world);
    float axis_error_len_sq = vector3_length_squared (axis_error);
    if (axis_error_len_sq > 0.000001f) {
        /* Baumgarte stabilization: apply angular velocity correction proportional to axis_error */
        const float axis_baumgarte_beta = 0.1f; /* MFS_127: reduced from 0.2 to reduce oscillation */
        vector3 axis_correction = vector3_scaling (axis_error, axis_baumgarte_beta / dt);
        /* Compute effective angular mass for the correction */
        math3 drift_angular_mass =
            math3_addition (rigidbody_effective_inv_inertia (body_a), rigidbody_effective_inv_inertia (body_b));
        math3 drift_angular_mass_inv = math3_inverse (drift_angular_mass);
        vector3 axis_impulse =
            vector3_scaling (math3_multiplication_vector3 (drift_angular_mass_inv, axis_correction), -1.0f);
        /* Apply angular impulse to both bodies.
         * FIX-AUDIT-DESPOT: gate on EFFECTIVE inv mass (>0), not just
         * !static_state (same sleeping/kinematic hole as the fixed-weld
         * drift above: effective helpers already zero those sides, so the
         * impulse application must skip them or sleepers get kicked). */
        if (rigidbody_effective_inv_mass (body_a) > 0.0f) {
            body_a->angular_velocity = vector3_subtraction (
                body_a->angular_velocity,
                math3_multiplication_vector3 (rigidbody_effective_inv_inertia (body_a), axis_impulse));
        }
        if (rigidbody_effective_inv_mass (body_b) > 0.0f) {
            body_b->angular_velocity = vector3_addition (
                body_b->angular_velocity,
                math3_multiplication_vector3 (rigidbody_effective_inv_inertia (body_b), axis_impulse));
        }
    }
    /* NOTE (DESPOT-2026-10-01): a positional limit re-seat (Baumgarte drive
     * of escaped twist back into range, same pattern as above) was tried and
     * REVERTED: it loses a tug-of-war with the P2P anchor bias (which keeps
     * rotating the arm with angular parts of its own correction), so the
     * violation never shrinks no matter the gain, and escalating the gain
     * toward oscillation is not a trade worth making blind. Velocity-level
     * enforcement on live (never-frozen) books is what ships; static
     * rest-past-stop under a starved solver stays a ticketed limit
     * (see REMAINING_WORK). */
}