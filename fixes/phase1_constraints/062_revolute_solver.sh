#!/usr/bin/env bash
# ============================================================
# FIX 062 — PHYS-003: revolute (hinge) constraint solver
#   Point-to-point anchor constraint (3 DOF) via a 3x3 effective-mass
#   impulse with Baumgarte positional bias, plus hinge-axis alignment
#   (removes relative angular velocity perpendicular to the axis) and
#   a torque-accumulator motor. Limits are deferred (need persistent
#   relative-angle state).
# Phase:   phase1_constraints
# Files:   v15R3/src/physics/revolute_joint.h, revolute_joint.c (new)
# Depends: 060, 061
# Risk:    low (new files only)
# ============================================================
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"
H="v15R3/src/physics/revolute_joint.h"
C="v15R3/src/physics/revolute_joint.c"
grep -q 'MPE_FTC_062' "$C" 2>/dev/null && { echo "[SKIP] revolute solver already present"; exit 0; }
[[ -f "v15R3/src/physics/constraint.h" ]] || { echo "[SKIP] constraint.h missing (run 060)"; exit 0; }

cat > "$H" <<'EOF'
/* MPE_FTC_062 */
#ifndef revolute_joint_h
#define revolute_joint_h
#include "constraint.h"
/* Iterative positional/axis solve (call once per tick). */
void revolute_solve (revolute_params *p, rigidbody *body_a, rigidbody *body_b, float dt);
/* Motor: adds drive torque to the torque accumulator (call once per tick). */
void revolute_apply_motor (revolute_params *p, rigidbody *body_a, rigidbody *body_b, float dt);
#endif
EOF

cat > "$C" <<'EOF'
/* MPE_FTC_062: revolute (hinge) constraint solver.
 *
 * revolute_solve() enforces, in one pass per tick:
 *   1. point-to-point: the two anchors coincide (3 DOF removed), solved
 *      with a 3x3 effective-mass impulse + Baumgarte positional bias.
 *   2. axis alignment: relative angular velocity perpendicular to the
 *      hinge axis is removed (2 DOF removed), leaving spin about the axis.
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
#include <math.h>

static math3 skew_symmetric (vector3 v) {
    math3 m = {{{0.0f}}};
    m.matrix [0][1] = -v.z;  m.matrix [0][2] =  v.y;
    m.matrix [1][0] =  v.z;  m.matrix [1][2] = -v.x;
    m.matrix [2][0] = -v.y;  m.matrix [2][1] =  v.x;
    return m;
}

static math3 math3_addition (math3 a, math3 b) {
    math3 r;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) { r.matrix [i][j] = a.matrix [i][j] + b.matrix [i][j]; }
    }
    return r;
}

void revolute_solve (revolute_params *p, rigidbody *body_a, rigidbody *body_b, float dt) {
    if ((!p) || (!body_a) || (!body_b) || (dt <= 0.0f)) {return;}
    /* Jointed bodies stay awake so the constraint always acts. */
    if (body_a->is_sleeping) {rigidbody_wake (body_a);}
    if (body_b->is_sleeping) {rigidbody_wake (body_b);}
    float inv_mass_a = body_a->static_state ? 0.0f : body_a->inverse_mass;
    float inv_mass_b = body_b->static_state ? 0.0f : body_b->inverse_mass;
    if ((inv_mass_a <= 0.0f) && (inv_mass_b <= 0.0f)) {return;}

    vector3 r_a = vector4_rotate_to_vector3 (body_a->orientation, p->anchor_a);
    vector3 r_b = vector4_rotate_to_vector3 (body_b->orientation, p->anchor_b);

    /* ---- point-to-point ---- */
    vector3 anchor_a_world = vector3_addition (body_a->position, r_a);
    vector3 anchor_b_world = vector3_addition (body_b->position, r_b);
    vector3 position_error = vector3_subtraction (anchor_b_world, anchor_a_world);

    vector3 vel_a_at_anchor = vector3_addition (body_a->velocity, vector3_cross (body_a->angular_velocity, r_a));
    vector3 vel_b_at_anchor = vector3_addition (body_b->velocity, vector3_cross (body_b->angular_velocity, r_b));
    vector3 relative_velocity = vector3_subtraction (vel_b_at_anchor, vel_a_at_anchor);

    const float baumgarte_beta = 0.3f;
    vector3 bias = vector3_scaling (position_error, baumgarte_beta / dt);

    float inv_mass_sum = inv_mass_a + inv_mass_b;
    math3 K = {{{0.0f}}};
    for (int i = 0; i < 3; i++) { K.matrix [i][i] = inv_mass_sum; }
    math3 skew_a = skew_symmetric (r_a);
    math3 skew_b = skew_symmetric (r_b);
    /* K = inv_mass_sum*I - skew(r_a)*Ia^-1*skew(r_a) - skew(r_b)*Ib^-1*skew(r_b)
     * (the subtracted terms are positive semi-definite, so K stays SPD) */
    math3 term_a = math3_multiplication (skew_a, math3_multiplication (body_a->inverse_inertia_system, skew_a));
    math3 term_b = math3_multiplication (skew_b, math3_multiplication (body_b->inverse_inertia_system, skew_b));
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            K.matrix [i][j] -= term_a.matrix [i][j];
            K.matrix [i][j] -= term_b.matrix [i][j];
        }
    }
    math3 K_inv = math3_inverse (K);
    vector3 rhs = vector3_scaling (vector3_addition (relative_velocity, bias), -1.0f);
    vector3 impulse = math3_multiplication_vector3 (K_inv, rhs);

    body_a->velocity = vector3_subtraction (body_a->velocity, vector3_scaling (impulse, inv_mass_a));
    body_b->velocity = vector3_addition (body_b->velocity, vector3_scaling (impulse, inv_mass_b));
    body_a->angular_velocity = vector3_subtraction (body_a->angular_velocity,
        math3_multiplication_vector3 (body_a->inverse_inertia_system, vector3_cross (r_a, impulse)));
    body_b->angular_velocity = vector3_addition (body_b->angular_velocity,
        math3_multiplication_vector3 (body_b->inverse_inertia_system, vector3_cross (r_b, impulse)));

    /* ---- axis alignment: kill relative angular velocity off the hinge ---- */
    vector3 axis_world = vector4_rotate_to_vector3 (body_a->orientation, vector3_normalisation (p->axis_a));
    vector3 relative_angular = vector3_subtraction (body_b->angular_velocity, body_a->angular_velocity);
    float along_axis = vector3_dot (relative_angular, axis_world);
    vector3 perpendicular_angular = vector3_subtraction (relative_angular, vector3_scaling (axis_world, along_axis));

    math3 angular_mass = math3_addition (body_a->inverse_inertia_system, body_b->inverse_inertia_system);
    math3 angular_mass_inv = math3_inverse (angular_mass);
    vector3 angular_impulse = vector3_scaling (math3_multiplication_vector3 (angular_mass_inv, perpendicular_angular), -1.0f);
    body_a->angular_velocity = vector3_subtraction (body_a->angular_velocity,
        math3_multiplication_vector3 (body_a->inverse_inertia_system, angular_impulse));
    body_b->angular_velocity = vector3_addition (body_b->angular_velocity,
        math3_multiplication_vector3 (body_b->inverse_inertia_system, angular_impulse));
}

void revolute_apply_motor (revolute_params *p, rigidbody *body_a, rigidbody *body_b, float dt) {
    (void) dt;
    if ((!p) || (!p->motor_enabled) || (!body_a) || (!body_b)) {return;}
    vector3 axis_world = vector4_rotate_to_vector3 (body_a->orientation, vector3_normalisation (p->axis_a));
    vector3 relative_angular = vector3_subtraction (body_b->angular_velocity, body_a->angular_velocity);
    float current_speed = vector3_dot (relative_angular, axis_world);
    float speed_error = p->motor_target_speed - current_speed;
    const float motor_gain = 8.0f; /* proportional gain; promote to config later */
    float desired_torque = speed_error * motor_gain;
    if (desired_torque > p->motor_max_torque) {desired_torque = p->motor_max_torque;}
    if (desired_torque < -p->motor_max_torque) {desired_torque = -p->motor_max_torque;}
    vector3 drive_torque = vector3_scaling (axis_world, desired_torque);
    body_a->torque_accumulator = vector3_subtraction (body_a->torque_accumulator, drive_torque);
    body_b->torque_accumulator = vector3_addition (body_b->torque_accumulator, drive_torque);
}
EOF

grep -q 'revolute_solve' "$H" || { echo "[FAIL] header not written"; exit 1; }
grep -q 'revolute_apply_motor' "$C" || { echo "[FAIL] motor not written"; exit 1; }
echo "[PASS] 062: revolute solver (point-to-point + axis + motor) added"
