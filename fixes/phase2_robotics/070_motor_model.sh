#!/usr/bin/env bash
# ============================================================
# FIX 070 — FTC Phase 2: DC motor electrical model
#   Implements the pipeline from the development plan:
#     BackEMF = Kv * omega
#     Current = (V_applied - BackEMF) / R
#     Torque  = Kt * Current * efficiency
#     Output  = Torque * gear_ratio
#   With stall-current clamping and thermal accumulation.
# Phase:   phase2_robotics
# Files:   v15R2/src/robotics/motor.h, motor.c (new)
# Depends: none (physics_world from Phase 0)
# Risk:    low (new files only)
# ============================================================
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"
DIR="v15R2/src/robotics"
H="$DIR/motor.h"
C="$DIR/motor.c"
grep -q 'MPE_FTC_070' "$C" 2>/dev/null && { echo "[SKIP] motor model already present"; exit 0; }
mkdir -p "$DIR"

cat > "$H" <<'EOF'
/* MPE_FTC_070: DC motor electrical model */
#ifndef motor_h
#define motor_h

typedef struct {
    /* Electrical (derive from spec sheet: stall_torque, free_speed, stall_current) */
    float resistance;          /* ohms */
    float kt;                  /* N·m/A torque constant */
    float kv;                  /* V/(rad/s) back-EMF constant */
    float stall_current;       /* A */
    float free_speed_rad_s;    /* rad/s at no load */

    /* Mechanical */
    float gear_ratio;          /* output/input */
    float efficiency;          /* 0..1 */

    /* Live state */
    float command;             /* -1..1 from controller */
    float current;             /* A (computed each tick) */
    float back_emf;            /* V (computed each tick) */
    float torque;              /* N·m at motor shaft */
    float output_torque;       /* N·m at wheel after gearing */
    float rpm;                 /* current output speed */
    float temperature;         /* simplified thermal model */
} motor;

/* Derive motor params from the four spec-sheet numbers. */
void motor_from_spec (motor *m,
                      float stall_torque_nm,
                      float free_speed_rpm,
                      float stall_current_a,
                      float nominal_voltage,
                      float gear_ratio,
                      float efficiency);

/* Advance one tick. wheel_angular_vel = output shaft speed (rad/s). */
void motor_update (motor *m, float wheel_angular_vel, float dt, float battery_voltage);

#endif /* motor_h */
EOF

cat > "$C" <<'EOF'
/* MPE_FTC_070: DC motor electrical model implementation */
#include "motor.h"
#include <math.h>

#define MOTOR_RPM_TO_RAD_S 0.10472f  /* 2*pi/60 */

void motor_from_spec (motor *m,
                      float stall_torque_nm,
                      float free_speed_rpm,
                      float stall_current_a,
                      float nominal_voltage,
                      float gear_ratio,
                      float efficiency) {
    if (!m) {return;}
    m->stall_current = stall_current_a;
    m->free_speed_rad_s = free_speed_rpm * MOTOR_RPM_TO_RAD_S;
    m->gear_ratio = (gear_ratio > 0.0f) ? gear_ratio : 1.0f;
    m->efficiency = (efficiency > 0.0f && efficiency <= 1.0f) ? efficiency : 0.85f;

    /* Kt = stall_torque / stall_current */
    m->kt = (stall_current_a > 0.0f) ? (stall_torque_nm / stall_current_a) : 0.0f;

    /* R = V_nominal / stall_current */
    m->resistance = (stall_current_a > 0.0f) ? (nominal_voltage / stall_current_a) : 1.0f;

    /* Kv: at free speed, current ~ 0, so BackEMF ~ V_nominal */
    /* Kv = V / omega_free  (motor shaft, before gearing) */
    float motor_free_speed = m->free_speed_rad_s * m->gear_ratio;
    m->kv = (motor_free_speed > 0.0f) ? (nominal_voltage / motor_free_speed) : 0.0f;

    m->command = 0.0f;
    m->current = 0.0f;
    m->back_emf = 0.0f;
    m->torque = 0.0f;
    m->output_torque = 0.0f;
    m->rpm = 0.0f;
    m->temperature = 25.0f;
}

void motor_update (motor *m, float wheel_angular_vel, float dt, float battery_voltage) {
    if ((!m) || (dt <= 0.0f)) {return;}

    float motor_shaft_vel = wheel_angular_vel * m->gear_ratio;

    /* BackEMF opposes applied voltage */
    m->back_emf = m->kv * motor_shaft_vel;

    /* Applied voltage from command */
    float applied_voltage = battery_voltage * m->command;

    /* Current = (V - BackEMF) / R, clamped to stall */
    float raw_current = (applied_voltage - m->back_emf) / m->resistance;
    if (raw_current > m->stall_current) {raw_current = m->stall_current;}
    if (raw_current < -m->stall_current) {raw_current = -m->stall_current;}
    m->current = raw_current;

    /* Torque = Kt * I * efficiency */
    m->torque = m->kt * m->current * m->efficiency;

    /* Output torque at wheel (after gearing) */
    m->output_torque = m->torque * m->gear_ratio;

    /* Speed tracking */
    m->rpm = fabsf (wheel_angular_vel) / MOTOR_RPM_TO_RAD_S;

    /* Simplified thermal: heat from I^2*R, cooling to ambient */
    float heat_generated = m->current * m->current * m->resistance * dt;
    float cooling = (m->temperature - 25.0f) * 0.01f * dt;
    m->temperature += heat_generated * 0.1f - cooling;
    if (m->temperature < 25.0f) {m->temperature = 25.0f;}
}
EOF

grep -q 'motor_update' "$H" || { echo "[FAIL] motor.h not written"; exit 1; }
grep -q 'motor_from_spec' "$C" || { echo "[FAIL] motor.c not written"; exit 1; }
echo "[PASS] 070: DC motor electrical model added"
