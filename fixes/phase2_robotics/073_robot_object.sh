#!/usr/bin/env bash
# ============================================================
# FIX 073 — FTC Phase 2: FTC robot object
#   Chassis box + 4 wheel spheres connected via revolute joints
#   with motor-driven axles. Uses physics_world from Phase 0
#   and constraint framework from Phase 1.
# Phase:   phase2_robotics
# Files:   v15R2/src/robotics/robot.h, robot.c (new)
# Depends: 070, 071, 072, 063 (constraint dispatch)
# Risk:    low (new files only)
# ============================================================
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"
DIR="v15R2/src/robotics"
H="$DIR/robot.h"
C="$DIR/robot.c"
grep -q 'MPE_FTC_073' "$C" 2>/dev/null && { echo "[SKIP] robot object already present"; exit 0; }
[[ -f "$DIR/motor.h" ]] || { echo "[SKIP] motor.h missing (run 070 first)"; exit 0; }
[[ -f "$DIR/battery.h" ]] || { echo "[SKIP] battery.h missing (run 072 first)"; exit 0; }
[[ -f "v15R2/src/physics/constraint.h" ]] || { echo "[SKIP] constraint.h missing (run 060 first)"; exit 0; }

cat > "$H" <<'EOF'
/* MPE_FTC_073: FTC robot object */
#ifndef robot_h
#define robot_h

#include "motor.h"
#include "motor_presets.h"
#include "battery.h"
#include "../core/physics_world.h"

#define FTC_MAX_WHEELS 8

typedef struct {
    /* Body indices in physics_world */
    int chassis_body;
    int wheel_bodies [FTC_MAX_WHEELS];
    int wheel_joints [FTC_MAX_WHEELS];  /* revolute joint indices */
    int wheel_count;

    /* Motor + electrical */
    motor wheel_motors [FTC_MAX_WHEELS];
    motor_preset_id motor_preset;
    battery battery;

    /* Axle direction in chassis-local space (for reading wheel speed) */
    float axle_axis_x, axle_axis_y, axle_axis_z;
} ftc_robot;

/* Create a 4-wheel robot at the given position. Returns 0 on success. */
int ftc_robot_create (physics_world *world, ftc_robot *robot,
                      float x, float y, float z,
                      motor_preset_id preset);

/* Update all motors for one tick. Reads wheel angular velocity,
   computes electrical state, applies torque to wheel bodies. */
void ftc_robot_update (physics_world *world, ftc_robot *robot, float dt);

/* Set wheel motor commands (-1..1). */
void ftc_robot_set_wheel_commands (ftc_robot *robot, const float *commands, int count);

/* Get the chassis body's position (for validation). */
void ftc_robot_get_position (physics_world *world, ftc_robot *robot,
                             float *px, float *py, float *pz);

#endif /* robot_h */
EOF

cat > "$C" <<'EOF'
/* MPE_FTC_073: FTC robot object implementation */
#include "robot.h"
#include "../physics/constraint.h"
#include <math.h>
#include <string.h>

/* Robot dimensions (metres, approximate FTC 18" x 18" chassis) */
#define CHASSIS_HALF_X 0.225f
#define CHASSIS_HALF_Y 0.075f
#define CHASSIS_HALF_Z 0.225f
#define CHASSIS_MASS   8.0f   /* ~18 lb robot */
#define WHEEL_RADIUS   0.05f  /* 100mm wheels */
#define WHEEL_MASS     0.2f
#define WHEEL_OFFSET_X 0.24f  /* slightly outside chassis */
#define WHEEL_OFFSET_Z 0.20f
#define WHEEL_Y_OFFSET (-CHASSIS_HALF_Y - WHEEL_RADIUS + 0.01f)

int ftc_robot_create (physics_world *world, ftc_robot *robot,
                      float x, float y, float z,
                      motor_preset_id preset) {
    if ((!world) || (!robot)) {return 1;}
    memset (robot, 0, sizeof (ftc_robot));
    robot->motor_preset = preset;
    robot->axle_axis_x = 1.0f;  /* axles point along X (left-right) */
    robot->axle_axis_y = 0.0f;
    robot->axle_axis_z = 0.0f;
    battery_init (&robot->battery);

    /* Chassis: a box at the given position */
    robot->chassis_body = physics_world_add_cube (
        world,
        (vector3) {x, y, z},
        (vector3) {CHASSIS_HALF_X, CHASSIS_HALF_Y, CHASSIS_HALF_Z},
        CHASSIS_MASS
    );
    if (robot->chassis_body < 0) {return 1;}

    uint32_t chassis_id = world->bodies [robot->chassis_body].object_id;

    /* 4 wheels at corners */
    float wheel_positions [4][3] = {
        {x - WHEEL_OFFSET_X, y + WHEEL_Y_OFFSET, z - WHEEL_OFFSET_Z},  /* front-left */
        {x + WHEEL_OFFSET_X, y + WHEEL_Y_OFFSET, z - WHEEL_OFFSET_Z},  /* front-right */
        {x - WHEEL_OFFSET_X, y + WHEEL_Y_OFFSET, z + WHEEL_OFFSET_Z},  /* back-left */
        {x + WHEEL_OFFSET_X, y + WHEEL_Y_OFFSET, z + WHEEL_OFFSET_Z},  /* back-right */
    };
    robot->wheel_count = 4;

    for (int i = 0; i < robot->wheel_count; i++) {
        /* Create wheel as a sphere (rolling approximation) */
        robot->wheel_bodies [i] = physics_world_add_sphere (
            world,
            WHEEL_RADIUS,
            WHEEL_MASS,
            (vector3) {wheel_positions [i][0], wheel_positions [i][1], wheel_positions [i][2]}
        );
        if (robot->wheel_bodies [i] < 0) {return 1;}

        uint32_t wheel_id = world->bodies [robot->wheel_bodies [i]].object_id;

        /* Revolute joint: chassis (body_a) to wheel (body_b), axle along X */
        vector3 anchor_on_chassis = {
            wheel_positions [i][0] - x,
            WHEEL_Y_OFFSET,
            wheel_positions [i][2] - z
        };
        vector3 anchor_on_wheel = {0.0f, 0.0f, 0.0f};  /* wheel centre */
        vector3 axle_axis = {robot->axle_axis_x, robot->axle_axis_y, robot->axle_axis_z};

        robot->wheel_joints [i] = constraint_add_revolute (
            chassis_id, wheel_id,
            anchor_on_chassis, anchor_on_wheel, axle_axis
        );
        if (robot->wheel_joints [i] < 0) {return 1;}

        /* Set up motor for this wheel */
        motor_preset_apply (&robot->wheel_motors [i], preset);
    }

    return 0;
}

void ftc_robot_update (physics_world *world, ftc_robot *robot, float dt) {
    if ((!world) || (!robot) || (dt <= 0.0f)) {return;}

    /* Sum currents for battery sag */
    float total_current = 0.0f;
    for (int i = 0; i < robot->wheel_count; i++) {
        total_current += fabsf (robot->wheel_motors [i].current);
    }
    float terminal_voltage = battery_get_voltage (&robot->battery, total_current);
    battery_drain (&robot->battery, total_current, dt);

    /* Update each wheel motor */
    for (int i = 0; i < robot->wheel_count; i++) {
        int wheel_idx = robot->wheel_bodies [i];
        if ((wheel_idx < 0) || (wheel_idx >= world->body_count)) {continue;}
        rigidbody *wheel = &world->bodies [wheel_idx];

        /* Read wheel angular velocity about the axle axis */
        vector3 axle = {robot->axle_axis_x, robot->axle_axis_y, robot->axle_axis_z};
        float wheel_speed = wheel->angular_velocity.x * axle.x
                          + wheel->angular_velocity.y * axle.y
                          + wheel->angular_velocity.z * axle.z;

        /* Update motor electrical state */
        motor_update (&robot->wheel_motors [i], wheel_speed, dt, terminal_voltage);

        /* Apply motor torque to wheel body */
        float torque = robot->wheel_motors [i].output_torque;
        wheel->torque_accumulator.x += axle.x * torque;
        wheel->torque_accumulator.y += axle.y * torque;
        wheel->torque_accumulator.z += axle.z * torque;
    }
}

void ftc_robot_set_wheel_commands (ftc_robot *robot, const float *commands, int count) {
    if (!robot) {return;}
    int n = (count < robot->wheel_count) ? count : robot->wheel_count;
    for (int i = 0; i < n; i++) {
        float cmd = commands [i];
        if (cmd > 1.0f) {cmd = 1.0f;}
        if (cmd < -1.0f) {cmd = -1.0f;}
        robot->wheel_motors [i].command = cmd;
    }
}

void ftc_robot_get_position (physics_world *world, ftc_robot *robot,
                             float *px, float *py, float *pz) {
    if ((!world) || (!robot)) {return;}
    int idx = robot->chassis_body;
    if ((idx < 0) || (idx >= world->body_count)) {return;}
    if (px) {*px = world->bodies [idx].position.x;}
    if (py) {*py = world->bodies [idx].position.y;}
    if (pz) {*pz = world->bodies [idx].position.z;}
}
EOF

grep -q 'ftc_robot_create' "$H" || { echo "[FAIL] robot.h not written"; exit 1; }
grep -q 'ftc_robot_update' "$C" || { echo "[FAIL] robot.c not written"; exit 1; }
echo "[PASS] 073: FTC robot object added (chassis + 4 wheels + revolute joints)"
