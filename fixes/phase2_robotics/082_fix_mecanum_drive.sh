#!/usr/bin/env bash
# ============================================================
# FIX 082 — Repair drivetrain.c syntax error + implement real
#   mecanum chassis forces.
#
#   Root cause 1: The 075 awk script left a stray '}' at line 56
#   of drivetrain.c, causing a compile error.
#
#   Root cause 2: Mecanum strafe doesn't work because spherical
#   wheels can't produce lateral traction. The IK only sets motor
#   commands (which spin the wheels) but doesn't apply lateral
#   forces to the chassis. Fix: compute a chassis-level force
#   from the IK and apply it directly in drivetrain_update.
#
# Phase:   phase2_robotics
# Files:   v15R2/src/robotics/robot.h (add mecanum fields)
#          v15R2/src/robotics/drivetrain.c (fix syntax + set forces)
#          v15R2/src/robotics/robot.c (init new fields)
# Depends: 075, 076
# Risk:    medium (three targeted edits)
# ============================================================
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"

ROBOT_H="v15R2/src/robotics/robot.h"
DRIVETRAIN_C="v15R2/src/robotics/drivetrain.c"
ROBOT_C="v15R2/src/robotics/robot.c"

for f in "$ROBOT_H" "$DRIVETRAIN_C" "$ROBOT_C"; do
    [[ -f "$f" ]] || { echo "[SKIP] $f not found"; exit 0; }
done

grep -q 'MPE_FTC_082' "$DRIVETRAIN_C" && { echo "[SKIP] 082 already applied"; exit 0; }

cp "$ROBOT_H"    "${ROBOT_H}.pre_082"
cp "$DRIVETRAIN_C" "${DRIVETRAIN_C}.pre_082"
cp "$ROBOT_C"    "${ROBOT_C}.pre_082"

# ============================================================
# STEP 1: Add mecanum force fields to ftc_robot struct in robot.h
# ============================================================
sed -i '/float axle_axis_x, axle_axis_y, axle_axis_z;/a\    /* MPE_FTC_082: mecanum chassis-force fields */\n    vector3 mecanum_chassis_force;\n    float mecanum_chassis_torque;\n    bool mecanum_active;' "$ROBOT_H"

grep -q 'mecanum_chassis_force' "$ROBOT_H" \
    || { echo "[FAIL] mecanum fields not added to robot.h"; exit 1; }

# ============================================================
# STEP 2: Rewrite drivetrain.c — fix syntax + implement forces
# ============================================================
cat > "$DRIVETRAIN_C" << 'DRIVETRAIN_EOF'
/* MPE_FTC_074: Drivetrain implementation */
/* MPE_FTC_082: Fixed syntax error (stray '}') + real mecanum chassis forces */
#include "drivetrain.h"
#include "../core/math3D.h"

void drivetrain_tank (ftc_robot *robot, float left_power, float right_power) {
    if (!robot) {return;}
    if (left_power > 1.0f) {left_power = 1.0f;}
    if (left_power < -1.0f) {left_power = -1.0f;}
    if (right_power > 1.0f) {right_power = 1.0f;}
    if (right_power < -1.0f) {right_power = -1.0f;}
    /* Wheel layout: [0]=front-left, [1]=front-right, [2]=back-left, [3]=back-right */
    float commands [FTC_MAX_WHEELS];
    for (int i = 0; i < robot->wheel_count; i++) {
        bool is_left = (i % 2 == 0);  /* 0,2 = left; 1,3 = right */
        commands [i] = is_left ? left_power : right_power;
    }
    ftc_robot_set_wheel_commands (robot, commands, robot->wheel_count);
    robot->mecanum_active = false; /* MPE_FTC_082 */
}

/* MPE_FTC_075 + MPE_FTC_082: Mecanum drive with inverse kinematics
 *
 * Since the wheel model uses spheres (no natural rolling direction),
 * mecanum strafe cannot work through wheel friction alone. We set
 * per-wheel motor commands for forward drive (which the wheel_traction
 * raycast converts to forward force), AND we compute a direct chassis
 * force for the strafe/rotate components. drivetrain_update() applies
 * that chassis force after ftc_robot_update(). */
void drivetrain_mecanum (ftc_robot *robot, float forward, float strafe, float rotate) {
    if (!robot) {return;}
    /* Clamp inputs */
    if (forward > 1.0f) {forward = 1.0f;}
    if (forward < -1.0f) {forward = -1.0f;}
    if (strafe > 1.0f) {strafe = 1.0f;}
    if (strafe < -1.0f) {strafe = -1.0f;}
    if (rotate > 1.0f) {rotate = 1.0f;}
    if (rotate < -1.0f) {rotate = -1.0f;}

    /* Mecanum IK: per-wheel velocity targets
       Wheel layout: [0]=FL, [1]=FR, [2]=BL, [3]=BR
       FL: forward + strafe - rotate
       FR: forward - strafe + rotate
       BL: forward - strafe - rotate
       BR: forward + strafe + rotate */
    float wheel_targets [4];
    wheel_targets [0] = forward + strafe - rotate;
    wheel_targets [1] = forward - strafe + rotate;
    wheel_targets [2] = forward - strafe - rotate;
    wheel_targets [3] = forward + strafe + rotate;

    /* Normalize if any target exceeds 1.0 */
    float max_mag = 0.0f;
    for (int i = 0; i < 4; i++) {
        float mag = fabsf (wheel_targets [i]);
        if (mag > max_mag) {max_mag = mag;}
    }
    if (max_mag > 1.0f) {
        for (int i = 0; i < 4; i++) {wheel_targets [i] /= max_mag;}
    }

    /* Set motor commands (forward component uses wheel traction) */
    ftc_robot_set_wheel_commands (robot, wheel_targets, 4);

    /* MPE_FTC_082: Compute direct chassis force for strafe + rotate.
     * Local space: X = lateral (strafe), Y = up, Z = forward.
     * Force scale: tuned so full input ≈ 80 N, enough to move an 8 kg
     * chassis at ~0.5 m/s² against friction. */
    const float force_scale = 80.0f;   /* N per unit input */
    const float torque_scale = 8.0f;   /* N·m per unit input */
    robot->mecanum_chassis_force = (vector3) {
        strafe * force_scale,
        0.0f,
        forward * force_scale * 0.5f   /* forward partly via wheels */
    };
    robot->mecanum_chassis_torque = rotate * torque_scale;
    robot->mecanum_active = true;
}

void drivetrain_update (physics_world *world, ftc_robot *robot, float dt) {
    if ((!world) || (!robot) || (dt <= 0.0f)) {return;}
    ftc_robot_update (world, robot, dt);

    /* MPE_FTC_082: Apply mecanum chassis forces after motor update */
    if (robot->mecanum_active) {
        int idx = robot->chassis_body;
        if ((idx >= 0) && (idx < world->body_count)) {
            rigidbody *chassis = &world->bodies [idx];
            /* Transform local force to world space using chassis orientation */
            vector3 world_force = vector4_rotate_to_vector3 (
                chassis->orientation, robot->mecanum_chassis_force);
            rb_apply_forces_perfect (chassis, world_force);
            /* Yaw torque (around local Y axis) */
            vector3 local_torque = {0.0f, robot->mecanum_chassis_torque, 0.0f};
            vector3 world_torque = vector4_rotate_to_vector3 (
                chassis->orientation, local_torque);
            chassis->torque_accumulator = vector3_addition (
                chassis->torque_accumulator, world_torque);
            rigidbody_wake (chassis);
        }
        robot->mecanum_active = false;
    }
}
DRIVETRAIN_EOF

grep -q 'MPE_FTC_082' "$DRIVETRAIN_C" \
    || { echo "[FAIL] drivetrain.c not rewritten"; exit 1; }

# ============================================================
# STEP 3: Initialize new fields in robot.c (ftc_robot_create)
# ============================================================
# The memset(robot, 0, ...) in ftc_robot_create already zeros everything,
# so mecanum_active=false and mecanum_chassis_force={0,0,0} are handled.
# Just verify the memset is present.
if ! grep -q 'memset (robot, 0, sizeof (ftc_robot));' "$ROBOT_C"; then
    echo "[WARN] memset not found in ftc_robot_create — adding defensive init"
    sed -i '/robot->motor_preset = preset;/a\    robot->mecanum_active = false; /* MPE_FTC_082 */' "$ROBOT_C"
fi

# ============================================================
# STEP 4: Rebuild and run the mecanum test
# ============================================================
cd v15R2/src
if make test_mecanum_drive > /tmp/mecanum_test_082.log 2>&1; then
    tail -6 /tmp/mecanum_test_082.log
    echo "[PASS] 082: mecanum syntax fixed + chassis forces applied; test passes"
else
    tail -12 /tmp/mecanum_test_082.log
    echo "[FAIL] 082: mecanum test still failing after fix"
    exit 1
fi
