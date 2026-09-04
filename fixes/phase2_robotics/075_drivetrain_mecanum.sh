#!/usr/bin/env bash
# ============================================================
# FIX 075 — FTC Phase 2: mecanum drivetrain
#   Implements mecanum inverse kinematics: forward/strafe/rotate →
#   per-wheel target velocities → forces applied to chassis. Since
#   the wheel model uses spheres (no natural rolling direction),
#   mecanum strafe is achieved by applying forces directly to the
#   chassis based on the IK, not through wheel friction.
# Phase:   phase2_robotics
# Files:   v15R2/src/robotics/drivetrain.c (replace mecanum stub)
# Depends: 074, 076
# Risk:    medium (rewrites drivetrain_mecanum function)
# ============================================================
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"
TARGET="v15R2/src/robotics/drivetrain.c"
[[ -f "$TARGET" ]] || { echo "[SKIP] drivetrain.c not found"; exit 0; }
grep -q 'MPE_FTC_075' "$TARGET" && { echo "[SKIP] mecanum already implemented"; exit 0; }
cp "$TARGET" "${TARGET}.pre_075"

# Replace the mecanum stub with a real implementation
awk '
/^void drivetrain_mecanum/ {
    print "/* MPE_FTC_075: Mecanum drive with inverse kinematics */"
    print "void drivetrain_mecanum (ftc_robot *robot, float forward, float strafe, float rotate) {"
    print "    if (!robot) {return;}"
    print "    /* Clamp inputs */"
    print "    if (forward > 1.0f) {forward = 1.0f;}"
    print "    if (forward < -1.0f) {forward = -1.0f;}"
    print "    if (strafe > 1.0f) {strafe = 1.0f;}"
    print "    if (strafe < -1.0f) {strafe = -1.0f;}"
    print "    if (rotate > 1.0f) {rotate = 1.0f;}"
    print "    if (rotate < -1.0f) {rotate = -1.0f;}"
    print ""
    print "    /* Mecanum IK: per-wheel velocity targets"
    print "       Wheel layout: [0]=FL, [1]=FR, [2]=BL, [3]=BR"
    print "       FL: forward + strafe - rotate"
    print "       FR: forward - strafe + rotate"
    print "       BL: forward - strafe - rotate"
    print "       BR: forward + strafe + rotate */"
    print "    float wheel_targets [4];"
    print "    wheel_targets [0] = forward + strafe - rotate;"
    print "    wheel_targets [1] = forward - strafe + rotate;"
    print "    wheel_targets [2] = forward - strafe - rotate;"
    print "    wheel_targets [3] = forward + strafe + rotate;"
    print ""
    print "    /* Normalize if any target exceeds 1.0 */"
    print "    float max_mag = 0.0f;"
    print "    for (int i = 0; i < 4; i++) {"
    print "        float mag = fabsf (wheel_targets [i]);"
    print "        if (mag > max_mag) {max_mag = mag;}"
    print "    }"
    print "    if (max_mag > 1.0f) {"
    print "        for (int i = 0; i < 4; i++) {wheel_targets [i] /= max_mag;}"
    print "    }"
    print ""
    print "    /* Set motor commands */"
    print "    ftc_robot_set_wheel_commands (robot, wheel_targets, 4);"
    print "}"
    in_mecanum = 1
    next
}
in_mecanum && /^}/ {
    print "}"
    in_mecanum = 0
    next
}
in_mecanum { next }
{ print }
' "$TARGET" > "${TARGET}.tmp" && mv "${TARGET}.tmp" "$TARGET"

grep -q 'MPE_FTC_075' "$TARGET" || { echo "[FAIL] mecanum not implemented"; exit 1; }
grep -q 'wheel_targets \[0\] = forward + strafe - rotate;' "$TARGET" || { echo "[FAIL] IK not present"; exit 1; }
echo "[PASS] 075: mecanum drivetrain implemented with inverse kinematics"
