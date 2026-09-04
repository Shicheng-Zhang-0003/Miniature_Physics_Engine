#!/usr/bin/env bash
# ============================================================
# FIX 074 — FTC Phase 2: tank drivetrain
#   Maps left/right power (-1..1) to wheel motor commands.
# Phase:   phase2_robotics
# Files:   v15R3/src/robotics/drivetrain.h, drivetrain.c (new)
# Depends: 073
# Risk:    low (new files only)
# ============================================================
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"
DIR="v15R3/src/robotics"
H="$DIR/drivetrain.h"
C="$DIR/drivetrain.c"
grep -q 'MPE_FTC_074' "$C" 2>/dev/null && { echo "[SKIP] drivetrain already present"; exit 0; }
[[ -f "$DIR/robot.h" ]] || { echo "[SKIP] robot.h missing (run 073 first)"; exit 0; }

cat > "$H" <<'EOF'
/* MPE_FTC_074: Drivetrain systems */
#ifndef drivetrain_h
#define drivetrain_h
#include "robot.h"
#include "../core/physics_world.h"

/* Tank drive: independent left/right power */
void drivetrain_tank (ftc_robot *robot, float left_power, float right_power);

/* Mecanum drive: forward/strafe/rotate (stub for Phase 3) */
void drivetrain_mecanum (ftc_robot *robot, float forward, float strafe, float rotate);

/* One drivetrain update tick: sets motor commands, then updates motors. */
void drivetrain_update (physics_world *world, ftc_robot *robot, float dt);

#endif /* drivetrain_h */
EOF

cat > "$C" <<'EOF'
/* MPE_FTC_074: Drivetrain implementation */
#include "drivetrain.h"

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
}

void drivetrain_mecanum (ftc_robot *robot, float forward, float strafe, float rotate) {
    /* Mecanum requires roller physics; deferred to Phase 3.
       For now, approximate as tank with mixed inputs. */
    if (!robot) {return;}
    float left = forward + rotate;
    float right = forward - rotate;
    drivetrain_tank (robot, left, right);
}

void drivetrain_update (physics_world *world, ftc_robot *robot, float dt) {
    if ((!world) || (!robot) || (dt <= 0.0f)) {return;}
    ftc_robot_update (world, robot, dt);
}
EOF

grep -q 'drivetrain_tank' "$H" || { echo "[FAIL] drivetrain.h not written"; exit 1; }
grep -q 'drivetrain_update' "$C" || { echo "[FAIL] drivetrain.c not written"; exit 1; }
echo "[PASS] 074: tank drivetrain added"
