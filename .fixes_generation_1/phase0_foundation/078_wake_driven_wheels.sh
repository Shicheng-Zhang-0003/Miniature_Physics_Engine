#!/usr/bin/env bash
# ============================================================
# FIX 078 — DRIVETRAIN: wake driven wheels so motor torque applies
#   Symptom: teleop test "passes" but motor RPM=0, robot only falls.
#   Cause: driven wheels sleep while stationary, so rb_integrate_velocity
#   returns early and motor torque in torque_accumulator is never applied.
#   Fix: wake each driven wheel when motor torque is applied.
# Phase:   phase0_foundation (drivetrain bring-up)
# Files:   v15R3/src/robotics/robot.c
# Depends: 073
# Risk:    low
# ============================================================
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"
TARGET="v15R3/src/robotics/robot.c"
[[ -f "$TARGET" ]] || { echo "[SKIP] $TARGET not found"; exit 0; }
grep -q 'MPE_FTC_078' "$TARGET" && { echo "[SKIP] 078 already applied"; exit 0; }
grep -q 'wheel->torque_accumulator.z += axle.z \* torque;' "$TARGET" \
  || { echo "[SKIP] torque-application anchor not found"; exit 0; }

cp "$TARGET" "${TARGET}.pre_078"
sed -i 's|wheel->torque_accumulator.z += axle.z \* torque;|wheel->torque_accumulator.z += axle.z * torque;\n        rigidbody_wake (wheel); /* MPE_FTC_078: keep driven wheels awake so motor torque is applied */|' "$TARGET"

grep -q 'MPE_FTC_078' "$TARGET" || { echo "[FAIL] wake call not added"; exit 1; }
echo "[PASS] 078: driven wheels woken so motor torque is applied"
