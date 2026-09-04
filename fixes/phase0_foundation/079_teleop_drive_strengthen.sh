#!/usr/bin/env bash
# ============================================================
# FIX 079 — DRIVETRAIN: make teleop test require real driving
#   Old threshold (displacement > 0.05) is cleared just by falling.
#   Raise to 0.5 m over 3 s so a non-driving robot fails, then
#   rebuild and run the test for an immediate verdict.
# Phase:   phase0_foundation (drivetrain bring-up)
# Files:   v15R2/src/tests/teleop_drive_test.c
# Depends: 077, 078
# Risk:    low
# ============================================================
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"
TARGET="v15R2/src/tests/teleop_drive_test.c"
[[ -f "$TARGET" ]] || { echo "[SKIP] $TARGET not found"; exit 0; }
grep -q 'MPE_FTC_079' "$TARGET" && { echo "[SKIP] 079 already applied"; exit 0; }
grep -q 'total_displacement < 0.05f' "$TARGET" \
  || { echo "[SKIP] displacement threshold anchor not found"; exit 0; }

cp "$TARGET" "${TARGET}.pre_079"
sed -i 's|total_displacement < 0.05f|total_displacement < 0.5f /* MPE_FTC_079: require real driving, not just falling */|' "$TARGET"

grep -q 'MPE_FTC_079' "$TARGET" || { echo "[FAIL] threshold not updated"; exit 1; }

# Rebuild + run for an immediate verdict (recompiles robot.c from 078 too)
cd v15R2/src
if make test_teleop_drive > /tmp/teleop_drive.log 2>&1; then
  tail -6 /tmp/teleop_drive.log
  echo "[PASS] 079: teleop test now requires real driving, and it drives"
else
  tail -12 /tmp/teleop_drive.log
  echo "[FAIL] 079: drivetrain still not driving — needs deeper traction debug"
  exit 1
fi
