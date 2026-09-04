#!/usr/bin/env bash
# ============================================================
# FIX 093b — Repair: driven wheel test needs rigidbody_wake
#   093a's wheel never spun because it fell asleep. A sleeping
#   body's rb_integrate_velocity returns early, so torque in
#   torque_accumulator is ignored. This is the exact issue FIX 078
#   solved for the robot's driven wheels. Mirror that fix: wake
#   the wheel each step when applying drive torque.
#
# Phase:   phase3_sensors (cylinder keystone)
# Files:   v15R3/src/tests/driven_wheel_test.c
# Depends: 093a
# Risk:    trivial
# ============================================================
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"

TEST="v15R3/src/tests/driven_wheel_test.c"
[[ -f "$TEST" ]] || { echo "[SKIP] $TEST not found"; exit 0; }
grep -q 'MPE_FTC_093b' "$TEST" && { echo "[SKIP] 093b already applied"; exit 0; }
grep -q 'torque_accumulator.x += drive_torque;' "$TEST" \
    || { echo "[SKIP] torque anchor not found"; exit 0; }

cp "$TEST" "${TEST}.pre_093b"

sed -i 's|world.bodies\[wheel\].torque_accumulator.x += drive_torque;|world.bodies[wheel].torque_accumulator.x += drive_torque;\n        rigidbody_wake(\&world.bodies[wheel]); /* MPE_FTC_093b: keep driven wheel awake (mirrors FIX 078) */|' "$TEST"

grep -q 'MPE_FTC_093b' "$TEST" || { echo "[FAIL] wake call not added"; exit 1; }

cd v15R3/src
if ! make test_driven_wheel > /tmp/driven_wheel_093b.log 2>&1; then
    if ! grep -q '\[info\] wheel dz' /tmp/driven_wheel_093b.log; then
        tail -15 /tmp/driven_wheel_093b.log
        echo "[FAIL] 093b: build failed"
        exit 1
    fi
fi

echo "----- driven wheel result (with wake) -----"
grep -E '\[info\]|\[PASS\]|\[GAP\]|\[FAIL\]' /tmp/driven_wheel_093b.log || true
echo "-------------------------------------------"

if grep -q '\[PASS\] driven cylinder wheel' /tmp/driven_wheel_093b.log; then
    echo "[PASS] 093b: wheel now spins and propels once kept awake"
else
    echo "[INFO] 093b: see diagnostics above — paste them back"
    exit 1
fi
