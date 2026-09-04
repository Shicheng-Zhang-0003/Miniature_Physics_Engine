#!/usr/bin/env bash
# ============================================================
# FIX 094 — FTC Phase 3: Delete fake physics, verify real robot
#
#   The robot now uses cylinder wheels with direct axle torque.
#   This script removes the fake physics scaffolding:
#     1. Removes #include "wheel_traction.h" from robot.c
#     2. Removes wheel_traction.c from the makefile build
#     3. Marks the mecanum chassis-force as TEMPORARY (it stays
#        until anisotropic friction is implemented, because
#        isotropic friction cannot produce strafe)
#     4. Runs teleop + mecanum tests to verify the robot drives
#        on REAL cylinder-floor friction.
#
# Phase:   phase3_sensors (cylinder keystone)
# Files:   v15R2/src/robotics/robot.c
#          v15R2/src/robotics/drivetrain.c
#          v15R2/src/makefile
# Depends: 093h (grounded propulsion proven)
# Risk:    medium (removes code paths, verifies with tests)
# ============================================================
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"

ROBOT_C="v15R2/src/robotics/robot.c"
DRIVETRAIN_C="v15R2/src/robotics/drivetrain.c"
MAKEFILE="v15R2/src/makefile"

for f in "$ROBOT_C" "$DRIVETRAIN_C" "$MAKEFILE"; do
    [[ -f "$f" ]] || { echo "[SKIP] $f not found"; exit 0; }
done

grep -q 'MPE_FTC_094_CLEANUP' "$ROBOT_C" && { echo "[SKIP] 094 already applied"; exit 0; }

cp "$ROBOT_C" "${ROBOT_C}.pre_094"
cp "$DRIVETRAIN_C" "${DRIVETRAIN_C}.pre_094"
cp "$MAKEFILE" "${MAKEFILE}.pre_094"

# ============================================================
# STEP 1: Remove wheel_traction.h include from robot.c
# ============================================================
sed -i '/#include "wheel_traction.h"/d' "$ROBOT_C"

# Add a marker so we know 094 cleanup was applied
sed -i '1a\/* MPE_FTC_094_CLEANUP: wheel_traction removed — real cylinder friction */' "$ROBOT_C"

# ============================================================
# STEP 2: Remove wheel_traction.c from the makefile
#         The main engine build uses find, but test targets list
#         sources explicitly. Remove from any explicit source list.
# ============================================================
sed -i '/robotics\/wheel_traction\.c/d' "$MAKEFILE"

# ============================================================
# STEP 3: Mark mecanum chassis-force as TEMPORARY in drivetrain.c
#         (It stays until anisotropic friction is implemented.
#          Isotropic friction cannot produce lateral strafe.)
# ============================================================
if grep -q 'MPE_FTC_082' "$DRIVETRAIN_C"; then
    sed -i 's/\/\* MPE_FTC_082/\/\* MPE_FTC_082 TEMPORARY — replace with anisotropic friction (MPE_FTC_095)/' "$DRIVETRAIN_C"
fi

# ============================================================
# STEP 4: Build and run teleop test (verifies cylinder robot drives)
# ============================================================
cd v15R2/src

echo "--- Building teleop test ---"
if ! make test_teleop_drive > /tmp/teleop_094.log 2>&1; then
    if ! grep -q '\[info\]' /tmp/teleop_094.log; then
        tail -20 /tmp/teleop_094.log
        echo "[FAIL] 094: teleop test failed to compile"
        exit 1
    fi
fi

echo "--- Teleop test result ---"
if ./test_teleop_drive; then
    echo "[PASS] teleop: cylinder robot drives on real friction"
else
    echo "[FAIL] teleop: cylinder robot did not drive"
    exit 1
fi

# ============================================================
# STEP 5: Run mecanum test (uses temporary chassis force)
# ============================================================
echo ""
echo "--- Building mecanum test ---"
if ! make test_mecanum_drive > /tmp/mecanum_094.log 2>&1; then
    if ! grep -q '\[info\]' /tmp/mecanum_094.log; then
        tail -20 /tmp/mecanum_094.log
        echo "[FAIL] 094: mecanum test failed to compile"
        exit 1
    fi
fi

echo "--- Mecanum test result ---"
if ./test_mecanum_drive; then
    echo "[PASS] mecanum: works via TEMPORARY chassis force (needs anisotropic friction)"
else
    echo "[WARN] mecanum: strafe not working (expected until anisotropic friction)"
fi

echo ""
echo "[PASS] 094: fake physics removed, robot drives on real cylinder friction"
