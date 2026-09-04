#!/usr/bin/env bash
# ============================================================
# FIX 093g — Repair: math3_inverse epsilon too large for small bodies
#   math3_inverse uses `math_epsilon` (1e-6) to detect singular matrices.
#   For small/light bodies like FTC wheels, the inertia tensor determinant
#   is ~1e-10, which falls below 1e-6 and is wrongly treated as singular,
#   returning a zero matrix. This prevents torque from spinning wheels.
#
#   Fix: use a smaller absolute epsilon (1e-12f) specifically for the
#   matrix inversion singularity check. This is safe for IEEE-754 floats
#   and allows small inertia tensors to invert correctly.
#
# Phase:   phase3_sensors (cylinder keystone)
# Files:   v15R3/src/core/math3D.h
# Depends: 093f
# Risk:    medium (modifies core math, but isolated to one check)
# ============================================================
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"

M3D="v15R3/src/core/math3D.h"
[[ -f "$M3D" ]] || { echo "[SKIP] $M3D not found"; exit 0; }
grep -q 'MPE_FTC_093g' "$M3D" && { echo "[SKIP] 093g already applied"; exit 0; }

cp "$M3D" "${M3D}.pre_093g"

# Replace the singularity check in math3_inverse
sed -i 's/(fabsf(determinant) < math_epsilon)/(fabsf(determinant) < 1e-12f) \/* MPE_FTC_093g: smaller epsilon for small inertia tensors *\//' "$M3D"

# Postflight
if ! grep -q 'MPE_FTC_093g' "$M3D"; then
    echo "[FAIL] math3_inverse epsilon not updated"
    exit 1
fi

# Build check
cd v15R3/src
if make > /tmp/build_093g.log 2>&1; then
    echo "[PASS] 093g: math3_inverse epsilon reduced to 1e-12f"
else
    tail -10 /tmp/build_093g.log
    echo "[FAIL] 093g: build failed"
    exit 1
fi
