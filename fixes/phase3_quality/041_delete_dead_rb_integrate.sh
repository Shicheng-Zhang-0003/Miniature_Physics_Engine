#!/usr/bin/env bash
# ============================================================
# FIX 041 — ARCH-012: delete dead rb_integrate()
# Phase:   phase3_quality
# Files:   v15R2/src/core/rigidbody.c, v15R2/src/core/rigidbody.h
# Depends: none
# Risk:    low
# ============================================================
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"

TARGET_C="v15R2/src/core/rigidbody.c"
TARGET_H="v15R2/src/core/rigidbody.h"

if [[ ! -f "$TARGET_C" ]]; then
    echo "[SKIP] $TARGET_C not found"
    exit 0
fi

# Check if the function still exists
if ! grep -q '^void rb_integrate (' "$TARGET_C"; then
    echo "[SKIP] rb_integrate already removed"
    exit 0
fi

cp "$TARGET_C" "${TARGET_C}.pre_041"
cp "$TARGET_H" "${TARGET_H}.pre_041"

# Remove the declaration from the header
sed -i '/^void rb_integrate (rigidbody \*rigid_body, float delta_time, float linear_damping, float angular_damping);$/d' "$TARGET_H"

# Remove the function body from the .c file
# The function starts with "void rb_integrate (" and ends before "void rb_integrate_velocity ("
# Use awk to delete everything between the two markers
awk '
/^void rb_integrate \(rigidbody \*rigid_body, float delta_time/ { skip=1 }
/^void rb_integrate_velocity \(rigidbody \*rigid_body, float delta_time/ { skip=0 }
!skip { print }
' "$TARGET_C" > "${TARGET_C}.tmp" && mv "${TARGET_C}.tmp" "$TARGET_C"

# Postflight
if grep -q 'void rb_integrate (' "$TARGET_C"; then
    echo "[FAIL] rb_integrate still present in rigidbody.c"
    exit 1
fi

if grep -q 'void rb_integrate (' "$TARGET_H"; then
    echo "[FAIL] rb_integrate still declared in rigidbody.h"
    exit 1
fi

# Verify rb_integrate_velocity and rb_integrate_position still exist
if ! grep -q 'void rb_integrate_velocity' "$TARGET_C"; then
    echo "[FAIL] rb_integrate_velocity was accidentally removed"
    exit 1
fi

if ! grep -q 'void rb_integrate_position' "$TARGET_C"; then
    echo "[FAIL] rb_integrate_position was accidentally removed"
    exit 1
fi

echo "[PASS] 041: ARCH-012 fixed — dead rb_integrate() removed"
