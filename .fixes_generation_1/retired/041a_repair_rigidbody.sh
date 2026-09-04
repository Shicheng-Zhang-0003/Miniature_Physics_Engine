#!/usr/bin/env bash
# ============================================================
# FIX 041a — Repair: restore rigidbody.c/h from backup,
#             then delete ONLY rb_integrate (not velocity/position)
# Phase:   phase3_quality
# Files:   v15R3/src/core/rigidbody.c, v15R3/src/core/rigidbody.h
# Depends: 041 (failed)
# Risk:    low
# ============================================================
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"

TARGET_C="v15R3/src/core/rigidbody.c"
TARGET_H="v15R3/src/core/rigidbody.h"
BACKUP_C="${TARGET_C}.pre_041"
BACKUP_H="${TARGET_H}.pre_041"

# --- Step 1: Restore from backup ---
if [[ -f "$BACKUP_C" ]]; then
    cp "$BACKUP_C" "$TARGET_C"
    echo "  Restored rigidbody.c from backup"
else
    echo "[SKIP] No backup found for rigidbody.c"
    exit 0
fi

if [[ -f "$BACKUP_H" ]]; then
    cp "$BACKUP_H" "$TARGET_H"
    echo "  Restored rigidbody.h from backup"
fi

# --- Step 2: Verify rb_integrate exists in restored file ---
if ! grep -q 'void rb_integrate (rigidbody \*rigid_body, float delta_time, float linear_damping, float angular_damping)' "$TARGET_C"; then
    echo "[SKIP] rb_integrate not found in restored file"
    exit 0
fi

# --- Step 3: Delete ONLY rb_integrate body using line-number approach ---
# Find the start line (exact match: "void rb_integrate (" not "rb_integrate_")
START_LINE=$(grep -n '^void rb_integrate (rigidbody' "$TARGET_C" | head -1 | cut -d: -f1)

if [[ -z "$START_LINE" ]]; then
    echo "[SKIP] Could not locate rb_integrate definition"
    exit 0
fi

# Find the end: the next function definition after START_LINE
# rb_integrate_velocity is the next function
END_LINE=$(awk -v start="$START_LINE" 'NR > start && /^void rb_integrate_velocity/ { print NR; exit }' "$TARGET_C")

if [[ -z "$END_LINE" ]]; then
    echo "[FAIL] Could not find rb_integrate_velocity after rb_integrate"
    exit 1
fi

# Delete from START_LINE to END_LINE-1
DELETE_END=$((END_LINE - 1))
sed -i "${START_LINE},${DELETE_END}d" "$TARGET_C"

echo "  Deleted rb_integrate body (lines ${START_LINE}-${DELETE_END})"

# --- Step 4: Remove declaration from header ---
sed -i '/^void rb_integrate (rigidbody \*rigid_body, float delta_time, float linear_damping, float angular_damping);$/d' "$TARGET_H"

# --- Postflight: verify velocity and position still exist ---
if ! grep -q 'void rb_integrate_velocity' "$TARGET_C"; then
    echo "[FAIL] rb_integrate_velocity was removed"
    exit 1
fi

if ! grep -q 'void rb_integrate_position' "$TARGET_C"; then
    echo "[FAIL] rb_integrate_position was removed"
    exit 1
fi

if ! grep -q 'void rigidbody_wake' "$TARGET_C"; then
    echo "[FAIL] rigidbody_wake was removed"
    exit 1
fi

if ! grep -q 'void rigidbody_set_static' "$TARGET_C"; then
    echo "[FAIL] rigidbody_set_static was removed"
    exit 1
fi

if ! grep -q 'void rigidbody_initialisation_cube' "$TARGET_C"; then
    echo "[FAIL] rigidbody_initialisation_cube was removed"
    exit 1
fi

# Verify rb_integrate is gone
if grep -q '^void rb_integrate (rigidbody' "$TARGET_C"; then
    echo "[FAIL] rb_integrate still present"
    exit 1
fi

# Verify header declaration is gone
if grep -q 'void rb_integrate (rigidbody' "$TARGET_H"; then
    echo "[FAIL] rb_integrate declaration still in header"
    exit 1
fi

echo "[PASS] 041a: rigidbody.c/h repaired — only rb_integrate removed"
