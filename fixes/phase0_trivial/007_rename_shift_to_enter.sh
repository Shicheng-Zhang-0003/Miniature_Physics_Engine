#!/usr/bin/env bash
# ============================================================
# FIX 007 — BUG-009: shift_* variables should be enter_*
# Phase:   phase0_trivial
# Files:   v15R2/src/simulation.c
# Depends: none
# Risk:    low
# ============================================================
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"

TARGET="v15R2/src/simulation.c"

# --- Preflight ---
if [[ ! -f "$TARGET" ]]; then
    echo "[SKIP] $TARGET not found"
    exit 0
fi

if ! grep -q 'shift_previously_held' "$TARGET"; then
    echo "[SKIP] shift_* variables not found — already renamed"
    exit 0
fi

# --- Backup ---
cp "$TARGET" "${TARGET}.pre_007"

# --- Fix: rename all three variables ---
sed -i 's/shift_hold_timer/enter_hold_timer/g' "$TARGET"
sed -i 's/shift_spawn_interval_timer/enter_spawn_interval_timer/g' "$TARGET"
sed -i 's/shift_previously_held/enter_previously_held/g' "$TARGET"

# --- Postflight ---
if grep -q 'shift_hold_timer\|shift_spawn_interval_timer\|shift_previously_held' "$TARGET"; then
    echo "[FAIL] Some shift_* variables still remain"
    exit 1
fi

if ! grep -q 'enter_previously_held' "$TARGET"; then
    echo "[FAIL] enter_previously_held not found after rename"
    exit 1
fi

if ! grep -q 'enter_hold_timer' "$TARGET"; then
    echo "[FAIL] enter_hold_timer not found after rename"
    exit 1
fi

if ! grep -q 'enter_spawn_interval_timer' "$TARGET"; then
    echo "[FAIL] enter_spawn_interval_timer not found after rename"
    exit 1
fi

echo "[PASS] 007: BUG-009 fixed — shift_* renamed to enter_*"
