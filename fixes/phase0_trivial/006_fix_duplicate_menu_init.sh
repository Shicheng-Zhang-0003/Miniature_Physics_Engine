#!/usr/bin/env bash
# ============================================================
# FIX 006 — BUG-008: duplicate menu_4/5/6_pressed init
# Phase:   phase0_trivial
# Files:   v15R3/src/ui_input/input_control.c
# Depends: 005
# Risk:    low
# ============================================================
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"

TARGET="v15R3/src/ui_input/input_control.c"

# --- Preflight ---
if [[ ! -f "$TARGET" ]]; then
    echo "[SKIP] $TARGET not found"
    exit 0
fi

# The duplicate block ends with this unique marker (MPE_TASK_35 without _FOCUS)
if ! grep -q 'menu_6_pressed = false; /\* MPE_TASK_35 \*/' "$TARGET"; then
    echo "[SKIP] Duplicate init block not found — already fixed"
    exit 0
fi

# --- Backup ---
cp "$TARGET" "${TARGET}.pre_006"

# --- Fix: find the line number of the unique marker, delete it and 2 lines above ---
LINE=$(grep -n 'menu_6_pressed = false; /\* MPE_TASK_35 \*/' "$TARGET" | grep -v 'FOCUS' | head -1 | cut -d: -f1)

if [[ -z "$LINE" ]]; then
    echo "[SKIP] Could not locate duplicate marker line"
    exit 0
fi

START=$((LINE - 2))
if [[ $START -lt 1 ]]; then
    START=1
fi

sed -i "${START},${LINE}d" "$TARGET"

# --- Postflight ---
if grep -q 'menu_6_pressed = false; /\* MPE_TASK_35 \*/' "$TARGET" | grep -v 'FOCUS'; then
    echo "[FAIL] Duplicate block still present"
    exit 1
fi

# The legitimate block (with _FOCUS) must still exist
if ! grep -q 'menu_6_pressed = false; /\* MPE_TASK_35_FOCUS \*/' "$TARGET"; then
    echo "[FAIL] Legitimate menu_6 init (MPE_TASK_35_FOCUS) was also removed"
    exit 1
fi

echo "[PASS] 006: BUG-008 fixed — duplicate menu_4/5/6 init removed"
