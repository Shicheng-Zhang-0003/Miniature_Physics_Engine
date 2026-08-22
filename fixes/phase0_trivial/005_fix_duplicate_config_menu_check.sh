#!/usr/bin/env bash
# ============================================================
# FIX 005 — BUG-007: duplicate config_menu_is_open() check
# Phase:   phase0_trivial
# Files:   v15R2/src/ui_input/input_control.c
# Depends: none
# Risk:    low
# ============================================================
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"

TARGET="v15R2/src/ui_input/input_control.c"

# --- Preflight ---
if [[ ! -f "$TARGET" ]]; then
    echo "[SKIP] $TARGET not found"
    exit 0
fi

if ! grep -q '(!config_menu_is_open ()) && (!config_menu_is_open ())' "$TARGET"; then
    echo "[SKIP] Duplicate condition not found — already fixed"
    exit 0
fi

# --- Backup ---
cp "$TARGET" "${TARGET}.pre_005"

# --- Fix: collapse the duplicate into a single check ---
sed -i 's/(!config_menu_is_open ()) && (!config_menu_is_open ())/(!config_menu_is_open ())/' "$TARGET"

# --- Postflight ---
if grep -q '(!config_menu_is_open ()) && (!config_menu_is_open ())' "$TARGET"; then
    echo "[FAIL] Duplicate condition still present"
    exit 1
fi

# Verify the single check still exists
if ! grep -q '(!config_menu_is_open ())' "$TARGET"; then
    echo "[FAIL] The config_menu_is_open check was removed entirely"
    exit 1
fi

echo "[PASS] 005: BUG-007 fixed — duplicate config_menu_is_open() removed"
