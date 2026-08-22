#!/usr/bin/env bash
# ============================================================
# FIX 029 — BUG-005: uptime measures from first call, not engine start
# Phase:   phase2_bugs
# Files:   v15R2/src/ui_input/debug_terminal.c
# Depends: none
# Risk:    low
# ============================================================
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"

TARGET="v15R2/src/ui_input/debug_terminal.c"

if [[ ! -f "$TARGET" ]]; then
    echo "[SKIP] $TARGET not found"
    exit 0
fi

# Check if the bug pattern exists (local static in cmd_uptime)
if ! grep -q 'static gint64 a3_term_start_time = 0;' "$TARGET"; then
    echo "[SKIP] uptime pattern not found — already fixed"
    exit 0
fi

cp "$TARGET" "${TARGET}.pre_029"

# Step 1: Add file-scope start time variable after term_alias_count declaration
sed -i '/^static int term_alias_count = 0;$/a static gint64 term_engine_start_time = 0; /* FIX_029 */' "$TARGET"

# Step 2: Initialize it in debug_terminal_open (after the mode sync call)
sed -i '/debug_terminal_sync_mode ();$/a\    if (term_engine_start_time == 0) { term_engine_start_time = g_get_monotonic_time (); } /* FIX_029 */' "$TARGET"

# Step 3: In cmd_uptime, replace the local static init with the global
# Remove the local static declaration
sed -i '/static gint64 a3_term_start_time = 0;/d' "$TARGET"
# Remove the init check
sed -i '/if (a3_term_start_time == 0) {a3_term_start_time = g_get_monotonic_time ();}/d' "$TARGET"
# Replace usage of a3_term_start_time with term_engine_start_time
sed -i 's/a3_term_start_time/term_engine_start_time/g' "$TARGET"

# Postflight
if grep -q 'static gint64 a3_term_start_time' "$TARGET"; then
    echo "[FAIL] Local static still present"
    exit 1
fi

if ! grep -q 'term_engine_start_time' "$TARGET"; then
    echo "[FAIL] Global start time not found"
    exit 1
fi

echo "[PASS] 029: BUG-005 fixed — uptime now measures from terminal open"
