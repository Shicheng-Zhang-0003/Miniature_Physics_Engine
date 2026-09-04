#!/usr/bin/env bash
# ============================================================
# FIX 030a — Repair: remove misplaced term_capture_reset,
#             ensure correct one in on_terminal_window_destroy
# Phase:   phase2_bugs
# Files:   v15R3/src/ui_input/debug_terminal.c
# Depends: 030
# Risk:    low
# ============================================================
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"

TARGET="v15R3/src/ui_input/debug_terminal.c"

if [[ ! -f "$TARGET" ]]; then
    echo "[SKIP] $TARGET not found"
    exit 0
fi

cp "$TARGET" "${TARGET}.pre_030a"

# Step 1: Remove any FIX_030 line that sits immediately before term_execute
#         (that is the wrong location inside on_terminal_entry_activate).
#         Read the next line; if it contains term_execute, drop the FIX_030 line.
sed -i '/FIX_030.*free capture buffer/{
    N
    /term_execute/s/.*\n//
}' "$TARGET"

# Step 2: Make sure term_capture_reset exists inside on_terminal_window_destroy.
#         Use a range so we only touch that one function.
if ! sed -n '/on_terminal_window_destroy/,/^}/p' "$TARGET" | grep -q 'term_capture_reset'; then
    sed -i '/on_terminal_window_destroy/,/^}/ {
        /term_history_cursor = -1;/a\    term_capture_reset (); /* FIX_030: free capture buffer on close */
    }' "$TARGET"
fi

# Postflight: correct location has it
if ! sed -n '/on_terminal_window_destroy/,/^}/p' "$TARGET" | grep -q 'term_capture_reset'; then
    echo "[FAIL] term_capture_reset missing from on_terminal_window_destroy"
    exit 1
fi

# Postflight: wrong location does NOT have it
if sed -n '/on_terminal_entry_activate/,/^}/p' "$TARGET" | grep -q 'term_capture_reset'; then
    echo "[FAIL] term_capture_reset still present in on_terminal_entry_activate"
    exit 1
fi

echo "[PASS] 030a: capture cleanup correctly placed in on_terminal_window_destroy only"
