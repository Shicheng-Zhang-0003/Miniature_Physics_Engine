#!/usr/bin/env bash
# ============================================================
# FIX 030 — BUG-006: capture buffer not freed on terminal close
# Phase:   phase2_bugs
# Files:   v15R3/src/ui_input/debug_terminal.c
# Depends: 029
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

# Check if cleanup is already present in destroy handler
if grep -q 'term_capture_reset' "$TARGET" && grep -A5 'on_terminal_window_destroy' "$TARGET" | grep -q 'term_capture_reset'; then
    echo "[SKIP] Capture cleanup already in destroy handler"
    exit 0
fi

cp "$TARGET" "${TARGET}.pre_030"

# Add term_capture_reset() to on_terminal_window_destroy
# The destroy function ends with "term_history_cursor = -1;"
sed -i '/^    term_history_cursor = -1;$/a\    term_capture_reset (); /* FIX_030: free capture buffer on close */' "$TARGET"

# Postflight
if ! grep -A6 'on_terminal_window_destroy' "$TARGET" | grep -q 'term_capture_reset'; then
    echo "[FAIL] term_capture_reset not added to destroy handler"
    exit 1
fi

echo "[PASS] 030: BUG-006 fixed — capture buffer freed on terminal close"
