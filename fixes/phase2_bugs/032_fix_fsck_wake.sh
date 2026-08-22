#!/usr/bin/env bash
# ============================================================
# FIX 032 — BUG-012: fsck auto-fix doesn't wake sanitized body
# Phase:   phase2_bugs
# Files:   v15R2/src/ui_input/debug_terminal.c
# Depends: 030
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

# The pattern: rigidbody_sanitize followed by printf "sanitized" without a wake call
if ! grep -q 'rigidbody_sanitize (rb);' "$TARGET"; then
    echo "[SKIP] rigidbody_sanitize not found in fsck"
    exit 0
fi

# Check if wake is already there
if grep -A1 'rigidbody_sanitize (rb);' "$TARGET" | grep -q 'rigidbody_wake'; then
    echo "[SKIP] rigidbody_wake already present after sanitize"
    exit 0
fi

cp "$TARGET" "${TARGET}.pre_032"

# Add rigidbody_wake after rigidbody_sanitize in the fsck auto-fix block
sed -i 's/rigidbody_sanitize (rb);/rigidbody_sanitize (rb);\n                            rigidbody_wake (rb); \/* FIX_032 *\//' "$TARGET"

# Postflight
if ! grep -A1 'rigidbody_sanitize (rb);' "$TARGET" | grep -q 'rigidbody_wake'; then
    echo "[FAIL] rigidbody_wake not added"
    exit 1
fi

echo "[PASS] 032: BUG-012 fixed — fsck auto-fix now wakes sanitized bodies"
