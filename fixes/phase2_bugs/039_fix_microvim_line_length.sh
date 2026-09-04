#!/usr/bin/env bash
# ============================================================
# FIX 039 — TERM-010: add line-length guard in MicroVim insert
# Phase:   phase2_bugs
# Files:   v15R3/src/ui_input/microvim.c
# Depends: none
# Risk:    low
# ============================================================
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"

TARGET="v15R3/src/ui_input/microvim.c"

if [[ ! -f "$TARGET" ]]; then
    echo "[SKIP] $TARGET not found"
    exit 0
fi

# Check if the guard already exists
if grep -q 'mv_max_line_len' "$TARGET" && grep -B2 'malloc ((size_t)(len + 2))' "$TARGET" | grep -q 'mv_max_line_len'; then
    echo "[SKIP] Line-length guard already present"
    exit 0
fi

cp "$TARGET" "${TARGET}.pre_039"

# Add guard before the malloc in mv_insert_char
# The pattern is: "int len = mv_line_len (row);" followed by "char *line = mv.lines [row];"
# We insert the guard between them
sed -i '/^    int len = mv_line_len (row);$/a\    if (len >= mv_max_line_len - 1) {return;} /* FIX_039: prevent overflow */' "$TARGET"

# Postflight
if ! grep -q 'mv_max_line_len - 1' "$TARGET"; then
    echo "[FAIL] Line-length guard not added"
    exit 1
fi

echo "[PASS] 039: TERM-010 fixed — MicroVim insert now guards against line overflow"
