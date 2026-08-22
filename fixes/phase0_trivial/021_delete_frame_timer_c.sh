#!/usr/bin/env bash
# ============================================================
# FIX 021 — ARCH-016: delete empty frame_timer.c
# Phase:   phase0_trivial
# Files:   v15R2/src/core/frame_timer.c
# Depends: none
# Risk:    low
# ============================================================
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"

TARGET="v15R2/src/core/frame_timer.c"

if [[ ! -f "$TARGET" ]]; then
    echo "[SKIP] $TARGET already removed"
    exit 0
fi

# Verify it's actually the empty stub
if ! grep -q 'This is only here for building and compiling' "$TARGET"; then
    echo "[SKIP] frame_timer.c has real content — not deleting"
    exit 0
fi

# The makefile uses find to discover .c files, so removing this
# file simply means it won't be compiled. The header is still used.
cp "$TARGET" "${TARGET}.pre_021"
rm "$TARGET"

echo "[PASS] 021: ARCH-016 fixed — empty frame_timer.c removed"
