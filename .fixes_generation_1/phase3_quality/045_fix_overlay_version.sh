#!/usr/bin/env bash
# ============================================================
# FIX 045 — BUG-003c: overlay label still says v15R2
# Phase:   phase3_quality
# Files:   v15R3/src/ui_input/overlay.c
# Depends: none
# Risk:    low
# ============================================================
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"

TARGET="v15R3/src/ui_input/overlay.c"

if [[ ! -f "$TARGET" ]]; then
    echo "[SKIP] $TARGET not found"
    exit 0
fi

if ! grep -q 'v15R2' "$TARGET"; then
    echo "[SKIP] No v15R2 reference in overlay.c"
    exit 0
fi

cp "$TARGET" "${TARGET}.pre_045"

sed -i 's/v15R2/v15R3/g' "$TARGET"

if grep -q 'v15R2' "$TARGET"; then
    echo "[FAIL] v15R2 still present in overlay.c"
    exit 1
fi

echo "[PASS] 045: overlay label updated to v15R3"
