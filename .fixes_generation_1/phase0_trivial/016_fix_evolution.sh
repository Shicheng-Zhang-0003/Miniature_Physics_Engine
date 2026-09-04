#!/usr/bin/env bash
# ============================================================
# FIX 016 — DOC-005: update evolution.txt
# Phase:   phase0_trivial
# Files:   v15R3/evolution.txt
# Depends: none
# Risk:    low
# ============================================================
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"

TARGET="v15R3/evolution.txt"

if [[ ! -f "$TARGET" ]]; then
    echo "[SKIP] $TARGET not found"
    exit 0
fi

if ! grep -q 'Current Head: v15R2' "$TARGET"; then
    echo "[SKIP] evolution.txt already updated"
    exit 0
fi

cp "$TARGET" "${TARGET}.pre_016"

# Replace "Phase 5 is in progress:" block
sed -i 's/Phase 5 is in progress:/Phase 5 is terminated:/' "$TARGET"
sed -i 's/Current Head: v15R2/Current Head: v15R3/' "$TARGET"

# Add v15R2 to the Prev Versions list (after v15R1 line; fallback to v14A3)
if grep -q '^v15R1$' "$TARGET"; then
    sed -i '/^v15R1$/a v15R2' "$TARGET"
else
    sed -i '/^v14A3$/a v15R2' "$TARGET"
fi

if ! grep -q 'Current Head: v15R3' "$TARGET"; then
    echo "[FAIL] evolution.txt not updated correctly"
    exit 1
fi

echo "[PASS] 016: DOC-005 fixed — evolution.txt updated to v15R3"
