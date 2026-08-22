#!/usr/bin/env bash
# ============================================================
# FIX 010 — BUG-010: V04.sh references v15R1 instead of v15R2
# Phase:   phase0_trivial
# Files:   v15R2/validation/V04.sh
# Depends: none
# Risk:    low
# ============================================================
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"

TARGET="v15R2/validation/V04.sh"

# --- Preflight ---
if [[ ! -f "$TARGET" ]]; then
    echo "[SKIP] $TARGET not found"
    exit 0
fi

if ! grep -q 'v15R1' "$TARGET"; then
    echo "[SKIP] No v15R1 references found — already fixed"
    exit 0
fi

# --- Backup ---
cp "$TARGET" "${TARGET}.pre_010"

# --- Fix ---
sed -i 's/v15R1/v15R2/g' "$TARGET"

# --- Postflight ---
if grep -q 'v15R1' "$TARGET"; then
    echo "[FAIL] v15R1 references still remain"
    exit 1
fi

if ! grep -q 'v15R2' "$TARGET"; then
    echo "[FAIL] v15R2 not found after replacement"
    exit 1
fi

echo "[PASS] 010: BUG-010 fixed — V04.sh now references v15R2"
