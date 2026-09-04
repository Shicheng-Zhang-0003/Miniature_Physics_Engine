#!/usr/bin/env bash
# ============================================================
# FIX 008 — BUG-010: V01.sh references v15R2 instead of v15R3
# Phase:   phase0_trivial
# Files:   v15R3/validation/V01.sh
# Depends: none
# Risk:    low
# ============================================================
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"

TARGET="v15R3/validation/V01.sh"

# --- Preflight ---
if [[ ! -f "$TARGET" ]]; then
    echo "[SKIP] $TARGET not found"
    exit 0
fi

if ! grep -q 'v15R2' "$TARGET"; then
    echo "[SKIP] No v15R2 references found — already fixed"
    exit 0
fi

# --- Backup ---
cp "$TARGET" "${TARGET}.pre_008"

# --- Fix ---
sed -i 's/v15R2/v15R3/g' "$TARGET"

# --- Postflight ---
if grep -q 'v15R2' "$TARGET"; then
    echo "[FAIL] v15R2 references still remain"
    exit 1
fi

if ! grep -q 'v15R3' "$TARGET"; then
    echo "[FAIL] v15R3 not found after replacement"
    exit 1
fi

echo "[PASS] 008: BUG-010 fixed — V01.sh now references v15R3"
