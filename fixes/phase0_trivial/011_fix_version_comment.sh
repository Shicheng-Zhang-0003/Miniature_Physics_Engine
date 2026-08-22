#!/usr/bin/env bash
# ============================================================
# FIX 011 — BUG-003b: trailing comment on version define
# Phase:   phase0_trivial
# Files:   v15R2/src/mpe_engine.h
# Depends: 003
# Risk:    low
# ============================================================
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"

TARGET="v15R2/src/mpe_engine.h"

# --- Preflight ---
if [[ ! -f "$TARGET" ]]; then
    echo "[SKIP] $TARGET not found"
    exit 0
fi

if ! grep -q '/\* v15R1 release candidate \*/' "$TARGET"; then
    echo "[SKIP] Comment already updated or removed"
    exit 0
fi

# --- Backup ---
cp "$TARGET" "${TARGET}.pre_011"

# --- Fix ---
sed -i 's|/\* v15R1 release candidate \*/|/* v15R2 release candidate */|' "$TARGET"

# --- Postflight ---
if grep -q 'v15R1' "$TARGET"; then
    echo "[FAIL] v15R1 still present in $TARGET"
    grep -n 'v15R1' "$TARGET"
    exit 1
fi

echo "[PASS] 011: version comment updated to v15R2"
