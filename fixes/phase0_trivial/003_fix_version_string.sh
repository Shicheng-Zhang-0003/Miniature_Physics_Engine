#!/usr/bin/env bash
# ============================================================
# FIX 003 — BUG-003: version string says v15R1, should be v15R2
# Phase:   phase0_trivial
# Files:   v15R2/src/mpe_engine.h
# Depends: none
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

if ! grep -q '#define a3_version_string "v15R1"' "$TARGET"; then
    echo "[SKIP] Version string already updated or changed"
    exit 0
fi

# --- Backup ---
cp "$TARGET" "${TARGET}.pre_003"

# --- Fix ---
sed -i 's/#define a3_version_string "v15R1"/#define a3_version_string "v15R2"/' "$TARGET"

# --- Postflight ---
if grep -q '#define a3_version_string "v15R1"' "$TARGET"; then
    echo "[FAIL] Version string still says v15R1"
    exit 1
fi

if ! grep -q '#define a3_version_string "v15R2"' "$TARGET"; then
    echo "[FAIL] Version string not set to v15R2"
    exit 1
fi

echo "[PASS] 003: BUG-003 fixed — version string now reads v15R2"
