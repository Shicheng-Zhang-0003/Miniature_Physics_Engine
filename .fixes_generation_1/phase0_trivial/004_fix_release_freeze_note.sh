#!/usr/bin/env bash
# ============================================================
# FIX 004 — BUG-004: release freeze note still says v15R2
# Phase:   phase0_trivial
# Files:   v15R3/src/mpe_engine.h
# Depends: 003
# Risk:    low
# ============================================================
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"

TARGET="v15R3/src/mpe_engine.h"

# --- Preflight ---
if [[ ! -f "$TARGET" ]]; then
    echo "[SKIP] $TARGET not found"
    exit 0
fi

if ! grep -q 'a3_release_freeze_note "v15R2 development cycle active"' "$TARGET"; then
    echo "[SKIP] Freeze note already updated or changed"
    exit 0
fi

# --- Backup ---
cp "$TARGET" "${TARGET}.pre_004"

# --- Fix ---
sed -i 's/#define a3_release_freeze_note "v15R2 development cycle active"/#define a3_release_freeze_note "v15R3 development cycle active"/' "$TARGET"

# --- Postflight ---
if grep -q 'a3_release_freeze_note "v15R2' "$TARGET"; then
    echo "[FAIL] Freeze note still references v15R2"
    exit 1
fi

if ! grep -q 'a3_release_freeze_note "v15R3 development cycle active"' "$TARGET"; then
    echo "[FAIL] Freeze note not set to v15R3"
    exit 1
fi

echo "[PASS] 004: BUG-004 fixed — release freeze note now reads v15R3"
