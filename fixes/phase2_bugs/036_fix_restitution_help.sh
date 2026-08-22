#!/usr/bin/env bash
# ============================================================
# FIX 036 — PHYS-010: clarify negative restitution threshold
# Phase:   phase2_bugs
# Files:   v15R2/src/config/mpe_config_schema.c
# Depends: none
# Risk:    low
# ============================================================
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"

TARGET="v15R2/src/config/mpe_config_schema.c"

if [[ ! -f "$TARGET" ]]; then
    echo "[SKIP] $TARGET not found"
    exit 0
fi

if ! grep -q 'Approach speed below which bounce is suppressed' "$TARGET"; then
    echo "[SKIP] Help text already updated"
    exit 0
fi

cp "$TARGET" "${TARGET}.pre_036"

sed -i 's|"Approach speed below which bounce is suppressed (m/s)"|"Approach speed below which bounce is suppressed (negative = approaching, m/s)"|' "$TARGET"

if ! grep -q 'negative = approaching' "$TARGET"; then
    echo "[FAIL] Help text not updated"
    exit 1
fi

echo "[PASS] 036: PHYS-010 fixed — restitution threshold help clarified"
