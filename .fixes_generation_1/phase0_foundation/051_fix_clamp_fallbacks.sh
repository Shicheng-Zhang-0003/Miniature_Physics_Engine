#!/usr/bin/env bash
# ============================================================
# FIX 051 — PHYS: clamp fallbacks defeat config
#   separation_bias clamped to magic 5.0f, restitution_bias to 4.0f,
#   instead of their registered config values.
# Phase:   phase0_foundation
# Files:   v15R3/src/physics/collision_mechanics.c
# Depends: none
# Risk:    low (two unique single-line replacements)
# ============================================================
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"
TARGET="v15R3/src/physics/collision_mechanics.c"
[[ -f "$TARGET" ]] || { echo "[SKIP] $TARGET not found"; exit 0; }

if grep -q 'cp -> separation_bias = g_cfg.solver.max_separation_bias;' "$TARGET"; then
  echo "[SKIP] Clamp fallbacks already reference config"; exit 0
fi
grep -q 'cp -> separation_bias = 5.0f;' "$TARGET" || { echo "[SKIP] separation clamp pattern not found"; exit 0; }

cp "$TARGET" "${TARGET}.pre_051"
sed -i 's/cp -> separation_bias = 5\.0f;/cp -> separation_bias = g_cfg.solver.max_separation_bias;/' "$TARGET"
sed -i 's/cp -> restitution_bias = 4\.0f;/cp -> restitution_bias = g_cfg.solver.max_restitution_bias;/' "$TARGET"

grep -q 'cp -> separation_bias = g_cfg.solver.max_separation_bias;' "$TARGET" || { echo "[FAIL] separation clamp not updated"; exit 1; }
grep -q 'cp -> restitution_bias = g_cfg.solver.max_restitution_bias;' "$TARGET" || { echo "[FAIL] restitution clamp not updated"; exit 1; }
if grep -q 'cp -> separation_bias = 5\.0f;' "$TARGET"; then echo "[FAIL] magic 5.0f still present"; exit 1; fi
echo "[PASS] 051: clamp fallbacks now use config values"
