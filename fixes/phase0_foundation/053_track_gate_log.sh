#!/usr/bin/env bash
# ============================================================
# FIX 053 — REPO: track release evidence (un-gitignore gate log)
#   *.log excludes v03_gate_validation.log; add a negation so the
#   P0 proof artifact is actually tracked.
# Phase:   phase0_foundation
# Files:   .gitignore
# Depends: none
# Risk:    low
# ============================================================
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"
TARGET=".gitignore"
[[ -f "$TARGET" ]] || { echo "[SKIP] $TARGET not found"; exit 0; }
grep -q '!v15R2/v03_gate_validation.log' "$TARGET" && { echo "[SKIP] Exception already present"; exit 0; }
grep -q '^\*\.log$' "$TARGET" || { echo "[SKIP] *.log rule not found"; exit 0; }

cp "$TARGET" "${TARGET}.pre_053"
sed -i 's|^\*\.log$|!v15R2/v03_gate_validation.log\n*.log|' "$TARGET"

grep -q '!v15R2/v03_gate_validation.log' "$TARGET" || { echo "[FAIL] exception not added"; exit 1; }
echo "[PASS] 053: v03_gate_validation.log is now tracked"
