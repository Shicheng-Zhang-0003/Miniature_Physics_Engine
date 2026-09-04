#!/usr/bin/env bash
# ============================================================
# FIX 059e — Repair: catch the missed zero-arg internal call to
#   broadphase_update_cell_size() that 059 left behind because of
#   whitespace mismatch. Uses extended regex to match any leading
#   whitespace, so it works regardless of tabs/spaces/indent depth.
# Phase:   phase0_foundation
# Files:   v15R3/src/physics/broadphase.c
# Depends: 059
# Risk:    low (single targeted replacement, idempotent)
# ============================================================
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"
TARGET="v15R3/src/physics/broadphase.c"

[[ -f "$TARGET" ]] || { echo "[SKIP] $TARGET not found"; exit 0; }

# Already correct? (no zero-arg calls remain)
if ! grep -qE 'broadphase_update_cell_size \(\);' "$TARGET"; then
  echo "[SKIP] No zero-arg broadphase_update_cell_size() calls remain"
  exit 0
fi

cp "$TARGET" "${TARGET}.pre_059e"

# Replace ANY zero-arg call, preserving whatever leading whitespace exists
sed -i -E 's|broadphase_update_cell_size \(\);|broadphase_update_cell_size (bodies, body_count); /* MPE_FTC_059e */|g' "$TARGET"

# Postflight: no zero-arg calls may remain
if grep -qE 'broadphase_update_cell_size \(\);' "$TARGET"; then
  echo "[FAIL] zero-arg call still present:"
  grep -nE 'broadphase_update_cell_size \(\);' "$TARGET"
  exit 1
fi

# And the fixed call must exist
if ! grep -q 'broadphase_update_cell_size (bodies, body_count);' "$TARGET"; then
  echo "[FAIL] fixed call not found"
  exit 1
fi

echo "[PASS] 059e: internal broadphase_update_cell_size call now passes (bodies, body_count)"
