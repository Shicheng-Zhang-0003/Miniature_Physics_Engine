#!/usr/bin/env bash
# ============================================================
# FIX 001 — BUG-001: microvim file_exists always false
# Phase:   phase0_trivial
# Files:   v15R3/src/ui_input/microvim.c
# Depends: none
# Risk:    low
# ============================================================
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"

TARGET="v15R3/src/ui_input/microvim.c"

# --- Preflight ---
if [[ ! -f "$TARGET" ]]; then
    echo "[SKIP] $TARGET not found"
    exit 0
fi

if ! grep -q 'mv\.file_exists = false; /\* MPE_TASK_V15R2_INIT_FILE_EXISTS \*/' "$TARGET"; then
    echo "[SKIP] Bug pattern not found — already fixed or file changed"
    exit 0
fi

# --- Backup ---
cp "$TARGET" "${TARGET}.pre_001"

# --- Fix: delete the line that overwrites file_exists with false ---
sed -i '/mv\.file_exists = false; \/\* MPE_TASK_V15R2_INIT_FILE_EXISTS \*\//d' "$TARGET"

# --- Postflight ---
if grep -q 'mv\.file_exists = false; /\* MPE_TASK_V15R2_INIT_FILE_EXISTS \*/' "$TARGET"; then
    echo "[FAIL] Fix did not apply"
    exit 1
fi

# Verify the correct line still exists (the one that SETS file_exists)
if ! grep -q 'mv\.file_exists = mv_load_file' "$TARGET"; then
    echo "[FAIL] The legitimate file_exists assignment was also removed"
    exit 1
fi

echo "[PASS] 001: BUG-001 fixed — microvim file_exists no longer overwritten"
