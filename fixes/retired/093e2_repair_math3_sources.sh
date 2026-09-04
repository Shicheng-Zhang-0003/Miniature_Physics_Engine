#!/usr/bin/env bash
# ============================================================
# FIX 093e2 — Repair: math3_inverse test listed non-existent .c files
#   math3D and math4_special are header-only. The 093e make target
#   wrongly listed core/math3D.c and core/math4_special.c, which don't
#   exist. Remove them from the source list and re-run the diagnostic.
#
# Phase:   phase3_sensors (cylinder keystone)
# Files:   v15R2/src/makefile
# Depends: 093e (test file already written)
# Risk:    low
# ============================================================
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"

MAKEFILE="v15R2/src/makefile"
TEST="v15R2/src/tests/math3_inverse_test.c"

[[ -f "$MAKEFILE" ]] || { echo "[SKIP] makefile not found"; exit 0; }
[[ -f "$TEST" ]] || { echo "[FAIL] math3_inverse_test.c missing — run 093e first"; exit 1; }

cp "$MAKEFILE" "${MAKEFILE}.pre_093e2"

# Remove the non-existent header-only sources from the test target.
sed -i 's| core/math3D\.c core/math4_special\.c||g' "$MAKEFILE"

# Confirm the bad files are gone from the target
if grep -q 'core/math3D\.c' "$MAKEFILE"; then
    echo "[FAIL] core/math3D.c still referenced in makefile"
    exit 1
fi

cd v15R2/src
if make test_math3_inverse > /tmp/math3_inv_093e2.log 2>&1; then
    echo "----- math3_inverse diagnostic -----"
    grep -E '\[A\]|\[B\]|\[PASS\]|\[DIAG\]|\[FAIL\]' /tmp/math3_inv_093e2.log || true
    echo "------------------------------------"
    echo "[PASS] 093e2: math3_inverse diagnostic completed"
else
    tail -20 /tmp/math3_inv_093e2.log
    echo "[FAIL] 093e2: diagnostic failed to build or crashed"
    exit 1
fi
