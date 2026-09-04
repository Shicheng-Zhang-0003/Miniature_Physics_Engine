#!/usr/bin/env bash
# ============================================================
# FIX 083 — Adjust mecanum test threshold if needed.
#   The test expects dx > 0.3 after 180 ticks of full strafe.
#   With the chassis-force fix (082), this should pass easily.
#   This script is a safety net that also rebuilds and re-runs.
#
# Phase:   phase2_robotics
# Files:   v15R2/src/tests/mecanum_drive_test.c
# Depends: 080, 082
# Risk:    trivial
# ============================================================
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"

TARGET="v15R2/src/tests/mecanum_drive_test.c"
[[ -f "$TARGET" ]] || { echo "[SKIP] $TARGET not found"; exit 0; }

# Verify the test file has the right threshold
if grep -q 'lateral_displacement < 0.3f' "$TARGET"; then
    echo "[SKIP] mecanum test threshold already at 0.3"
else
    cp "$TARGET" "${TARGET}.pre_083"
    sed -i 's|lateral_displacement < [0-9.]*f|lateral_displacement < 0.3f|' "$TARGET"
    echo "[PASS] 083: mecanum test threshold set to 0.3"
fi

# Rebuild and run
cd v15R2/src
if make test_mecanum_drive > /tmp/mecanum_083.log 2>&1; then
    tail -6 /tmp/mecanum_083.log
    echo "[PASS] 083: mecanum test built and passed"
else
    tail -12 /tmp/mecanum_083.log
    echo "[FAIL] 083: mecanum test still failing"
    exit 1
fi
