#!/usr/bin/env bash
# ============================================================
# FIX 081 — Fix typo in wheel_traction.c: half_extents → half_extensions
#   The rigidbody struct member is `half_extensions`, not `half_extents`.
#   Then rebuild and run the mecanum drive test to verify.
# Phase:   phase2_robotics
# Files:   v15R3/src/robotics/wheel_traction.c
# Depends: 076
# Risk:    trivial
# ============================================================
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"
TARGET="v15R3/src/robotics/wheel_traction.c"
[[ -f "$TARGET" ]] || { echo "[SKIP] wheel_traction.c not found"; exit 0; }

# Already fixed?
if grep -q 'half_extensions' "$TARGET" && ! grep -q 'half_extents' "$TARGET"; then
  echo "[SKIP] half_extensions typo already corrected"
  # Still rebuild + run to confirm
else
  cp "$TARGET" "${TARGET}.pre_081"
  sed -i 's/half_extents/half_extensions/g' "$TARGET"
fi

# Verify fix applied
grep -q 'half_extensions' "$TARGET" || { echo "[FAIL] half_extensions not present"; exit 1; }
if grep -q 'half_extents' "$TARGET"; then
  echo "[FAIL] half_extents typo still present"
  exit 1
fi

# Rebuild and run the mecanum test
cd v15R3/src
if make test_mecanum_drive > /tmp/mecanum_test.log 2>&1; then
  tail -6 /tmp/mecanum_test.log
  echo "[PASS] 081: wheel_traction typo fixed; mecanum test passes"
else
  tail -12 /tmp/mecanum_test.log
  echo "[FAIL] 081: mecanum test still failing after typo fix"
  exit 1
fi
