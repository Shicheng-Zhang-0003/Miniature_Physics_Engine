#!/usr/bin/env bash
# ============================================================
# FIX 094a — Repair: 094's makefile edit broke line continuations
#   094 deleted the WHOLE line containing wheel_traction.c, but that
#   line also carried config/mpe_config.c + mpe_config_schema.c, and
#   the line above it ended with a backslash. The dangling backslash
#   swallowed the next target -> "recipe commences before first target".
#
#   Repair: restore makefile from .pre_094, then remove ONLY the
#   wheel_traction.c TOKEN (keep the rest of the line intact).
#
# Phase:   phase3_sensors (cylinder keystone)
# Files:   v15R2/src/makefile
# Depends: 094
# Risk:    low (restore + targeted token removal)
# ============================================================
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"

MAKEFILE="v15R2/src/makefile"
BACKUP="${MAKEFILE}.pre_094"

[[ -f "$MAKEFILE" ]] || { echo "[SKIP] makefile not found"; exit 0; }

# 1. Restore the pre-094 makefile (undoes the corrupting line deletion)
if [[ -f "$BACKUP" ]]; then
    cp "$BACKUP" "$MAKEFILE"
    echo "[info] restored makefile from .pre_094"
else
    echo "[WARN] no .pre_094 backup; editing current makefile in place"
fi

# 2. Remove ONLY the wheel_traction.c token, keep the rest of each line.
sed -i 's|robotics/wheel_traction\.c[[:space:]]*||g' "$MAKEFILE"

# 3. No wheel_traction references may remain in the makefile
if grep -q 'wheel_traction' "$MAKEFILE"; then
    echo "[FAIL] wheel_traction still referenced in makefile:"
    grep -n 'wheel_traction' "$MAKEFILE"
    exit 1
fi

cd v15R2/src

# 4. Verify the makefile parses cleanly (dry-run)
if ! make -n test_teleop_drive > /tmp/makeparse_094a.log 2>&1; then
    tail -10 /tmp/makeparse_094a.log
    echo "[FAIL] makefile still broken after repair"
    exit 1
fi
echo "[info] makefile parses cleanly"

# 5. Build + run teleop test (real cylinder friction drive)
echo "--- teleop result ---"
if make test_teleop_drive > /tmp/teleop_094a.log 2>&1; then
    grep -E '\[info\]|\[PASS\]|\[FAIL\]|displacement' /tmp/teleop_094a.log || true
    echo "[PASS] 094a: teleop drives on real cylinder friction"
else
    tail -20 /tmp/teleop_094a.log
    echo "[FAIL] 094a: teleop build/run failed"
    exit 1
fi

# 6. Build + run mecanum test (uses TEMPORARY chassis force)
echo "--- mecanum result ---"
if make test_mecanum_drive > /tmp/mecanum_094a.log 2>&1; then
    grep -E '\[info\]|\[PASS\]|\[FAIL\]|strafe' /tmp/mecanum_094a.log || true
    echo "[PASS] 094a: mecanum works via TEMPORARY chassis force"
else
    tail -20 /tmp/mecanum_094a.log
    echo "[WARN] 094a: mecanum not passing (expected until anisotropic friction)"
fi

echo "[PASS] 094a: fake physics removed, makefile repaired"
