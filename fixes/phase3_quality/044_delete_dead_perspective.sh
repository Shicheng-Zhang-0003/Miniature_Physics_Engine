#!/usr/bin/env bash
# ============================================================
# FIX 044 — ARCH-015: delete dead fov_aspr_perspective
# Phase:   phase3_quality
# Files:   v15R2/src/core/math3D.h
# Depends: none
# Risk:    low
# ============================================================
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"

TARGET="v15R2/src/core/math3D.h"

if [[ ! -f "$TARGET" ]]; then
    echo "[SKIP] $TARGET not found"
    exit 0
fi

if ! grep -q 'fov_aspr_perspective' "$TARGET"; then
    echo "[SKIP] fov_aspr_perspective already removed"
    exit 0
fi

cp "$TARGET" "${TARGET}.pre_044"

# Delete the function (starts with "static inline math3 fov_aspr_perspective"
# and ends with the closing "}" before "#endif //math3D_h")
awk '
/^static inline math3 fov_aspr_perspective/ { skip=1 }
/^#endif \/\/math3D_h/ { skip=0 }
!skip { print }
' "$TARGET" > "${TARGET}.tmp" && mv "${TARGET}.tmp" "$TARGET"

# Postflight
if grep -q 'fov_aspr_perspective' "$TARGET"; then
    echo "[FAIL] fov_aspr_perspective still present"
    exit 1
fi

# Verify math4_perspective_fov still exists in math4_special.h
if ! grep -q 'math4_perspective_fov' "v15R2/src/core/math4_special.h"; then
    echo "[FAIL] math4_perspective_fov missing from math4_special.h"
    exit 1
fi

echo "[PASS] 044: ARCH-015 fixed — dead fov_aspr_perspective removed"
