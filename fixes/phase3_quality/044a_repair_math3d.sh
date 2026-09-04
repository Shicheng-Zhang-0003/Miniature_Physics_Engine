#!/usr/bin/env bash
# ============================================================
# FIX 044a — Repair: delete fov_aspr_perspective from math3D.h
#             (previous script failed because line starts with "} static")
# Phase:   phase3_quality
# Files:   v15R3/src/core/math3D.h
# Depends: 044 (failed)
# Risk:    low
# ============================================================
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"

TARGET="v15R3/src/core/math3D.h"

if [[ ! -f "$TARGET" ]]; then
    echo "[SKIP] $TARGET not found"
    exit 0
fi

if ! grep -q 'fov_aspr_perspective' "$TARGET"; then
    echo "[SKIP] fov_aspr_perspective already removed"
    exit 0
fi

cp "$TARGET" "${TARGET}.pre_044a"

# The function starts on a line like:
#   } static inline math3 fov_aspr_perspective (float field_of_view, ...
# We need to:
#   1. Replace that line with just "}"  (keep the closing brace of math3_inverse)
#   2. Delete all subsequent lines until (but not including) "#endif //math3D_h"

# Step 1: Replace the opening line, keeping only the "}"
sed -i 's/^} static inline math3 fov_aspr_perspective.*/}/' "$TARGET"

# Step 2: Delete everything between the lone "}" and "#endif //math3D_h"
# Find the line number of the lone "}" that replaced fov_aspr_perspective
# and the line number of #endif
BRACE_LINE=$(grep -n '^}$' "$TARGET" | tail -1 | cut -d: -f1)
ENDIF_LINE=$(grep -n '^#endif //math3D_h' "$TARGET" | head -1 | cut -d: -f1)

if [[ -z "$BRACE_LINE" || -z "$ENDIF_LINE" ]]; then
    echo "[FAIL] Could not locate boundaries"
    exit 1
fi

# Delete from BRACE_LINE+1 to ENDIF_LINE-1
if [[ $((BRACE_LINE + 1)) -lt $ENDIF_LINE ]]; then
    sed -i "$((BRACE_LINE + 1)),$((ENDIF_LINE - 1))d" "$TARGET"
fi

# --- Postflight ---
if grep -q 'fov_aspr_perspective' "$TARGET"; then
    echo "[FAIL] fov_aspr_perspective still present"
    exit 1
fi

if ! grep -q '^#endif //math3D_h' "$TARGET"; then
    echo "[FAIL] #endif guard was removed"
    exit 1
fi

if ! grep -q 'math3_inverse' "$TARGET"; then
    echo "[FAIL] math3_inverse was removed"
    exit 1
fi

echo "[PASS] 044a: fov_aspr_perspective removed from math3D.h"
