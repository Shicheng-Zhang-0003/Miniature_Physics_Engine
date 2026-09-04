#!/usr/bin/env bash
# ============================================================
# FIX 037 — REND-009: replace render_init_status magic numbers
# Phase:   phase2_bugs
# Files:   v15R3/src/render/new_render.c
# Depends: none
# Risk:    low
# ============================================================
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"

TARGET="v15R3/src/render/new_render.c"

if [[ ! -f "$TARGET" ]]; then
    echo "[SKIP] $TARGET not found"
    exit 0
fi

if ! grep -q 'static int render_init_status = 0;' "$TARGET"; then
    echo "[SKIP] render_init_status already replaced"
    exit 0
fi

cp "$TARGET" "${TARGET}.pre_037"

# Replace the int declaration with an enum + variable
sed -i 's/static int render_init_status = 0;/typedef enum { RENDER_UNINITIALIZED = 0, RENDER_OK = 1, RENDER_FAILED = -1 } render_status;\nstatic render_status render_init_status = RENDER_UNINITIALIZED;/' "$TARGET"

# Replace the magic number assignments
sed -i 's/render_init_status = -1;/render_init_status = RENDER_FAILED;/' "$TARGET"
sed -i 's/render_init_status = 1;/render_init_status = RENDER_OK;/' "$TARGET"

# Replace the checks
sed -i 's/if (render_init_status) {return;}/if (render_init_status != RENDER_UNINITIALIZED) {return;}/' "$TARGET"
sed -i 's/if (render_init_status < 0)/if (render_init_status == RENDER_FAILED)/' "$TARGET"

# Postflight
if grep -q 'render_init_status = 0;' "$TARGET"; then
    echo "[FAIL] Magic number 0 still present"
    exit 1
fi

if ! grep -q 'RENDER_UNINITIALIZED' "$TARGET"; then
    echo "[FAIL] Enum not introduced"
    exit 1
fi

echo "[PASS] 037: REND-009 fixed — render_init_status now uses named enum"
