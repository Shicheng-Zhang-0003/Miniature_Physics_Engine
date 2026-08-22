#!/usr/bin/env bash
# ============================================================
# FIX 047 — QUAL-001a: remove A3_PATCH comment markers
# Phase:   phase3_quality
# Files:   all .c and .h files under v15R2/src/
# Depends: 041-046
# Risk:    low
# Note:    Only removes standalone comment lines like
#          /* A3_PATCH_XX_NAME */ that sit on their own line.
#          Does NOT rename functions or variables.
# ============================================================
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"

SRC_DIR="v15R2/src"
CHANGED=0

# Only remove standalone A3_PATCH/A3_TEST/A3_HOTFIX comment lines
# Pattern: line that is ONLY a comment marker (with optional whitespace)
for f in $(find "$SRC_DIR" -name '*.c' -o -name '*.h' | sort); do
    if grep -qE '^\s*/\* A3_(PATCH|TEST|HOTFIX)_[A-Z0-9_]+ \*/\s*$' "$f"; then
        sed -i -E '/^\s*\/\* A3_(PATCH|TEST|HOTFIX)_[A-Z0-9_]+ \*\/\s*$/d' "$f"
        CHANGED=$((CHANGED + 1))
    fi
done

if [[ $CHANGED -eq 0 ]]; then
    echo "[SKIP] No standalone A3_PATCH comment markers found"
    exit 0
fi

echo "[PASS] 047: removed standalone A3_PATCH markers from $CHANGED file(s)"
