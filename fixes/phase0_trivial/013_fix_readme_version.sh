#!/usr/bin/env bash
# ============================================================
# FIX 013 — DOC-002: update readme.md version references
# Phase:   phase0_trivial
# Files:   readme.md (root level, moved up with .git)
# Depends: none
# Risk:    low
# ============================================================
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"

# readme.md is at the root level (moved up with .git)
TARGET="readme.md"

if [[ ! -f "$TARGET" ]]; then
    # Fallback: check if it's still inside v15R3/
    if [[ -f "v15R3/readme.md" ]]; then
        TARGET="v15R3/readme.md"
    else
        echo "[SKIP] readme.md not found at root or v15R3/"
        exit 0
    fi
fi

if ! grep -q 'v15R2' "$TARGET"; then
    echo "[SKIP] No v15R2 references found in $TARGET"
    exit 0
fi

cp "$TARGET" "${TARGET}.pre_013"

sed -i 's/v15R2/v15R3/g' "$TARGET"

if grep -q 'v15R2' "$TARGET"; then
    echo "[FAIL] v15R2 still present in $TARGET"
    exit 1
fi

echo "[PASS] 013: DOC-002 fixed — $TARGET now references v15R3"
