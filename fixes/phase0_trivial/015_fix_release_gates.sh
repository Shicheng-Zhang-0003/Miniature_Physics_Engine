#!/usr/bin/env bash
# ============================================================
# FIX 015 — DOC-004: update RELEASE_GATES.md references
# Phase:   phase0_trivial
# Files:   v15R2/RELEASE_GATES.md
# Depends: none
# Risk:    low
# ============================================================
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"

TARGET="v15R2/RELEASE_GATES.md"

if [[ ! -f "$TARGET" ]]; then
    echo "[SKIP] $TARGET not found"
    exit 0
fi

if ! grep -q 'v15R1' "$TARGET"; then
    echo "[SKIP] No v15R1 references found"
    exit 0
fi

cp "$TARGET" "${TARGET}.pre_015"

sed -i 's/v15R1/v15R2/g' "$TARGET"

if grep -q 'v15R1' "$TARGET"; then
    echo "[FAIL] v15R1 still present"
    exit 1
fi

echo "[PASS] 015: DOC-004 fixed — RELEASE_GATES.md now references v15R2"
