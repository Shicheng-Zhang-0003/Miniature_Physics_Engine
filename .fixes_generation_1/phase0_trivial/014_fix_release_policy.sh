#!/usr/bin/env bash
# ============================================================
# FIX 014 — DOC-003: update RELEASE_POLICY.md header
# Phase:   phase0_trivial
# Files:   v15R3/RELEASE_POLICY.md
# Depends: none
# Risk:    low
# ============================================================
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"

TARGET="v15R3/RELEASE_POLICY.md"

if [[ ! -f "$TARGET" ]]; then
    echo "[SKIP] $TARGET not found"
    exit 0
fi

if ! grep -q 'v15R2' "$TARGET"; then
    echo "[SKIP] No v15R2 references found"
    exit 0
fi

cp "$TARGET" "${TARGET}.pre_014"

sed -i 's/v15R2/v15R3/g' "$TARGET"

if grep -q 'v15R2' "$TARGET"; then
    echo "[FAIL] v15R2 still present"
    exit 1
fi

echo "[PASS] 014: DOC-003 fixed — RELEASE_POLICY.md now references v15R3"
