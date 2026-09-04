#!/usr/bin/env bash
# ============================================================
# FIX 022 — ARCH-017: delete commented-out camera code in root_gtk.c
# Phase:   phase0_trivial
# Files:   v15R3/src/root_gtk.c
# Depends: none
# Risk:    low
# ============================================================
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"

TARGET="v15R3/src/root_gtk.c"

if [[ ! -f "$TARGET" ]]; then
    echo "[SKIP] $TARGET not found"
    exit 0
fi

if ! grep -q '//How to Pass Camera FOV View to GPU' "$TARGET"; then
    echo "[SKIP] Dead camera code not found — already removed"
    exit 0
fi

cp "$TARGET" "${TARGET}.pre_022"

# Delete from the comment to end of file
LINE=$(grep -n '//How to Pass Camera FOV View to GPU' "$TARGET" | head -1 | cut -d: -f1)
sed -i "${LINE},\$d" "$TARGET"

if grep -q '//How to Pass Camera FOV View to GPU' "$TARGET"; then
    echo "[FAIL] Dead code still present"
    exit 1
fi

echo "[PASS] 022: ARCH-017 fixed — dead camera code removed from root_gtk.c"
