#!/usr/bin/env bash
# ============================================================
# FIX 046 — QUAL-002: standardise initialise_input spelling
# Phase:   phase3_quality
# Files:   v15R3/src/ui_input/input_control.c, input_control.h,
#          v15R3/src/root_gtk.c
# Depends: none
# Risk:    low
# ============================================================
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"

TARGET_H="v15R3/src/ui_input/input_control.h"
TARGET_C="v15R3/src/ui_input/input_control.c"
CALLER="v15R3/src/root_gtk.c"

if [[ ! -f "$TARGET_H" ]]; then
    echo "[SKIP] $TARGET_H not found"
    exit 0
fi

if ! grep -q 'initialise_input' "$TARGET_H"; then
    echo "[SKIP] initialise_input already renamed"
    exit 0
fi

cp "$TARGET_H" "${TARGET_H}.pre_046"
cp "$TARGET_C" "${TARGET_C}.pre_046"
cp "$CALLER" "${CALLER}.pre_046"

sed -i 's/initialise_input/initialize_input/g' "$TARGET_H"
sed -i 's/initialise_input/initialize_input/g' "$TARGET_C"
sed -i 's/initialise_input/initialize_input/g' "$CALLER"

# Postflight
if grep -q 'initialise_input' "$TARGET_H" || grep -q 'initialise_input' "$TARGET_C" || grep -q 'initialise_input' "$CALLER"; then
    echo "[FAIL] initialise_input still present"
    exit 1
fi

if ! grep -q 'initialize_input' "$TARGET_H"; then
    echo "[FAIL] initialize_input not found in header"
    exit 1
fi

echo "[PASS] 046: QUAL-002 fixed — initialise_input renamed to initialize_input"
