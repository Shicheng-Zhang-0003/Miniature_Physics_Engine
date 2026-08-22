#!/usr/bin/env bash
# ============================================================
# FIX 018 — DOC-007: change ./compile to make in install docs
# Phase:   phase0_trivial
# Files:   v15R2/install/linux/linux_install_instructions.md
# Depends: 017
# Risk:    low
# ============================================================
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"

TARGET="v15R2/install/linux/linux_install_instructions.md"

if [[ ! -f "$TARGET" ]]; then
    echo "[SKIP] $TARGET not found"
    exit 0
fi

if ! grep -q 'Run ./compile' "$TARGET"; then
    echo "[SKIP] ./compile reference not found — already fixed"
    exit 0
fi

cp "$TARGET" "${TARGET}.pre_018"

sed -i 's|Run ./compile|Run make|' "$TARGET"

if grep -q 'Run ./compile' "$TARGET"; then
    echo "[FAIL] ./compile reference still present"
    exit 1
fi

echo "[PASS] 018: DOC-007 fixed — install docs now say 'make'"
