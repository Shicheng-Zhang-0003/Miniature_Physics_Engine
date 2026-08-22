#!/usr/bin/env bash
# ============================================================
# FIX 017 — DOC-006: remove hardcoded GitHub URL
# Phase:   phase0_trivial
# Files:   v15R2/install/linux/linux_install_instructions.md
# Depends: none
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

if ! grep -q 'github.com/shicheng-zhang/physics-engine' "$TARGET"; then
    echo "[SKIP] GitHub URL not found — already fixed"
    exit 0
fi

cp "$TARGET" "${TARGET}.pre_017"

sed -i 's|git clone https://github.com/shicheng-zhang/physics-engine.git --> This gets the actual source code.|git clone <repository-url> --> This gets the actual source code.|' "$TARGET"

if grep -q 'github.com/shicheng-zhang/physics-engine' "$TARGET"; then
    echo "[FAIL] Hardcoded URL still present"
    exit 1
fi

echo "[PASS] 017: DOC-006 fixed — GitHub URL parameterized"
