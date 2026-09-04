#!/usr/bin/env bash
# ============================================================
# FIX 019 — BUILD-005: add error checking to src/compile
# Phase:   phase0_trivial
# Files:   v15R3/src/compile
# Depends: none
# Risk:    low
# ============================================================
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"

TARGET="v15R3/src/compile"

if [[ ! -f "$TARGET" ]]; then
    echo "[SKIP] $TARGET not found"
    exit 0
fi

if grep -q 'set -e' "$TARGET"; then
    echo "[SKIP] compile script already has error checking"
    exit 0
fi

cp "$TARGET" "${TARGET}.pre_019"

cat > "$TARGET" << 'EOF'
#!/usr/bin/env bash
set -e
make clean
make -j"$(nproc)"
echo "Build complete."
EOF

chmod +x "$TARGET"

if ! grep -q 'set -e' "$TARGET"; then
    echo "[FAIL] compile script not updated"
    exit 1
fi

echo "[PASS] 019: BUILD-005 fixed — compile script now has error checking"
