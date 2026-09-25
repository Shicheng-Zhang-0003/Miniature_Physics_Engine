#!/usr/bin/env bash
# V-02: make clean + make; verify binary; summarize compiler warnings.
set -uo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
TEMP="$ROOT/temp"
mkdir -p "$TEMP"
export TMPDIR="$TEMP"
SRC="$ROOT/v15S/src"
[[ -d "$SRC" ]] || { echo "ERROR: $SRC not found." >&2; exit 1; }

echo "=== V-02: Clean build + warning review ==="
cd "$SRC"
make clean
if make 2>&1 | tee "$TEMP/v02_build.log"; then
    echo "[PASS] make succeeded."
else
    echo "[FAIL] make failed." >&2; exit 1
fi

if [[ -x engine ]]; then
    echo "[PASS] engine binary produced."
else
    echo "[FAIL] engine binary missing." >&2; exit 1
fi

WARN_COUNT=$(grep -c "warning:" "$TEMP/v02_build.log" || true)
echo ""
echo "Compiler warnings: $WARN_COUNT"
if [[ "$WARN_COUNT" -gt 0 ]]; then
    echo "--- warnings (Gate 2: each must be reviewed & understood) ---"
    grep "warning:" "$TEMP/v02_build.log"
fi
