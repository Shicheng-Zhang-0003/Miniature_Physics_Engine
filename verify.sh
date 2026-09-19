#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")" && pwd)"
cd "$ROOT"

echo "=== Post-fix verification (v15S head) ==="
echo ""

# 1. Build
echo "--- Build check ---"
cd v15S/src
make clean > /dev/null 2>&1 || true
if make 2>&1 | tail -5; then
    echo "[PASS] Build succeeded"
else
    echo "[FAIL] Build failed"
    exit 1
fi
cd "$ROOT"

# 2. Version string (release macros live in the v15S tree)
echo ""
echo "--- Version check ---"
if grep -q 'a3_version_string' v15S/src/mpe_engine.h; then
    echo "[PASS] mpe_engine.h carries a3_version_string ($(grep -o '"v[^"]*"' v15S/src/mpe_engine.h | head -n 1))"
else
    echo "[WARN] mpe_engine.h missing a3_version_string"
fi

# 3. File sizes (god file check)
echo ""
echo "--- File size check ---"
SIM_LINES=$(wc -l < v15S/src/simulation.c)
TERM_LINES=$(wc -l < v15S/src/ui_input/debug_terminal.c)
echo "  simulation.c:        $SIM_LINES lines"
echo "  debug_terminal.c:    $TERM_LINES lines"

# 4. Headless suite (fast gate: build + run the module + truth tests)
echo ""
echo "--- Headless spot check ---"
(cd v15S/src && python3 ../../tools/test_runner.py module 2>&1 | tail -3)

echo ""
echo "=== Verification complete ==="
