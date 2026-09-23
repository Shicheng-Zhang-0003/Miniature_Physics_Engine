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
if grep -q 'a3_version_string "v15S-dev"' v15S/src/mpe_engine.h; then
    echo "[PASS] mpe_engine.h carries a3_version_string v15S-dev"
else
    echo "[FAIL] mpe_engine.h version mismatch (want v15S-dev)"
    exit 1
fi
(cd v15S/src && make check-flags)

# 3. File sizes (god file check)
echo ""
echo "--- File size check ---"
SIM_LINES=$(wc -l < v15S/src/simulation.c)
TERM_LINES=$(wc -l < v15S/src/ui_input/debug_terminal.c)
echo "  simulation.c:        $SIM_LINES lines"
echo "  debug_terminal.c:    $TERM_LINES lines"

# 4. Headless suite v2 (full 31-test gate, not a spot check)
echo ""
echo "--- Headless full suite (v2) ---"
python3 tools/test_runner.py --suite 2>&1 | tee /tmp/mpe_verify.log | tail -8
grep -q "Blocking failures: 0" /tmp/mpe_verify.log

# 5. MFS robotics suite (8 gated + 5 info diags)
echo ""
echo "--- MFS suite ---"
(cd v15S/src/ecosystem/mfs && ./build_tests.sh 2>&1 | tee /tmp/mpe_verify_mfs.log | tail -3)
grep -q "fail=0" /tmp/mpe_verify_mfs.log

# 6. TUI f10 settle (byte-deterministic smoke)
echo ""
echo "--- TUI smoke (f10) ---"
(cd v15S/src && make tui-smoke 2>&1 | tail -3)

echo ""
echo "=== Verification complete ==="
