#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")" && pwd)"
cd "$ROOT"

echo "=== Post-fix verification ==="
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

# 2. Version string
echo ""
echo "--- Version check ---"
if grep -q 'v15R3' v15R3/src/mpe_engine.h; then
    echo "[PASS] mpe_engine.h references v15R3"
else
    echo "[WARN] mpe_engine.h missing v15R3 (stale version string)"
fi

# 3. File sizes (god file check)
echo ""
echo "--- File size check ---"
SIM_LINES=$(wc -l < v15R3/src/simulation.c)
TERM_LINES=$(wc -l < v15R3/src/ui_input/debug_terminal.c)
echo "  simulation.c:        $SIM_LINES lines"
echo "  debug_terminal.c:    $TERM_LINES lines"

# 4. Backup count
echo ""
echo "--- Backup files created ---"
BACKUP_COUNT=$(find v15R3 -name '*.pre_*' | wc -l)
echo "  $BACKUP_COUNT backup files created by fix scripts"

echo ""
echo "=== Verification complete ==="
