#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"

echo "=== Increment Split Verification ==="

# 1. Build check
echo "--- Build ---"
python3 tools/build_check.py
echo ""

# 2. Run all headless tests
echo "--- Tests ---"
python3 tools/test_runner.py
echo ""

# 3. Check simulation.c line count
SIM_LINES=$(wc -l < v15R3/src/simulation.c)
echo "simulation.c: $SIM_LINES lines (was 1123)"
echo ""

# 4. Check new files exist
for f in core/simulation_camera.c core/simulation_camera.h core/simulation_physics_loop.c core/simulation_physics_loop.h; do
    if [[ -f "v15R3/src/$f" ]]; then
        echo "[OK] $f exists"
    else
        echo "[FAIL] $f missing"
        exit 1
    fi
done

echo ""
echo "=== All checks passed ==="
