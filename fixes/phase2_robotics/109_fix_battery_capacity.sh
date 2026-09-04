#!/usr/bin/env bash
# ============================================================
# FIX 109 — Increase battery capacity from 3.0 Ah to 10.0 Ah
#
#   Problem: Battery drains too fast in simulator.
#   Root cause: 3.0 Ah capacity is too small for realistic
#   motor current draw. Real FTC batteries are typically
#   10-15 Ah (multiple cells in parallel).
#
#   Fix: Increase capacity to 10.0 Ah.
# ============================================================
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"

BATTERY_C="v15R2/src/robotics/battery.c"

cp "$BATTERY_C" "${BATTERY_C}.pre_109"

python3 - "$BATTERY_C" << 'PYEOF'
import sys
path = sys.argv[1]
with open(path) as f:
    content = f.read()

# Replace 3.0f capacity with 10.0f
if 'b->capacity_ah = 3.0f;' in content:
    content = content.replace('b->capacity_ah = 3.0f;', 'b->capacity_ah = 10.0f; /* FIX 109: realistic FTC battery capacity */')
    with open(path, 'w') as f:
        f.write(content)
    print("  [OK] battery.c: capacity increased to 10.0 Ah")
else:
    print("  [SKIP] capacity already changed or pattern not found")
PYEOF

echo "[PASS] 109: Battery capacity increased to 10.0 Ah"
