#!/usr/bin/env bash
# ============================================================
# FIX 110 — Increase battery capacity to realistic FTC levels
#
#   Problem: Battery drains too fast during rotation (H key).
#   Root cause: 10Ah is still too low. Real FTC batteries are
#   30-40Ah. During rotation, all 4 mecanum wheels run at full
#   power in opposite directions, drawing maximum stall current.
#
#   Fix: Increase capacity to 30.0 Ah (realistic FTC battery).
# ============================================================
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"

BATTERY_C="v15R2/src/robotics/battery.c"

cp "$BATTERY_C" "${BATTERY_C}.pre_110"

python3 - "$BATTERY_C" << 'PYEOF'
import sys
path = sys.argv[1]
with open(path) as f:
    content = f.read()

# Replace 10.0f capacity with 30.0f
if 'b->capacity_ah = 10.0f;' in content:
    content = content.replace('b->capacity_ah = 10.0f;', 'b->capacity_ah = 30.0f; /* FIX 110: realistic FTC battery capacity (30Ah) */')
    with open(path, 'w') as f:
        f.write(content)
    print("  [OK] battery.c: capacity increased to 30.0 Ah")
elif 'b->capacity_ah = 3.0f;' in content:
    # In case fix 109 wasn't applied
    content = content.replace('b->capacity_ah = 3.0f;', 'b->capacity_ah = 30.0f; /* FIX 110: realistic FTC battery capacity (30Ah) */')
    with open(path, 'w') as f:
        f.write(content)
    print("  [OK] battery.c: capacity increased from 3.0 to 30.0 Ah")
else:
    print("  [SKIP] capacity already changed or pattern not found")
PYEOF

echo "[PASS] 110: Battery capacity increased to 30.0 Ah (realistic FTC battery)"
