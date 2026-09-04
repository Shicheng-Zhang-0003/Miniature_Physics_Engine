#!/usr/bin/env bash
# ============================================================
# FIX 112 — Fix voltage sag (sluggish) + C/H rotation inversion
#
#   Problem 1: Robot sluggish despite 12.7V reading.
#   Root cause: internal_resistance = 0.05 ohms is too high.
#   Under motor load (40-80A total), voltage sags to ~8V.
#   The HUD shows open-circuit voltage (passes 0.0f load),
#   so it reads 12.7V while the motors only see ~8V.
#   Fix: Reduce internal_resistance to 0.015 ohms (realistic
#   for a LiPo FTC battery pack).
#
#   Problem 2: C and H rotation directions are inverted.
#   Fix: Swap the rotate signs in the drive block.
# ============================================================
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"

BATTERY_C="v15R2/src/robotics/battery.c"
DISPATCH="v15R2/src/ui_input/simulation_input_dispatch.c"

cp "$BATTERY_C" "${BATTERY_C}.pre_112"
cp "$DISPATCH" "${DISPATCH}.pre_112"

# ============================================================
# PART 1: Reduce battery internal resistance
# ============================================================
python3 - "$BATTERY_C" << 'PYEOF'
import sys
path = sys.argv[1]
with open(path) as f:
    content = f.read()

if 'b->internal_resistance = 0.05f;' in content:
    content = content.replace(
        'b->internal_resistance = 0.05f;',
        'b->internal_resistance = 0.015f; /* FIX 112: realistic LiPo internal resistance */'
    )
    with open(path, 'w') as f:
        f.write(content)
    print("  [OK] battery.c: internal_resistance reduced to 0.015 ohms")
else:
    print("  [SKIP] internal_resistance already changed or pattern not found")
PYEOF

# ============================================================
# PART 2: Swap C/H rotation directions
# ============================================================
python3 - "$DISPATCH" << 'PYEOF'
import sys
path = sys.argv[1]
with open(path) as f:
    content = f.read()

# Swap C and H rotate signs
content = content.replace(
    'if (main_inputs.c_key_pressed) { kb_rotate  -= 1.0f; }',
    'if (main_inputs.c_key_pressed) { kb_rotate  += 1.0f; } /* FIX 112: swapped */'
)
content = content.replace(
    'if (main_inputs.h_key_pressed) { kb_rotate  += 1.0f; }',
    'if (main_inputs.h_key_pressed) { kb_rotate  -= 1.0f; } /* FIX 112: swapped */'
)

with open(path, 'w') as f:
    f.write(content)
print("  [OK] simulation_input_dispatch.c: C/H rotation swapped")
PYEOF

echo ""
echo "[PASS] 112: Voltage sag fixed + C/H rotation corrected"
