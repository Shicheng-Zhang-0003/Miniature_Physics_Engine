#!/usr/bin/env bash
# ============================================================
# FIX 114 — Fix RPM (torque gear double-apply) + C/H rotation
#
#   Problem 1: RPM stuck at ~10.
#   Root cause: preset stall_torque is the OUTPUT torque, so kt
#   is already output-based. But output_torque multiplies by
#   gear_ratio AGAIN -> 30x too much torque -> wheel overshoots
#   free speed each tick -> back-EMF braking oscillation -> RPM
#   capped ~10. (ftc_debug_test flags this as "gear ratio
#   likely double-applied".)
#   Fix: remove the gear_ratio factor from output_torque.
#
#   Problem 2: C/H rotation inverted.
#   Fix: robust line-by-line sign swap with diagnostics.
# ============================================================
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"

MOTOR_C="v15R2/src/robotics/motor.c"
DISPATCH="v15R2/src/ui_input/simulation_input_dispatch.c"

cp "$MOTOR_C"  "${MOTOR_C}.pre_114"
cp "$DISPATCH" "${DISPATCH}.pre_114"

# ============================================================
# PART 1: Remove gear_ratio from output_torque in motor.c
# ============================================================
echo "--- motor.c output_torque BEFORE: ---"
grep -n 'output_torque' "$MOTOR_C" | grep -v '0.0f' || echo "  (none)"

python3 - "$MOTOR_C" << 'PYEOF'
import re, sys
path = sys.argv[1]
with open(path) as f:
    lines = f.read().split('\n')
modified = False
for i in range(len(lines)):
    if ('output_torque' in lines[i] and 'gear_ratio' in lines[i]
            and '=' in lines[i] and '0.0f' not in lines[i]):
        old = lines[i]
        lines[i] = re.sub(r'\s*\*\s*m->gear_ratio', '', lines[i])
        if lines[i] != old:
            modified = True
            print(f"  [OK] line {i+1}: removed * m->gear_ratio")
            print(f"       before: {old.strip()}")
            print(f"       after:  {lines[i].strip()}")
if modified:
    with open(path, 'w') as f:
        f.write('\n'.join(lines))
else:
    print("  [WARN] no output_torque line with gear_ratio found")
PYEOF

echo "--- motor.c output_torque AFTER: ---"
grep -n 'output_torque' "$MOTOR_C" | grep -v '0.0f' || echo "  (none)"

# ============================================================
# PART 2: Swap C/H rotation signs (robust line-by-line)
# ============================================================
echo ""
echo "--- C/H rotate lines BEFORE: ---"
grep -E "main_inputs\.[ch]_key_pressed.*kb_rotate" "$DISPATCH" || echo "  (none)"

python3 - "$DISPATCH" << 'PYEOF'
import sys
path = sys.argv[1]
with open(path) as f:
    lines = f.read().split('\n')
modified = False
for i in range(len(lines)):
    line = lines[i]
    if 'c_key_pressed' in line and 'kb_rotate' in line:
        if '+=' in line:
            lines[i] = line.replace('+=', '-=', 1); modified = True
            print(f"  [OK] C: += -> -=  (line {i+1})")
        elif '-=' in line:
            lines[i] = line.replace('-=', '+=', 1); modified = True
            print(f"  [OK] C: -= -> +=  (line {i+1})")
    if 'h_key_pressed' in line and 'kb_rotate' in line:
        if '+=' in line:
            lines[i] = line.replace('+=', '-=', 1); modified = True
            print(f"  [OK] H: += -> -=  (line {i+1})")
        elif '-=' in line:
            lines[i] = line.replace('-=', '+=', 1); modified = True
            print(f"  [OK] H: -= -> +=  (line {i+1})")
if modified:
    with open(path, 'w') as f:
        f.write('\n'.join(lines))
else:
    print("  [WARN] no C/H rotate lines found")
PYEOF

echo "--- C/H rotate lines AFTER: ---"
grep -E "main_inputs\.[ch]_key_pressed.*kb_rotate" "$DISPATCH" || echo "  (none)"

echo ""
echo "[PASS] 114: RPM torque fix + C/H rotation swap applied"
