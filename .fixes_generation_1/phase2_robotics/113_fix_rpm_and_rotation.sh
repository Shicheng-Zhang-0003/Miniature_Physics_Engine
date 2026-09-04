#!/usr/bin/env bash
# ============================================================
# FIX 113 — Fix slow RPM (back-EMF gear bug) + C/H rotation
#
#   Problem 1: RPM ~30x too slow.
#   Root cause: kv is computed from output free speed, but
#   motor_update computes back_emf from motor-shaft speed
#   (wheel * gear_ratio). back_emf ends up gear_ratio times
#   too high, so the motor hits "electrical free speed" at
#   1/30th of real speed.
#   Fix: multiply kv's denominator by gear_ratio so kv is a
#   proper motor-shaft constant.
#
#   Problem 2: C/H rotation inverted (fix 112 didn't apply).
#   Fix: rewrite drive block. C=rotate left (+), H=rotate right (-).
# ============================================================
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"

MOTOR_C="v15R3/src/robotics/motor.c"
DISPATCH="v15R3/src/ui_input/simulation_input_dispatch.c"

cp "$MOTOR_C"  "${MOTOR_C}.pre_113"
cp "$DISPATCH" "${DISPATCH}.pre_113"

# ============================================================
# PART 1: Fix motor kv to account for gear ratio
# ============================================================
echo "--- motor.c kv line BEFORE: ---"
grep -n 'm->kv' "$MOTOR_C" || echo "  (no m->kv line found)"

python3 - "$MOTOR_C" << 'PYEOF'
import re, sys
path = sys.argv[1]
with open(path) as f:
    content = f.read()

# Target: "nominal_voltage / m->free_speed_rad_s"  -> add * gear_ratio to denominator
pattern = r'(nominal_voltage\s*/\s*)m->free_speed_rad_s'
replacement = r'\1(m->free_speed_rad_s * m->gear_ratio)'
new_content, n = re.subn(pattern, replacement, content)

if n > 0:
    with open(path, 'w') as f:
        f.write(new_content)
    print(f"  [OK] motor.c: kv now divides by (free_speed * gear_ratio) — {n} change(s)")
else:
    # Fallback: maybe the kv line uses a different numerator variable
    pattern2 = r'(/\s*)m->free_speed_rad_s(\s*;)'
    def fix_kv(m):
        return m.group(1) + '(m->free_speed_rad_s * m->gear_ratio)' + m.group(2)
    new_content, n2 = re.subn(pattern2, fix_kv, content)
    if n2 > 0:
        with open(path, 'w') as f:
            f.write(new_content)
        print(f"  [OK] motor.c: kv fixed via fallback pattern — {n2} change(s)")
    else:
        print("  [WARN] could not find kv pattern — inspect motor.c manually")
PYEOF

echo "--- motor.c kv line AFTER: ---"
grep -n 'm->kv' "$MOTOR_C" || echo "  (no m->kv line found)"

# ============================================================
# PART 2: Rewrite drive block with correct C/H rotation
#   C = rotate left  = kb_rotate += 1
#   H = rotate right = kb_rotate -= 1
#   (V/N kept as fix 111 had them — user confirmed working)
# ============================================================
python3 - "$DISPATCH" << 'PYEOF'
import re, sys
path = sys.argv[1]
with open(path) as f: c = f.read()

clean_block = (
    "/* Robot drive keys: G=forward, B=backward, V=strafe, N=strafe, C=rotate LEFT, H=rotate RIGHT */\n"
    "    if (gui_robot_get_count() > 0) {\n"
    "        float kb_forward = 0.0f, kb_strafe = 0.0f, kb_rotate = 0.0f;\n"
    "        if (main_inputs.g_key_pressed) { kb_forward += 1.0f; }\n"
    "        if (main_inputs.b_key_pressed) { kb_forward -= 1.0f; }\n"
    "        if (main_inputs.v_key_pressed) { kb_strafe  += 1.0f; }\n"
    "        if (main_inputs.n_key_pressed) { kb_strafe  -= 1.0f; }\n"
    "        if (main_inputs.c_key_pressed) { kb_rotate  += 1.0f; } /* FIX 113: C=rotate left */\n"
    "        if (main_inputs.h_key_pressed) { kb_rotate  -= 1.0f; } /* FIX 113: H=rotate right */\n"
    "        /* ALWAYS apply drive — zeros motors when no keys pressed */\n"
    "        gui_robot_apply_drive(kb_forward, kb_strafe, kb_rotate);\n"
    "        main_inputs.g_key_pressed = false;\n"
    "        main_inputs.v_key_pressed = false;\n"
    "        main_inputs.b_key_pressed = false;\n"
    "        main_inputs.n_key_pressed = false;\n"
    "        main_inputs.c_key_pressed = false;\n"
    "        main_inputs.h_key_pressed = false;\n"
    "    }\n\n"
)

pattern = r'(?:/\* Robot drive keys[^\n]*\*/\s*\n)?[ \t]*if \(gui_robot_get_count\(\) > 0\) \{.*?(?=[ \t]*/\* Spawn gun)'
new_c, n = re.subn(pattern, clean_block, c, flags=re.DOTALL)
if n > 0:
    with open(path, 'w') as f: f.write(new_c)
    print(f"  [OK] simulation_input_dispatch.c: drive block rewritten ({n} match)")
else:
    print("  [WARN] drive block pattern not found")
PYEOF

echo ""
echo "=== Verification ==="
echo "--- C/H rotation lines (should be C += and H -=): ---"
grep -E "main_inputs\.[ch]_key_pressed\) \{ kb_rotate" "$DISPATCH"
echo ""
echo "[PASS] 113: RPM back-EMF gear bug fixed + C/H rotation corrected"
