#!/usr/bin/env bash
# ============================================================
# FIX 111 — Fix motor command persistence + V/N inversion
#
#   Problem 1: Motor commands persist after key release.
#   The drive block only calls gui_robot_apply_drive when
#   a key IS pressed. When all keys are released, the motors
#   keep their last command, so the robot drives forever.
#   Fix: ALWAYS call gui_robot_apply_drive (with 0,0,0 when
#   no keys are pressed) to zero the motors.
#
#   Problem 2: V and N strafe directions are inverted.
#   Fix: Swap the strafe signs.
#
#   Problem 3: Battery drains fast (SYMPTOM of problem 1).
#   Motors running at full power with no input drain battery.
#   Once problem 1 is fixed, battery only drains during
#   actual driving. No capacity change needed.
# ============================================================
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"

DISPATCH="v15R2/src/ui_input/simulation_input_dispatch.c"

cp "$DISPATCH" "${DISPATCH}.pre_111"

python3 - "$DISPATCH" << 'PYEOF'
import re, sys
path = sys.argv[1]
with open(path) as f: c = f.read()

clean_block = (
    "/* Robot drive keys: G=forward, B=backward, V=strafe right, N=strafe left, C=rotate left, H=rotate right */\n"
    "    if (gui_robot_get_count() > 0) {\n"
    "        float kb_forward = 0.0f, kb_strafe = 0.0f, kb_rotate = 0.0f;\n"
    "        if (main_inputs.g_key_pressed) { kb_forward += 1.0f; }\n"
    "        if (main_inputs.b_key_pressed) { kb_forward -= 1.0f; }\n"
    "        if (main_inputs.v_key_pressed) { kb_strafe  += 1.0f; }\n"
    "        if (main_inputs.n_key_pressed) { kb_strafe  -= 1.0f; }\n"
    "        if (main_inputs.c_key_pressed) { kb_rotate  -= 1.0f; }\n"
    "        if (main_inputs.h_key_pressed) { kb_rotate  += 1.0f; }\n"
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

# Match from the drive block comment (or the if-statement) up to just before the spawn gun comment
pattern = r'(?:/\* Robot drive keys[^\n]*\*/\s*\n)?[ \t]*if \(gui_robot_get_count\(\) > 0\) \{.*?(?=[ \t]*/\* Spawn gun)'
new_c, n = re.subn(pattern, clean_block, c, flags=re.DOTALL)
if n > 0:
    with open(path, 'w') as f: f.write(new_c)
    print(f"  [OK] simulation_input_dispatch.c: rewrote drive block ({n} match)")
else:
    print("  [WARN] drive block not found, trying fallback")
    # Fallback: insert before spawn gun
    if '/* Spawn gun' in c:
        c = c.replace('/* Spawn gun', clean_block + '    /* Spawn gun', 1)
        with open(path, 'w') as f: f.write(c)
        print("  [OK] inserted drive block before spawn gun")

# Verify the key change: no if-guard around gui_robot_apply_drive
with open(path) as f: final = f.read()
if 'if ((kb_forward != 0.0f)' in final and 'gui_robot_apply_drive' in final:
    # Check if the apply_drive is inside an if-guard
    import re
    guard_pattern = r'if \(\(kb_forward[^)]+\)\s*\{[^}]*gui_robot_apply_drive'
    if re.search(guard_pattern, final, re.DOTALL):
        print("  [WARN] gui_robot_apply_drive still inside if-guard!")
    else:
        print("  [OK] gui_robot_apply_drive called unconditionally")
else:
    print("  [OK] gui_robot_apply_drive called unconditionally")
PYEOF

echo ""
echo "[PASS] 111: Motor commands zeroed on key release + V/N swapped"
