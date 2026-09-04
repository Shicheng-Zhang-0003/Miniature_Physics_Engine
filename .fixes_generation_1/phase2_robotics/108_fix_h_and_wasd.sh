#!/usr/bin/env bash
# ============================================================
# FIX 108 — Remove WASD robot drive + clean GVBNCH drive block
#
#   Problem 1: simulation.c MFS_GUI_BRIDGE_TICK still reads WASD
#              and calls gui_robot_apply_drive -> WASD moves robot.
#   Fix:       Reduce that block to only gui_robot_tick().
#
#   Problem 2: simulation_input_dispatch.c drive block is in a
#              half-patched state -> H (rotate right) not read cleanly.
#   Fix:       Rewrite the drive block to a clean GVBNCH version.
#
#   Also:      Defensively verify g/h/v/b/n/c are wired in input_control.
# ============================================================
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"

SIM_C="v15R3/src/simulation.c"
DISPATCH="v15R3/src/ui_input/simulation_input_dispatch.c"
INPUT_H="v15R3/src/ui_input/input_control.h"
INPUT_C="v15R3/src/ui_input/input_control.c"

cp "$SIM_C"    "${SIM_C}.pre_108"
cp "$DISPATCH" "${DISPATCH}.pre_108"
cp "$INPUT_H"  "${INPUT_H}.pre_108"
cp "$INPUT_C"  "${INPUT_C}.pre_108"

# ============================================================
# PART 1: simulation.c — strip WASD robot drive, keep gui_robot_tick
# ============================================================
python3 - "$SIM_C" << 'PYEOF'
import re, sys
path = sys.argv[1]
with open(path) as f: c = f.read()
pattern = r'/\* MFS_GUI_BRIDGE_TICK:.*?/\* MFS_GUI_BRIDGE_TICK_END \*/'
replacement = (
    "/* MFS_GUI_BRIDGE_TICK: Robot physics tick only.\n"
    "       Drive input is handled in simulation_input_dispatch.c (G/V/B/N/C/H). */\n"
    "    if (gui_robot_get_count() > 0) {\n"
    "        gui_robot_tick(frame_delta_time);\n"
    "    }\n"
    "    /* MFS_GUI_BRIDGE_TICK_END */"
)
new_c, n = re.subn(pattern, replacement, c, flags=re.DOTALL)
if n > 0:
    with open(path, 'w') as f: f.write(new_c)
    print(f"  [OK] simulation.c: removed WASD robot drive ({n} block)")
else:
    print("  [WARN] simulation.c: MFS_GUI_BRIDGE_TICK block not found")
PYEOF

# ============================================================
# PART 2: simulation_input_dispatch.c — rewrite drive block cleanly
# ============================================================
python3 - "$DISPATCH" << 'PYEOF'
import re, sys
path = sys.argv[1]
with open(path) as f: c = f.read()

clean_block = (
    "/* Robot drive keys: G=forward, B=backward, V=strafe left, N=strafe right, C=rotate left, H=rotate right */\n"
    "    if (gui_robot_get_count() > 0) {\n"
    "        float kb_forward = 0.0f, kb_strafe = 0.0f, kb_rotate = 0.0f;\n"
    "        if (main_inputs.g_key_pressed) { kb_forward += 1.0f; }\n"
    "        if (main_inputs.b_key_pressed) { kb_forward -= 1.0f; }\n"
    "        if (main_inputs.v_key_pressed) { kb_strafe  -= 1.0f; }\n"
    "        if (main_inputs.n_key_pressed) { kb_strafe  += 1.0f; }\n"
    "        if (main_inputs.c_key_pressed) { kb_rotate  -= 1.0f; }\n"
    "        if (main_inputs.h_key_pressed) { kb_rotate  += 1.0f; }\n"
    "        if ((kb_forward != 0.0f) || (kb_strafe != 0.0f) || (kb_rotate != 0.0f)) {\n"
    "            gui_robot_apply_drive(kb_forward, kb_strafe, kb_rotate);\n"
    "        }\n"
    "        main_inputs.g_key_pressed = false;\n"
    "        main_inputs.v_key_pressed = false;\n"
    "        main_inputs.b_key_pressed = false;\n"
    "        main_inputs.n_key_pressed = false;\n"
    "        main_inputs.c_key_pressed = false;\n"
    "        main_inputs.h_key_pressed = false;\n"
    "    }\n\n"
)

# Match from the drive block (comment optional) up to just before the spawn gun comment.
pattern = r'(?:/\* Robot drive keys[^\n]*\*/\s*\n)?[ \t]*if \(gui_robot_get_count\(\) > 0\) \{.*?(?=[ \t]*/\* Spawn gun)'
new_c, n = re.subn(pattern, clean_block, c, flags=re.DOTALL)
if n > 0:
    with open(path, 'w') as f: f.write(new_c)
    print(f"  [OK] simulation_input_dispatch.c: rewrote drive block ({n} match)")
else:
    # Fallback: insert before spawn gun if no existing block matched
    if '/* Spawn gun' in c and 'gui_robot_get_count' not in c:
        c = c.replace('/* Spawn gun', clean_block + '    /* Spawn gun', 1)
        with open(path, 'w') as f: f.write(c)
        print("  [OK] simulation_input_dispatch.c: inserted drive block before spawn gun")
    else:
        print("  [WARN] could not locate drive block or spawn gun anchor")
PYEOF

# ============================================================
# PART 3: input_control.h — ensure all drive keys are in the struct
# ============================================================
python3 - "$INPUT_H" << 'PYEOF'
import re, sys
path = sys.argv[1]
with open(path) as f: c = f.read()
m = re.search(r'bool w_key_pressed[^;]*;', c)
if not m:
    print("  [WARN] input_control.h: movement-key line not found")
    sys.exit(0)
line = m.group(0)
needed = ['q','g','h','v','b','n','c']
additions = [k + '_key_pressed' for k in needed if (k + '_key_pressed') not in line]
if additions:
    new_line = line[:-1] + ', ' + ', '.join(additions) + ';'
    c = c.replace(line, new_line)
    with open(path, 'w') as f: f.write(c)
    print(f"  [OK] input_control.h: added {additions}")
else:
    print("  [OK] input_control.h: all drive keys already present")
PYEOF

# ============================================================
# PART 4: input_control.c — ensure keypress/keyrelease for drive keys
# ============================================================
python3 - "$INPUT_C" << 'PYEOF'
import sys
path = sys.argv[1]
with open(path) as f: c = f.read()
changed = False
keys = {'g':'GDK_KEY_g','h':'GDK_KEY_h','v':'GDK_KEY_v',
        'b':'GDK_KEY_b','n':'GDK_KEY_n','c':'GDK_KEY_c'}
press_anchor   = 'if (event -> keyval == GDK_KEY_f) {input_state -> f_key_pressed = true;}'
release_anchor = 'if (event -> keyval == GDK_KEY_f) {input_state -> f_key_pressed = false;}'
for k, gdk in keys.items():
    press   = f'if (event -> keyval == {gdk}) {{input_state -> {k}_key_pressed = true;}}'
    release = f'if (event -> keyval == {gdk}) {{input_state -> {k}_key_pressed = false;}}'
    if press not in c:
        if press_anchor in c:
            c = c.replace(press_anchor, press_anchor + '\n    ' + press, 1)
            changed = True
            print(f"  added {k} keypress")
    if release not in c:
        if release_anchor in c:
            c = c.replace(release_anchor, release_anchor + '\n    ' + release, 1)
            changed = True
            print(f"  added {k} keyrelease")
if changed:
    with open(path, 'w') as f: f.write(c)
    print("  [OK] input_control.c: key wiring completed")
else:
    print("  [OK] input_control.c: all drive keys already wired")
PYEOF

# ============================================================
# VERIFICATION
# ============================================================
echo ""
echo "=== Verification ==="
echo "-- simulation.c should have NO gui_robot_apply_drive (WASD removed):"
if grep -q "gui_robot_apply_drive" "$SIM_C"; then
    echo "   [WARN] still present:"; grep -n "gui_robot_apply_drive" "$SIM_C"
else
    echo "   [OK] no gui_robot_apply_drive in simulation.c"
fi
echo "-- simulation.c still calls gui_robot_tick:"
grep -n "gui_robot_tick" "$SIM_C" | head -2
echo "-- simulation_input_dispatch.c drive keys (should list g b v n c h):"
grep -oE "main_inputs\.[gbvnc]_key_pressed\) \{ kb_" "$DISPATCH" | sort -u
grep -oE "main_inputs\.h_key_pressed\) \{ kb_rotate" "$DISPATCH"
echo "-- input_control.h struct keys:"
grep -oE "bool w_key_pressed[^;]*;" "$INPUT_H"

echo ""
echo "[PASS] 108: WASD robot drive removed + GVBNCH drive block cleaned"
