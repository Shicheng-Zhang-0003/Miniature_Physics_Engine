#!/usr/bin/env bash
# ============================================================
# FIX 106 — Change robot drive keys to G/V/B/N + spawn offset
#
#   Problem 1: F key conflicts with force impulse.
#   Fix: Use G=forward, V=backward, B=left, N=right.
#
#   Problem 2: Robot spawns at (0,y,0) inside the default sphere.
#   Fix: Spawn at (5, rest_height, 5) so it's visible.
#
# Files: input_control.h, input_control.c,
#        simulation_input_dispatch.c, debug_terminal.c
# ============================================================
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"

H="v15R2/src/ui_input/input_control.h"
C="v15R2/src/ui_input/input_control.c"
D="v15R2/src/ui_input/simulation_input_dispatch.c"
T="v15R2/src/ui_input/debug_terminal.c"

# Backup
cp "$H" "${H}.pre_106"
cp "$C" "${C}.pre_106"
cp "$D" "${D}.pre_106"
cp "$T" "${T}.pre_106"

# ============================================================
# PART 1: Add v, b, n keys to input_control.h
# ============================================================
python3 - "$H" << 'PYEOF'
import sys
path = sys.argv[1]
with open(path, 'r') as f:
    content = f.read()

# Add v, b, n after h_key_pressed if not already present
if 'v_key_pressed' not in content:
    content = content.replace(
        'h_key_pressed;',
        'h_key_pressed, v_key_pressed, b_key_pressed, n_key_pressed;'
    )
    with open(path, 'w') as f:
        f.write(content)
    print("  [OK] input_control.h: added v, b, n keys")
else:
    print("  [SKIP] v/b/n keys already in header")
PYEOF

# ============================================================
# PART 2: Wire v, b, n keys in input_control.c
# ============================================================
python3 - "$C" << 'PYEOF'
import sys
path = sys.argv[1]
with open(path, 'r') as f:
    content = f.read()

if 'v_key_pressed' not in content:
    # Init (both initialize_input and on_focus_out)
    content = content.replace(
        'input_state -> h_key_pressed = false;',
        'input_state -> h_key_pressed = false;\n    input_state -> v_key_pressed = false;\n    input_state -> b_key_pressed = false;\n    input_state -> n_key_pressed = false;'
    )
    # Keypress
    content = content.replace(
        'if (event -> keyval == GDK_KEY_h) {input_state -> h_key_pressed = true;}',
        'if (event -> keyval == GDK_KEY_h) {input_state -> h_key_pressed = true;}\n    if (event -> keyval == GDK_KEY_v) {input_state -> v_key_pressed = true;}\n    if (event -> keyval == GDK_KEY_b) {input_state -> b_key_pressed = true;}\n    if (event -> keyval == GDK_KEY_n) {input_state -> n_key_pressed = true;}'
    )
    # Keyrelease
    content = content.replace(
        'if (event -> keyval == GDK_KEY_h) {input_state -> h_key_pressed = false;}',
        'if (event -> keyval == GDK_KEY_h) {input_state -> h_key_pressed = false;}\n    if (event -> keyval == GDK_KEY_v) {input_state -> v_key_pressed = false;}\n    if (event -> keyval == GDK_KEY_b) {input_state -> b_key_pressed = false;}\n    if (event -> keyval == GDK_KEY_n) {input_state -> n_key_pressed = false;}'
    )
    with open(path, 'w') as f:
        f.write(content)
    print("  [OK] input_control.c: wired v, b, n keys")
else:
    print("  [SKIP] v/b/n keys already wired")
PYEOF

# ============================================================
# PART 3: Update robot drive keys in simulation_input_dispatch.c
# Replace TFGH with GVBn
# ============================================================
python3 - "$D" << 'PYEOF'
import sys
path = sys.argv[1]
with open(path, 'r') as f:
    content = f.read()

# Replace the old TFGH drive block with GVBn
old_block = """/* Robot drive keys: T=forward, G=backward, F=left, H=right */
    if (gui_robot_get_count() > 0) {
        float kb_forward = 0.0f, kb_strafe = 0.0f, kb_rotate = 0.0f;
        if (main_inputs.t_key_pressed) kb_forward += 1.0f;
        if (main_inputs.g_key_pressed) kb_forward -= 1.0f;
        if (main_inputs.h_key_pressed) kb_strafe += 1.0f;
        if (main_inputs.f_key_pressed) kb_strafe -= 1.0f;
        if (main_inputs.e_key_pressed) kb_rotate += 1.0f;
        if (main_inputs.q_key_pressed) kb_rotate -= 1.0f;
        if (kb_forward != 0.0f || kb_strafe != 0.0f || kb_rotate != 0.0f) {
            gui_robot_apply_drive(kb_forward, kb_strafe, kb_rotate);
            main_inputs.t_key_pressed = false;
            main_inputs.g_key_pressed = false;
            main_inputs.h_key_pressed = false;
            main_inputs.f_key_pressed = false;
        }
    }"""

new_block = """/* Robot drive keys: G=forward, V=backward, B=left, N=right */
    if (gui_robot_get_count() > 0) {
        float kb_forward = 0.0f, kb_strafe = 0.0f, kb_rotate = 0.0f;
        if (main_inputs.g_key_pressed) kb_forward += 1.0f;
        if (main_inputs.v_key_pressed) kb_forward -= 1.0f;
        if (main_inputs.b_key_pressed) kb_strafe -= 1.0f;
        if (main_inputs.n_key_pressed) kb_strafe += 1.0f;
        if (main_inputs.e_key_pressed) kb_rotate += 1.0f;
        if (main_inputs.q_key_pressed) kb_rotate -= 1.0f;
        if (kb_forward != 0.0f || kb_strafe != 0.0f || kb_rotate != 0.0f) {
            gui_robot_apply_drive(kb_forward, kb_strafe, kb_rotate);
            main_inputs.g_key_pressed = false;
            main_inputs.v_key_pressed = false;
            main_inputs.b_key_pressed = false;
            main_inputs.n_key_pressed = false;
        }
    }"""

if old_block in content:
    content = content.replace(old_block, new_block)
    with open(path, 'w') as f:
        f.write(content)
    print("  [OK] simulation_input_dispatch.c: drive keys changed to GVBn")
else:
    # Try to find any robot drive block and replace it
    import re
    pattern = r'/\* Robot drive keys:.*?\*/\s*if \(gui_robot_get_count\(\) > 0\) \{.*?main_inputs\.\w+_key_pressed = false;\s*\}\s*\}'
    match = re.search(pattern, content, re.DOTALL)
    if match:
        content = content[:match.start()] + new_block + content[match.end():]
        with open(path, 'w') as f:
            f.write(content)
        print("  [OK] simulation_input_dispatch.c: drive keys replaced (regex)")
    else:
        print("  [WARN] Could not find robot drive block — check manually")
PYEOF

# ============================================================
# PART 4: Fix spawn position in debug_terminal.c
# Change from (0, rest_height, 0) to (5, rest_height, 5)
# ============================================================
python3 - "$T" << 'PYEOF'
import sys
path = sys.argv[1]
with open(path, 'r') as f:
    content = f.read()

old_spawn = 'gui_robot_spawn(0.0f, ftc_robot_rest_height(), 0.0f, MOTOR_GB_5203_30)'
new_spawn = 'gui_robot_spawn(5.0f, ftc_robot_rest_height(), 5.0f, MOTOR_GB_5203_30)'

if old_spawn in content:
    content = content.replace(old_spawn, new_spawn)
    # Also update the message
    content = content.replace(
        '"/robot/%d created at (0.0, %.2f, 0.0)',
        '"/robot/%d created at (5.0, %.2f, 5.0)'
    )
    with open(path, 'w') as f:
        f.write(content)
    print("  [OK] debug_terminal.c: spawn moved to (5, y, 5)")
else:
    print("  [SKIP] spawn position already changed or pattern not found")
PYEOF

echo ""
echo "[PASS] 106: Drive keys = G/V/B/N, spawn at (5, y, 5)"
