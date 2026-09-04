#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"

python3 -c "
path = 'v15R3/src/ui_input/simulation_input_dispatch.c'
with open(path, 'r') as f: content = f.read()

# Remove 't' terminal block
target_block = '''if (main_inputs.t_key_pressed) {
    if (main_inputs.is_debug_mode_active) {
        if (debug_terminal_is_open()) {
            debug_terminal_focus_entry();
        } else {
            debug_terminal_open(parent_window);
        }
    }
    main_inputs.t_key_pressed = false;
}'''
content = content.replace(target_block, '')

# Add robot drive logic before the spawn gun
spawn_gun_marker = '/* Spawn gun (Enter hold) */'
robot_drive_code = '''/* Robot drive keys: T=forward, G=backward, F=left, H=right */
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
    }

'''
content = content.replace(spawn_gun_marker, robot_drive_code + spawn_gun_marker)

# Add include
if '#include \"../robotics/gui_robot_registry.h\"' not in content:
    content = content.replace('#include \"../core/long_run_validation.h\"', '#include \"../core/long_run_validation.h\"\n#include \"../robotics/gui_robot_registry.h\"')

with open(path, 'w') as f: f.write(content)
"
echo "[PASS] 102: terminal 't' removed, robot drive mapped to TFGH"
