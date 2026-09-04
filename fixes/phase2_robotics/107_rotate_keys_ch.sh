#!/usr/bin/env bash
# ============================================================
# FIX 107 — Robot rotation keys: C=left, H=right
#   Q/E are taken by other systems.
#   G=forward, V=backward, B=left, N=right (unchanged)
#   C=rotate left, H=rotate right (new)
# ============================================================
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"

H="v15R2/src/ui_input/input_control.h"
C="v15R2/src/ui_input/input_control.c"
D="v15R2/src/ui_input/simulation_input_dispatch.c"

# ============================================================
# PART 1: Add c_key_pressed to input_control.h
# (h_key_pressed already exists)
# ============================================================
python3 - "$H" << 'PYEOF'
import sys
path = sys.argv[1]
with open(path, 'r') as f:
    content = f.read()

if 'c_key_pressed' not in content:
    content = content.replace(
        'h_key_pressed;',
        'h_key_pressed, c_key_pressed;'
    )
    with open(path, 'w') as f:
        f.write(content)
    print("  [OK] input_control.h: added c_key_pressed")
else:
    print("  [SKIP] c_key_pressed already present")
PYEOF

# ============================================================
# PART 2: Wire C key in input_control.c
# (H key should already be wired from fix 101)
# ============================================================
python3 - "$C" << 'PYEOF'
import sys
path = sys.argv[1]
with open(path, 'r') as f:
    content = f.read()

if 'c_key_pressed' not in content:
    # Init (both initialize_input and on_focus_out)
    content = content.replace(
        'input_state -> h_key_pressed = false;',
        'input_state -> h_key_pressed = false;\n    input_state -> c_key_pressed = false;'
    )
    # Keypress
    content = content.replace(
        'if (event -> keyval == GDK_KEY_h) {input_state -> h_key_pressed = true;}',
        'if (event -> keyval == GDK_KEY_h) {input_state -> h_key_pressed = true;}\n    if (event -> keyval == GDK_KEY_c) {input_state -> c_key_pressed = true;}'
    )
    # Keyrelease
    content = content.replace(
        'if (event -> keyval == GDK_KEY_h) {input_state -> h_key_pressed = false;}',
        'if (event -> keyval == GDK_KEY_h) {input_state -> h_key_pressed = false;}\n    if (event -> keyval == GDK_KEY_c) {input_state -> c_key_pressed = false;}'
    )
    with open(path, 'w') as f:
        f.write(content)
    print("  [OK] input_control.c: wired c_key_pressed")
else:
    print("  [SKIP] c_key_pressed already wired")
PYEOF

# ============================================================
# PART 3: Update rotation keys in simulation_input_dispatch.c
# Replace Q/E with C/H
# ============================================================
python3 - "$D" << 'PYEOF'
import sys
path = sys.argv[1]
with open(path, 'r') as f:
    content = f.read()

# Replace rotation key assignments
content = content.replace(
    'if (main_inputs.e_key_pressed) kb_rotate += 1.0f;',
    'if (main_inputs.h_key_pressed) kb_rotate += 1.0f; /* H=rotate right */'
)
content = content.replace(
    'if (main_inputs.q_key_pressed) kb_rotate -= 1.0f;',
    'if (main_inputs.c_key_pressed) kb_rotate -= 1.0f; /* C=rotate left */'
)

# Update the key-clearing section
content = content.replace(
    'main_inputs.e_key_pressed = false;',
    'main_inputs.h_key_pressed = false;'
)
content = content.replace(
    'main_inputs.q_key_pressed = false;',
    'main_inputs.c_key_pressed = false;'
)

# Update comment
content = content.replace(
    '/* Robot drive keys: G=forward, B=backward, V=left, N=right */',
    '/* Robot drive keys: G=forward, B=backward, V=left, N=right, C=rotL, H=rotR */'
)

with open(path, 'w') as f:
    f.write(content)
print("  [OK] simulation_input_dispatch.c: rotation keys changed to C/H")
PYEOF

echo ""
echo "[PASS] 107: Robot rotation = C (left) + H (right)"
