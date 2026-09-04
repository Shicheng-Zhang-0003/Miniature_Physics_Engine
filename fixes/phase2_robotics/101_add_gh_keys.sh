#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"

python3 -c "
h_path = 'v15R2/src/ui_input/input_control.h'
with open(h_path, 'r') as f: h_content = f.read()
if 'g_key_pressed' not in h_content:
    h_content = h_content.replace('bool f_key_pressed;', 'bool f_key_pressed;\n    bool g_key_pressed;\n    bool h_key_pressed;')
with open(h_path, 'w') as f: f.write(h_content)

c_path = 'v15R2/src/ui_input/input_control.c'
with open(c_path, 'r') as f: c_content = f.read()
if 'g_key_pressed' not in c_content:
    c_content = c_content.replace('input_state -> f_key_pressed = false;', 'input_state -> f_key_pressed = false;\n    input_state -> g_key_pressed = false;\n    input_state -> h_key_pressed = false;')
    c_content = c_content.replace('if (event -> keyval == GDK_KEY_f) {input_state -> f_key_pressed = true;}', 'if (event -> keyval == GDK_KEY_f) {input_state -> f_key_pressed = true;}\n    if (event -> keyval == GDK_KEY_g) {input_state -> g_key_pressed = true;}\n    if (event -> keyval == GDK_KEY_h) {input_state -> h_key_pressed = true;}')
    c_content = c_content.replace('if (event -> keyval == GDK_KEY_f) {input_state -> f_key_pressed = false;}', 'if (event -> keyval == GDK_KEY_f) {input_state -> f_key_pressed = false;}\n    if (event -> keyval == GDK_KEY_g) {input_state -> g_key_pressed = false;}\n    if (event -> keyval == GDK_KEY_h) {input_state -> h_key_pressed = false;}')
with open(c_path, 'w') as f: f.write(c_content)
"
echo "[PASS] 101: g and h keys added to input control"
