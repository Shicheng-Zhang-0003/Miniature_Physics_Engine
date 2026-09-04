#!/usr/bin/env bash
# ============================================================
# FIX 118 — Repair traction block and regex replacements
#
#   1. Dynamically find the exact cross product / rotate / norm
#      function names from math3D.h and patch the traction block.
#   2. Replace the C99 compound literal (vector3){...} with a
#      named variable to avoid parsing/implicit declaration errors.
#   3. Use robust regex with \s+ to find and replace the
#      mecanum_chassis_force assignment and torque_scale regardless
#      of exact whitespace.
# ============================================================
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"
DT="v15R2/src/robotics/drivetrain.c"
MATH="v15R2/src/core/math3D.h"

echo "=== Inspecting math3D.h ==="
grep -iE 'cross|normalisation|rotate_to_vector3' "$MATH" | head -10

echo ""
echo "=== Inspecting drivetrain.c current state ==="
grep -n 'vector3_cross' "$DT" || echo "No vector3_cross found"
grep -n 'mecanum_chassis_force\s*=' "$DT" | head -5
grep -n 'torque_scale' "$DT" | head -5

echo ""
echo "=== Fixing traction block and regex replacements ==="
python3 - "$DT" << 'PYEOF'
import re, sys

path = sys.argv[1]
with open('v15R2/src/core/math3D.h', 'r') as f:
    math3d = f.read()

# Dynamically find the exact function names
cross_match = re.search(r'(?:vector3|math3)\s+(\w*cross\w*)\s*\(', math3d, re.IGNORECASE)
CROSS = cross_match.group(1) if cross_match else "vector3_cross_product"

rot_match = re.search(r'(?:vector3|math3)\s+(\w*rotate_to_vector3\w*)\s*\(', math3d, re.IGNORECASE)
ROTATE = rot_match.group(1) if rot_match else "vector4_rotate_to_vector3"

norm_match = re.search(r'(?:vector3|math3)\s+(\w*normalisation\w*)\s*\(', math3d, re.IGNORECASE)
NORM = norm_match.group(1) if norm_match else "vector3_normalisation"

print(f"Found functions: CROSS={CROSS}, ROTATE={ROTATE}, NORM={NORM}")

with open(path, 'r') as f:
    src = f.read()

# Fix the traction block if it has the wrong function names
src = src.replace('vector3_cross_product', CROSS)
src = src.replace('vector4_rotate_to_vector3', ROTATE)
src = src.replace('vector3_normalisation', NORM)

# Fix the compound literal issue just in case (use a named vector)
src = src.replace('(vector3){1.0f, 0.0f, 0.0f}', 'local_axle')
if 'local_axle' in src and 'vector3 local_axle = {1.0f, 0.0f, 0.0f};' not in src:
    src = src.replace('vector3 axle = ', 'vector3 local_axle = {1.0f, 0.0f, 0.0f};\n            vector3 axle = ')

# Now fix the mecanum_chassis_force assignment using regex
# Match: robot->mecanum_chassis_force = (vector3) { ... };
force_pattern = r'(robot->mecanum_chassis_force\s*=\s*\([^)]+\)\s*\{)([^}]*)(\}\s*;)'
def replace_force(m):
    # We want: {-strafe * force_scale, 0.0f, 0.0f}
    return m.group(1) + '-strafe * force_scale,\n                                          0.0f,\n                                          0.0f /* forward via real traction (FIX 117) */' + m.group(3)

src, n1 = re.subn(force_pattern, replace_force, src, flags=re.DOTALL)
print(f"Replaced mecanum_chassis_force: {n1} time(s)")

# Fix torque_scale using regex
torque_pattern = r'(const\s+float\s+\w*torque_scale\w*\s*=\s*)[0-9.]+(f?\s*;)'
src, n2 = re.subn(torque_pattern, r'\g<1>30.0f\2 /* FIX 117 */', src)
print(f"Replaced torque_scale: {n2} time(s)")

with open(path, 'w') as f:
    f.write(src)

print("[DONE] Traction block repaired")
PYEOF

echo ""
echo "=== Rebuilding ==="
cd v15R2/src
./compile 2>&1 | tail -15
