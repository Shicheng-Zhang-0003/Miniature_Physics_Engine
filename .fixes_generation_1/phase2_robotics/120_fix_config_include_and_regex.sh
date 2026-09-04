#!/usr/bin/env python3
import re
import subprocess
import os

os.chdir("v15R3/src")
path = 'robotics/drivetrain.c'

with open(path, 'r') as f:
    src = f.read()

# 1. Add config include if not present
if '#include "../config/mpe_config.h"' not in src and 'g_cfg.world.gravity' in src:
    src = src.replace('#include "../core/math3D.h"', '#include "../core/math3D.h"\n#include "../config/mpe_config.h"')
    print("[OK] 1: Added mpe_config.h include")
else:
    print("[SKIP] mpe_config.h include already present or not needed")

# 2. Find and replace mecanum_chassis_force
pattern_force = r'robot->mecanum_chassis_force\s*=\s*\(vector3\)\s*\{[^}]*\};'
new_force = 'robot->mecanum_chassis_force = (vector3) {-strafe * force_scale,\n                                          0.0f,\n                                          0.0f /* forward via real traction (FIX 117) */};'
src, n1 = re.subn(pattern_force, new_force, src, flags=re.DOTALL)
print(f"[OK] 2: mecanum_chassis_force replaced ({n1} times)")

# 3. Find and replace torque_scale
pattern_torque = r'const\s+float\s+torque_scale\s*=\s*[0-9.]+f?\s*;'
new_torque = 'const float torque_scale = 30.0f;'
src, n2 = re.subn(pattern_torque, new_torque, src)
print(f"[OK] 3: torque_scale replaced ({n2} times)")

with open(path, 'w') as f:
    f.write(src)

print("\n--- Building ---")
result = subprocess.run(['./compile'], capture_output=True, text=True)
print(result.stdout[-1500:] if result.stdout else "")
print(result.stderr[-1500:] if result.stderr else "")
if result.returncode == 0:
    print("[PASS] Build successful")
else:
    print("[FAIL] Build failed")
