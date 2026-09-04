#!/usr/bin/env python3
import re
import subprocess
import os

os.chdir("v15R2/src")
path = 'robotics/drivetrain.c'

with open(path, 'r') as f:
    src = f.read()

# 1. Fix the math3D function name (vector3_cross_product -> vector3_cross)
if 'vector3_cross_product' in src:
    src = src.replace('vector3_cross_product', 'vector3_cross')
    print("[OK] 1: Replaced vector3_cross_product with vector3_cross")
else:
    print("[SKIP] vector3_cross_product not found")

# 2. Zero the forward cheat and flip strafe sign
old_force = """robot->mecanum_chassis_force = (vector3) {strafe * force_scale,
                                          0.0f,
                                          forward * force_scale * 0.5f /* forward partly via wheels */};"""

new_force = """robot->mecanum_chassis_force = (vector3) {-strafe * force_scale,
                                          0.0f,
                                          0.0f /* forward via real traction (FIX 117) */};"""

if old_force in src:
    src = src.replace(old_force, new_force)
    print("[OK] 2 & 3: mecanum_chassis_force updated (forward zeroed, strafe flipped)")
else:
    # Fallback to a looser regex
    pattern = r'robot->mecanum_chassis_force\s*=\s*\(vector3\)\s*\{[^}]*forward[^}]*\};'
    if re.search(pattern, src, re.DOTALL):
        src = re.sub(pattern, new_force, src, flags=re.DOTALL)
        print("[OK] 2 & 3: mecanum_chassis_force updated via regex")
    else:
        print("[WARN] Could not find mecanum_chassis_force assignment")

# 3. Raise torque_scale
old_torque = "const float torque_scale = 8.0f;"
new_torque = "const float torque_scale = 30.0f; /* FIX 117: overpowers contact asymmetry */"
if old_torque in src:
    src = src.replace(old_torque, new_torque)
    print("[OK] 4: torque_scale raised to 30.0f")
else:
    # Fallback regex
    pattern = r'(const\s+float\s+torque_scale\s*=\s*)[0-9.]+(f?\s*;)'
    src_new, n = re.subn(pattern, r'\g<1>30.0f\2 /* FIX 117 */', src)
    if n > 0:
        src = src_new
        print(f"[OK] 4: torque_scale updated via regex ({n} matches)")
    else:
        print("[WARN] Could not find torque_scale")

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
