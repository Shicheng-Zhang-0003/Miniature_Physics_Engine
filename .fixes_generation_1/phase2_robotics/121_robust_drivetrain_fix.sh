#!/usr/bin/env python3
import re
import subprocess
import os

os.chdir("v15R3/src")
path = 'robotics/drivetrain.c'

with open(path, 'r') as f:
    src = f.read()

# 1. Fix friction (hardcode to 0.8f since mpe_config_t doesn't have physics.friction_static)
if 'g_cfg.physics.friction_static' in src:
    src = src.replace('g_cfg.physics.friction_static', '0.8f')
    print("[OK] 1: Replaced g_cfg.physics.friction_static with 0.8f")
else:
    print("[SKIP] g_cfg.physics.friction_static not found")

# 2. Fix mecanum_chassis_force (multiline assignment)
start_idx = src.find('robot->mecanum_chassis_force')
if start_idx != -1:
    end_idx = src.find(';', start_idx)
    if end_idx != -1:
        old_assignment = src[start_idx:end_idx+1]
        new_assignment = 'robot->mecanum_chassis_force = (vector3) {-strafe * force_scale, 0.0f, 0.0f}; /* FIX 117 */'
        src = src.replace(old_assignment, new_assignment)
        print("[OK] 2: mecanum_chassis_force assignment replaced (forward=0, strafe flipped)")
else:
    print("[WARN] mecanum_chassis_force not found")

# 3. Fix torque_scale
torque_match = re.search(r'(const\s+float\s+torque_scale\s*=\s*)[0-9.]+f?', src)
if torque_match:
    src = src.replace(torque_match.group(0), torque_match.group(1) + '30.0f')
    print(f"[OK] 3: torque_scale replaced with 30.0f")
else:
    torque_match2 = re.search(r'(torque_scale\s*=\s*)[0-9.]+f?', src)
    if torque_match2:
        src = src.replace(torque_match2.group(0), torque_match2.group(1) + '30.0f')
        print(f"[OK] 3: torque_scale replaced with 30.0f (fallback)")
    else:
        print("[WARN] torque_scale not found")

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
