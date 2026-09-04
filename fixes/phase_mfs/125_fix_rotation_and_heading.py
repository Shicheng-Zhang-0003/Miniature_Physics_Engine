#!/usr/bin/env python3
"""
MFS 125: Fix C/H rotation inversion + add heading indicator
=============================================================
1. Swap C/H rotation signs in simulation_input_dispatch.c
2. Add a small orange "nose" sphere proxy at the front of the chassis
   so the user can see which direction the robot is facing.

Usage:
  cd <project_root>
  python3 fixes/phase_mfs/125_fix_rotation_and_heading.py [--dry-run]
"""

import sys
import subprocess
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
ROOT = SCRIPT_DIR.parent.parent
SRC = ROOT / "v15R2" / "src"
TOOLS = ROOT / "tools"

sys.path.insert(0, str(TOOLS))
from refactor import Refactor, RefactorError

DRY_RUN = "--dry-run" in sys.argv


def log(msg):
    print(f"  [125] {msg}")


def run_build():
    result = subprocess.run(
        [sys.executable, str(TOOLS / "build_check.py"), "--quick"],
        cwd=str(ROOT), capture_output=True, text=True, timeout=120
    )
    if result.returncode != 0:
        print(result.stdout[-2000:] if result.stdout else "")
        print(result.stderr[-2000:] if result.stderr else "")
        return False
    return True


def run_tests():
    result = subprocess.run(
        [sys.executable, str(TOOLS / "test_runner.py")],
        cwd=str(ROOT), capture_output=True, text=True, timeout=300
    )
    print(result.stdout[-3000:] if result.stdout else "")
    if result.returncode != 0:
        print(result.stderr[-1000:] if result.stderr else "")
        return False
    return True


# ============================================================
# STEP 1: Fix C/H rotation inversion
# ============================================================
def step_fix_rotation():
    """Swap C/H rotation signs in simulation_input_dispatch.c."""
    log("Step 1: Fixing C/H rotation inversion")
    path = SRC / "ui_input" / "simulation_input_dispatch.c"
    content = path.read_text()

    # Current (inverted):
    #   C: kb_rotate += 1.0  (should be -= for left)
    #   H: kb_rotate -= 1.0  (should be += for right)
    #
    # The mecanum IK produces counter-clockwise rotation for positive rotate.
    # C = left = counter-clockwise = negative rotate
    # H = right = clockwise = positive rotate

    old_c = 'if (main_inputs.c_key_pressed) { kb_rotate  += 1.0f; }'
    new_c = 'if (main_inputs.c_key_pressed) { kb_rotate  -= 1.0f; } /* MFS_125: C=rotate left (CCW) */'

    old_h = 'if (main_inputs.h_key_pressed) { kb_rotate  -= 1.0f; }'
    new_h = 'if (main_inputs.h_key_pressed) { kb_rotate  += 1.0f; } /* MFS_125: H=rotate right (CW) */'

    changed = False
    if old_c in content:
        content = content.replace(old_c, new_c)
        changed = True
        log("  [OK] C: += → -= (left/CCW)")
    else:
        log("  [WARN] C pattern not found")

    if old_h in content:
        content = content.replace(old_h, new_h)
        changed = True
        log("  [OK] H: -= → += (right/CW)")
    else:
        log("  [WARN] H pattern not found")

    if changed:
        if not DRY_RUN:
            path.write_text(content)
        return True
    return True


# ============================================================
# STEP 2: Add heading indicator to gui_robot_registry.h
# ============================================================
def step_add_heading_header():
    """Add nose_proxy field to gui_robot_proxy struct."""
    log("Step 2: Adding heading indicator to gui_robot_registry.h")
    path = SRC / "robotics" / "gui_robot_registry.h"
    content = path.read_text()

    if "nose_proxy" in content:
        log("  [SKIP] nose_proxy already present")
        return True

    # Add nose_proxy to the gui_robot_proxy struct
    old_struct = """typedef struct {
int chassis_proxy;          /* index in obj_per_scene, -1 if none */
int wheel_proxies[FTC_MAX_WHEELS];
} gui_robot_proxy;"""

    new_struct = """typedef struct {
int chassis_proxy;          /* index in obj_per_scene, -1 if none */
int wheel_proxies[FTC_MAX_WHEELS];
int nose_proxy;             /* MFS_125: heading indicator at front of chassis */
} gui_robot_proxy;"""

    if old_struct in content:
        content = content.replace(old_struct, new_struct)
        if not DRY_RUN:
            path.write_text(content)
        log("  [OK] Added nose_proxy to gui_robot_proxy struct")
        return True

    # Fallback: try to find the struct and add the field
    if "int wheel_proxies[FTC_MAX_WHEELS];" in content:
        content = content.replace(
            "int wheel_proxies[FTC_MAX_WHEELS];",
            "int wheel_proxies[FTC_MAX_WHEELS];\nint nose_proxy;             /* MFS_125: heading indicator */"
        )
        if not DRY_RUN:
            path.write_text(content)
        log("  [OK] Added nose_proxy (fallback)")
        return True

    log("  [WARN] Could not find gui_robot_proxy struct")
    return False


# ============================================================
# STEP 3: Add heading indicator to gui_robot_registry.c
# ============================================================
def step_add_heading_impl():
    """Add nose sphere creation and sync to gui_robot_registry.c."""
    log("Step 3: Adding heading indicator to gui_robot_registry.c")
    path = SRC / "robotics" / "gui_robot_registry.c"
    content = path.read_text()

    if "nose_proxy" in content:
        log("  [SKIP] nose_proxy already present")
        return True

    # --- Part A: Initialize nose_proxy to -1 in gui_robot_spawn ---
    old_init = """proxy->chassis_proxy = -1;
for (int i = 0; i < FTC_MAX_WHEELS; i++) {
proxy->wheel_proxies[i] = -1;
}"""

    new_init = """proxy->chassis_proxy = -1;
proxy->nose_proxy = -1; /* MFS_125 */
for (int i = 0; i < FTC_MAX_WHEELS; i++) {
proxy->wheel_proxies[i] = -1;
}"""

    if old_init in content:
        content = content.replace(old_init, new_init)
        log("  [OK] Initialized nose_proxy = -1")
    else:
        log("  [WARN] Could not find proxy init block")

    # --- Part B: Create nose sphere after chassis proxy creation ---
    # Insert after the chassis proxy creation block
    old_chassis_end = """proxy->chassis_proxy = proxy_idx;
}
}"""

    new_chassis_end = """proxy->chassis_proxy = proxy_idx;
}
}
/* MFS_125: Create heading indicator (orange nose) at front of chassis */
{
/* Front of chassis is at +Z (forward direction based on mecanum IK).
* Place a small orange sphere slightly in front of the chassis. */
vector3 nose_offset = {0.0f, 0.0f, 0.28f}; /* slightly beyond chassis half_z=0.225 */
vector3 nose_pos = vector3_addition(src->position,
vector4_rotate_to_vector3(src->orientation, nose_offset));
int nose_idx = scene_add_object(0.03f, 0.0f, nose_pos);
if (nose_idx >= 0) {
obj_per_scene[nose_idx].colour = (vector3){1.0f, 0.5f, 0.0f}; /* orange */
obj_per_scene[nose_idx].static_state = true;
obj_per_scene[nose_idx].inverse_mass = 0.0f;
proxy->nose_proxy = nose_idx;
}
}"""

    if old_chassis_end in content:
        content = content.replace(old_chassis_end, new_chassis_end, 1)
        log("  [OK] Added nose sphere creation")
    else:
        log("  [WARN] Could not find chassis proxy end block, trying alternate")
        # Try alternate pattern
        if "proxy->chassis_proxy = proxy_idx;" in content:
            content = content.replace(
                "proxy->chassis_proxy = proxy_idx;",
                """proxy->chassis_proxy = proxy_idx;
}
}
/* MFS_125: Create heading indicator (orange nose) at front of chassis */
{
vector3 nose_offset = {0.0f, 0.0f, 0.28f};
vector3 nose_pos = vector3_addition(src->position,
vector4_rotate_to_vector3(src->orientation, nose_offset));
int nose_idx = scene_add_object(0.03f, 0.0f, nose_pos);
if (nose_idx >= 0) {
obj_per_scene[nose_idx].colour = (vector3){1.0f, 0.5f, 0.0f};
obj_per_scene[nose_idx].static_state = true;
obj_per_scene[nose_idx].inverse_mass = 0.0f;
proxy->nose_proxy = nose_idx;
}""",
                1
            )
            log("  [OK] Added nose sphere creation (alternate)")

    # --- Part C: Sync nose position in gui_robot_tick ---
    # Add nose sync after the wheel sync loop
    old_sync_end = """/* Sync wheels */
for (int w = 0; w < robot->wheel_count; w++) {
int proxy_idx = proxy->wheel_proxies[w];
if ((proxy_idx >= 0) && (proxy_idx < object_count)) {
int wheel_body = robot->wheel_bodies[w];
if ((wheel_body >= 0) && (wheel_body < mfs_gui_robot_world->body_count)) {
rigidbody *src = &mfs_gui_robot_world->bodies[wheel_body];
rigidbody *dst = &obj_per_scene[proxy_idx];
dst->position = src->position;
dst->orientation = src->orientation;
rigidbody_update_axes(dst);
}
}
}
}
}
}"""

    new_sync_end = """/* Sync wheels */
for (int w = 0; w < robot->wheel_count; w++) {
int proxy_idx = proxy->wheel_proxies[w];
if ((proxy_idx >= 0) && (proxy_idx < object_count)) {
int wheel_body = robot->wheel_bodies[w];
if ((wheel_body >= 0) && (wheel_body < mfs_gui_robot_world->body_count)) {
rigidbody *src = &mfs_gui_robot_world->bodies[wheel_body];
rigidbody *dst = &obj_per_scene[proxy_idx];
dst->position = src->position;
dst->orientation = src->orientation;
rigidbody_update_axes(dst);
}
}
}
/* MFS_125: Sync heading indicator (nose) */
if ((proxy->nose_proxy >= 0) && (proxy->nose_proxy < object_count)) {
int chassis_body = robot->chassis_body;
if ((chassis_body >= 0) && (chassis_body < mfs_gui_robot_world->body_count)) {
rigidbody *chassis = &mfs_gui_robot_world->bodies[chassis_body];
vector3 nose_offset = {0.0f, 0.0f, 0.28f};
vector3 nose_world = vector3_addition(chassis->position,
vector4_rotate_to_vector3(chassis->orientation, nose_offset));
obj_per_scene[proxy->nose_proxy].position = nose_world;
}
}
}
}
}"""

    if old_sync_end in content:
        content = content.replace(old_sync_end, new_sync_end, 1)
        log("  [OK] Added nose sync to gui_robot_tick")
    else:
        log("  [WARN] Could not find sync end block, trying line-based insert")
        # Fallback: insert nose sync after the wheel sync closing brace
        # Find the last occurrence of the wheel sync pattern and add after it
        lines = content.split('\n')
        insert_idx = -1
        for i in range(len(lines) - 1, -1, -1):
            if 'rigidbody_update_axes(dst);' in lines[i] and i > 0:
                # Find the closing braces after this
                brace_count = 0
                for j in range(i, min(i + 10, len(lines))):
                    if '}' in lines[j]:
                        brace_count += lines[j].count('}')
                    if brace_count >= 3:
                        insert_idx = j + 1
                        break
                break
        if insert_idx > 0:
            nose_sync = """        /* MFS_125: Sync heading indicator (nose) */
        if ((proxy->nose_proxy >= 0) && (proxy->nose_proxy < object_count)) {
            int chassis_body = robot->chassis_body;
            if ((chassis_body >= 0) && (chassis_body < mfs_gui_robot_world->body_count)) {
                rigidbody *chassis = &mfs_gui_robot_world->bodies[chassis_body];
                vector3 nose_offset = {0.0f, 0.0f, 0.28f};
                vector3 nose_world = vector3_addition(chassis->position,
                    vector4_rotate_to_vector3(chassis->orientation, nose_offset));
                obj_per_scene[proxy->nose_proxy].position = nose_world;
            }
        }"""
            lines.insert(insert_idx, nose_sync)
            content = '\n'.join(lines)
            log("  [OK] Added nose sync (line-based)")

    if not DRY_RUN:
        path.write_text(content)
    return True


# ============================================================
# MAIN
# ============================================================
def main():
    print("=" * 60)
    print("MFS 125: Fix Rotation + Add Heading Indicator")
    print("=" * 60)

    if DRY_RUN:
        print("  ** DRY RUN — no files will be modified **")
        print()

    if not SRC.exists():
        print(f"FATAL: Source directory not found: {SRC}")
        return 1

    steps = [
        ("Fix C/H rotation", step_fix_rotation),
        ("Heading header", step_add_heading_header),
        ("Heading impl", step_add_heading_impl),
    ]

    for name, func in steps:
        try:
            if not func():
                print(f"\n[FAIL] Step '{name}' failed. Aborting.")
                return 1
        except RefactorError as e:
            print(f"\n[FAIL] Step '{name}' raised RefactorError: {e}")
            return 1
        except Exception as e:
            print(f"\n[FAIL] Step '{name}' raised unexpected error: {e}")
            import traceback
            traceback.print_exc()
            return 1

    print()

    if not DRY_RUN:
        log("Running build verification...")
        if not run_build():
            log("[FAIL] Build failed. Review errors above.")
            return 1
        log("[PASS] Build successful!")

        log("Running headless tests...")
        if not run_tests():
            log("[WARN] Some tests failed. Review output above.")
        else:
            log("[PASS] All tests passed!")
    else:
        log("[DRY RUN] Skipping build verification.")

    print()
    print("=" * 60)
    print("  DONE. Summary of changes:")
    print("    1. C = rotate left (CCW), H = rotate right (CW)")
    print("    2. Orange nose sphere at front of chassis shows heading")
    print("       Front direction = +Z (forward based on mecanum IK)")
    print("=" * 60)
    return 0


if __name__ == "__main__":
    sys.exit(main())
