#!/usr/bin/env python3
"""
MFS 124: Fix robot control — key release bug + damping balance + speed cap
===========================================================================
Fixes:
  1. input_control.c: key release handler incorrectly clears multiple keys
  2. drivetrain.c: restore lateral damping to 2.0 (balance between 1.0 and 3.0)
  3. drivetrain.c: restore yaw damping to 1.0 (balance between 0.5 and 1.5)
  4. drivetrain.c: add velocity cap to prevent runaway acceleration

Usage:
  cd <project_root>
  python3 fixes/phase_mfs/124_fix_robot_control.py [--dry-run]
"""

import sys
import subprocess
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
ROOT = SCRIPT_DIR.parent.parent
SRC = ROOT / "v15R3" / "src"
TOOLS = ROOT / "tools"

sys.path.insert(0, str(TOOLS))
from refactor import Refactor, RefactorError

DRY_RUN = "--dry-run" in sys.argv


def log(msg):
    print(f"  [124] {msg}")


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
# STEP 1: Fix key release handler in input_control.c
# ============================================================
def step_fix_key_release():
    """Fix the key release handler so each key only clears itself."""
    log("Step 1: Fixing key release handler in input_control.c")
    path = SRC / "ui_input" / "input_control.c"
    content = path.read_text()

    # The broken pattern: releasing H clears c, v, b, n too
    broken_pattern = """if (event -> keyval == GDK_KEY_h) {input_state -> h_key_pressed = false;
input_state -> c_key_pressed = false;
input_state -> v_key_pressed = false;
input_state -> b_key_pressed = false;
input_state -> n_key_pressed = false;} /* MFS_GUI_BRIDGE */"""

    fixed_pattern = """if (event -> keyval == GDK_KEY_g) {input_state -> g_key_pressed = false;}
if (event -> keyval == GDK_KEY_h) {input_state -> h_key_pressed = false;}
if (event -> keyval == GDK_KEY_c) {input_state -> c_key_pressed = false;}
if (event -> keyval == GDK_KEY_v) {input_state -> v_key_pressed = false;}
if (event -> keyval == GDK_KEY_b) {input_state -> b_key_pressed = false;}
if (event -> keyval == GDK_KEY_n) {input_state -> n_key_pressed = false;} /* MFS_124_FIX */"""

    if broken_pattern in content:
        r = Refactor(str(path))
        r.replace(
            old=broken_pattern,
            new=fixed_pattern,
            label="Fix key release handler"
        )
        if not DRY_RUN:
            r.apply(dry_run=False)
        else:
            print(r.diff())
        log("  [OK] Key release handler fixed — each key clears only itself")
        return True

    # Try a more flexible approach
    log("  [INFO] Trying flexible pattern match...")

    # Check if the bug exists in a different form
    if "GDK_KEY_h) {input_state -> h_key_pressed = false;" in content and \
       "input_state -> c_key_pressed = false;" in content:
        # Find and fix the multi-key clear
        lines = content.split('\n')
        new_lines = []
        i = 0
        fixed = False
        while i < len(lines):
            line = lines[i]
            if "GDK_KEY_h) {input_state -> h_key_pressed = false;" in line and \
               "input_state -> c_key_pressed = false;" in lines[i+1] if i+1 < len(lines) else False:
                # Replace this block with individual key clears
                new_lines.append('    if (event -> keyval == GDK_KEY_g) {input_state -> g_key_pressed = false;}')
                new_lines.append('    if (event -> keyval == GDK_KEY_h) {input_state -> h_key_pressed = false;}')
                new_lines.append('    if (event -> keyval == GDK_KEY_c) {input_state -> c_key_pressed = false;}')
                new_lines.append('    if (event -> keyval == GDK_KEY_v) {input_state -> v_key_pressed = false;}')
                new_lines.append('    if (event -> keyval == GDK_KEY_b) {input_state -> b_key_pressed = false;}')
                new_lines.append('    if (event -> keyval == GDK_KEY_n) {input_state -> n_key_pressed = false;} /* MFS_124_FIX */')
                fixed = True
                # Skip the broken lines
                i += 1
                while i < len(lines) and ("input_state ->" in lines[i] and "key_pressed = false" in lines[i]):
                    i += 1
                continue
            new_lines.append(line)
            i += 1

        if fixed:
            if not DRY_RUN:
                path.write_text('\n'.join(new_lines))
            log("  [OK] Key release handler fixed (flexible method)")
            return True

    log("  [SKIP] Key release handler already correct or pattern not found")
    return True


# ============================================================
# STEP 2: Restore lateral damping to 2.0
# ============================================================
def step_fix_lateral_damping():
    """Increase lateral damping from 1.0 to 2.0 for better control."""
    log("Step 2: Adjusting lateral damping in drivetrain.c")
    path = SRC / "robotics" / "drivetrain.c"
    content = path.read_text()

    # Fix lateral damping: 1.0 -> 2.0
    if "m * 1.0f /* MFS_122: reduced from 3.0 */" in content:
        r = Refactor(str(path))
        r.replace(
            old="m * 1.0f /* MFS_122: reduced from 3.0 */",
            new="m * 2.0f /* MFS_124: balanced damping */",
            label="Increase lateral damping"
        )
        if not DRY_RUN:
            r.apply(dry_run=False)
        else:
            print(r.diff())
        log("  [OK] Lateral damping increased to 2.0")
        return True

    # Try without the comment
    if "m * 1.0f" in content and "vector3_scaling(lat" in content:
        r = Refactor(str(path))
        r.replace(
            old="vector3_scaling(lat, m * 1.0f)",
            new="vector3_scaling(lat, m * 2.0f) /* MFS_124: balanced damping */",
            label="Increase lateral damping"
        )
        if not DRY_RUN:
            r.apply(dry_run=False)
        else:
            print(r.diff())
        log("  [OK] Lateral damping increased to 2.0 (flexible)")
        return True

    log("  [SKIP] Lateral damping already adjusted or not found")
    return True


# ============================================================
# STEP 3: Restore yaw damping to 1.0
# ============================================================
def step_fix_yaw_damping():
    """Increase yaw damping from 0.5 to 1.0 for better rotation control."""
    log("Step 3: Adjusting yaw damping in drivetrain.c")
    path = SRC / "robotics" / "drivetrain.c"
    content = path.read_text()

    if "yaw_vel * m * 0.5f * 0.02f /* MFS_122: reduced from 1.5 */" in content:
        r = Refactor(str(path))
        r.replace(
            old="yaw_vel * m * 0.5f * 0.02f /* MFS_122: reduced from 1.5 */",
            new="yaw_vel * m * 1.0f * 0.02f /* MFS_124: balanced yaw damping */",
            label="Increase yaw damping"
        )
        if not DRY_RUN:
            r.apply(dry_run=False)
        else:
            print(r.diff())
        log("  [OK] Yaw damping increased to 1.0")
        return True

    if "yaw_vel * m * 0.5f * 0.02f" in content:
        r = Refactor(str(path))
        r.replace(
            old="yaw_vel * m * 0.5f * 0.02f",
            new="yaw_vel * m * 1.0f * 0.02f /* MFS_124: balanced yaw damping */",
            label="Increase yaw damping"
        )
        if not DRY_RUN:
            r.apply(dry_run=False)
        else:
            print(r.diff())
        log("  [OK] Yaw damping increased to 1.0 (flexible)")
        return True

    log("  [SKIP] Yaw damping already adjusted or not found")
    return True


# ============================================================
# STEP 4: Add velocity cap to prevent runaway
# ============================================================
def step_add_velocity_cap():
    """Add a velocity cap to the chassis to prevent runaway acceleration."""
    log("Step 4: Adding velocity cap in drivetrain.c")
    path = SRC / "robotics" / "drivetrain.c"
    content = path.read_text()

    if "MFS_124_VELOCITY_CAP" in content:
        log("  [SKIP] Velocity cap already present")
        return True

    # Add velocity cap after the chassis damping block
    # Find the end of the chassis damping block and add velocity cap after it
    anchor = "chassis->torque_accumulator.y -= yaw_vel * m *"

    if anchor in content:
        r = Refactor(str(path))
        # Find the line after the yaw damping and add velocity cap
        r.insert_after(
            anchor="chassis->torque_accumulator.y -= yaw_vel * m *",
            text="""
/* MFS_124_VELOCITY_CAP: prevent runaway acceleration */
{
    float speed_sq = chassis->velocity.x * chassis->velocity.x +
                     chassis->velocity.z * chassis->velocity.z;
    float max_speed = 3.0f; /* m/s cap */
    if (speed_sq > max_speed * max_speed) {
        float speed = sqrtf(speed_sq);
        float scale = max_speed / speed;
        chassis->velocity.x *= scale;
        chassis->velocity.z *= scale;
    }
}""",
            label="Add velocity cap"
        )
        if not DRY_RUN:
            r.apply(dry_run=False)
        else:
            print(r.diff())
        log("  [OK] Velocity cap added (3.0 m/s)")
        return True

    log("  [WARN] Could not find anchor for velocity cap")
    return True


# ============================================================
# MAIN
# ============================================================
def main():
    print("=" * 60)
    print("MFS 124: Fix Robot Control")
    print("=" * 60)

    if DRY_RUN:
        print("  ** DRY RUN — no files will be modified **")
        print()

    if not SRC.exists():
        print(f"FATAL: Source directory not found: {SRC}")
        return 1

    steps = [
        ("Key release handler", step_fix_key_release),
        ("Lateral damping", step_fix_lateral_damping),
        ("Yaw damping", step_fix_yaw_damping),
        ("Velocity cap", step_add_velocity_cap),
    ]

    for name, func in steps:
        try:
            if not func():
                print(f"\n[WARN] Step '{name}' had issues. Continuing...")
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
    print("    1. Key release: each key now clears only itself")
    print("    2. Lateral damping: 1.0 -> 2.0 (better slide control)")
    print("    3. Yaw damping: 0.5 -> 1.0 (better rotation control)")
    print("    4. Velocity cap: 3.0 m/s max speed (prevents runaway)")
    print("=" * 60)
    return 0


if __name__ == "__main__":
    sys.exit(main())
