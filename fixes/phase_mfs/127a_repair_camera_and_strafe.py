#!/usr/bin/env python3
"""
MFS 127a — Repair: camera float + strafe diagnostics
=====================================================
Repairs the failed 127 script:
1. Fixes camera float in debug mode (correct file path: core/simulation_camera.c)
2. Adds strafe diagnostics to collision_mechanics.c (correct anchor)

Usage:
  cd <project_root>
  python3 fixes/phase_mfs/127a_repair_camera_and_strafe.py [--dry-run]
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
    print(f"  [127a] {msg}")


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
    return result.returncode == 0


def step_fix_camera_float():
    """Fix camera float in debug mode by resetting vertical_velocity."""
    log("Step 1: Fixing camera debug fly mode (correct path)")
    path = SRC / "core" / "simulation_camera.c"
    content = path.read_text()

    if "MFS_127_CAMERA_FLOAT_FIX" in content:
        log("  [SKIP] Camera float fix already present")
        return True

    # The camera float issue: when entering debug mode, the camera's
    # vertical_velocity has a residual value from game mode.
    # Fix: reset vertical_velocity to 0 at the start of debug mode section.

    # Find the debug mode section and add vertical_velocity reset
    anchor = "if (main_inputs.is_debug_mode_active) {"

    if anchor in content:
        r = Refactor(str(path))
        r.replace(
            old=anchor,
            new="""/* MFS_127_CAMERA_FLOAT_FIX: Reset vertical velocity when entering debug mode
     * to prevent camera from floating upward due to residual game-mode velocity. */
    if (main_inputs.is_debug_mode_active) {
        /* Reset vertical velocity if not actively controlled */
        if (!main_inputs.space_key_pressed && !main_inputs.shift_key_pressed) {
            main_camera_fov.vertical_velocity = 0.0f;
        }""",
            label="Fix camera float in debug mode"
        )
        if not DRY_RUN:
            r.apply(dry_run=False)
        else:
            print(r.diff())
        log("  [OK] Camera float fix applied")
        return True
    else:
        log("  [WARN] Could not find debug mode anchor")
        return False


def step_add_strafe_diagnostics():
    """Add strafe diagnostics to collision_mechanics.c with correct anchor."""
    log("Step 2: Adding strafe diagnostics (correct anchor)")
    path = SRC / "physics" / "collision_mechanics.c"
    content = path.read_text()

    if "MFS_127_STRAFE_DIAG" in content:
        log("  [SKIP] Strafe diagnostics already present")
        return True

    # Find the mecanum friction section - look for the roller angle code
    anchor = "if (mecanum_wheel && mecanum_wheel->type == object_cylinder) {"

    if anchor in content:
        # Add diagnostics after the mecanum tangent computation
        diag_code = """
/* MFS_127_STRAFE_DIAG: Conditional diagnostics for mecanum strafe debugging.
 * Compile with -DMFS_DEBUG_STRAFE to enable. */
#ifdef MFS_DEBUG_STRAFE
{
    static int strafe_diag_counter = 0;
    if ((strafe_diag_counter++ % 60) == 0) {
        float grip_len = vector3_length(grip_dir);
        printf("[STRAFE_DIAG] roller_angle=%.2f rad grip_len=%.4f mecanum=%d\\n",
               mecanum_wheel->roller_angle_rad, grip_len, mecanum_wheel->is_mecanum ? 1 : 0);
    }
}
#endif
"""
        # Find the end of the mecanum tangent computation block
        # Look for the closing brace of the axle_proj_len check
        roller_anchor = "if (axle_proj_len > 0.0001f) {"

        if roller_anchor in content:
            # Find the position after the axle_proj_len block
            roller_pos = content.find(roller_anchor)
            # Find the closing brace of this block
            brace_count = 0
            pos = roller_pos
            found_open = False
            while pos < len(content):
                if content[pos] == '{':
                    brace_count += 1
                    found_open = True
                elif content[pos] == '}':
                    brace_count -= 1
                    if found_open and brace_count == 0:
                        # Found the closing brace of the axle_proj_len block
                        # Insert diagnostics after this
                        insert_pos = pos + 1
                        content = content[:insert_pos] + diag_code + content[insert_pos:]
                        break
                pos += 1

            if not DRY_RUN:
                path.write_text(content)
            log("  [OK] Strafe diagnostics added")
            return True
        else:
            log("  [WARN] Could not find axle_proj_len anchor")
            return False
    else:
        log("  [WARN] Could not find mecanum wheel anchor")
        return False


def main():
    print("=" * 60)
    print("MFS 127a: Repair Camera Float + Strafe Diagnostics")
    print("=" * 60)

    if DRY_RUN:
        print("  ** DRY RUN — no files will be modified **")
        print()

    if not SRC.exists():
        print(f"FATAL: Source directory not found: {SRC}")
        return 1

    steps = [
        ("Camera float fix", step_fix_camera_float),
        ("Strafe diagnostics", step_add_strafe_diagnostics),
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
    print("  127a repair complete.")
    print("  Camera float: vertical_velocity reset when entering debug mode")
    print("  Strafe diagnostics: compile with -DMFS_DEBUG_STRAFE to enable")
    print("=" * 60)
    return 0


if __name__ == "__main__":
    sys.exit(main())
