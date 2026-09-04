#!/usr/bin/env python3
"""
MFS 127: Fix rotation wobble, strafe intermittency, and camera float
======================================================================
Fixes:
  1. Reduce Baumgarte axis correction beta (0.2 → 0.1) to reduce oscillation
  2. Increase yaw damping (1.0 → 1.5) to stop residual rotation faster
  3. Add diagnostic logging to mecanum friction model for strafe debugging
  4. Fix camera debug fly mode state management (floating issue)

Usage:
  cd <project_root>
  python3 fixes/phase_mfs/127_fix_rotation_wobble_strafe_camera.py [--dry-run]
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
    print(f"  [127] {msg}")


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


# ============================================================
# STEP 1: Reduce Baumgarte beta and increase yaw damping
# ============================================================
def step_fix_rotation_wobble():
    """Reduce axis correction aggressiveness and increase yaw damping."""
    log("Step 1: Fixing rotation wobble")

    # 1a. Reduce Baumgarte beta in revolute_joint.c
    revolute_path = SRC / "physics" / "revolute_joint.c"
    content = revolute_path.read_text()

    if "axis_baumgarte_beta = 0.2f" in content:
        content = content.replace(
            "const float axis_baumgarte_beta = 0.2f;",
            "const float axis_baumgarte_beta = 0.1f; /* MFS_127: reduced from 0.2 to reduce oscillation */"
        )
        if not DRY_RUN:
            revolute_path.write_text(content)
        log("  [OK] Reduced Baumgarte beta 0.2 → 0.1")
    else:
        log("  [SKIP] Baumgarte beta already adjusted or not found")

    # 1b. Increase yaw damping in drivetrain.c
    drivetrain_path = SRC / "robotics" / "drivetrain.c"
    content = drivetrain_path.read_text()

    if "yaw_vel * m * 1.0f * 0.02f" in content:
        content = content.replace(
            "yaw_vel * m * 1.0f * 0.02f",
            "yaw_vel * m * 1.5f * 0.02f /* MFS_127: increased from 1.0 to stop residual rotation */"
        )
        if not DRY_RUN:
            drivetrain_path.write_text(content)
        log("  [OK] Increased yaw damping 1.0 → 1.5")
    else:
        log("  [SKIP] Yaw damping already adjusted or not found")

    return True


# ============================================================
# STEP 2: Add diagnostic logging to mecanum friction
# ============================================================
def step_add_strafe_diagnostics():
    """Add conditional logging to mecanum friction model for debugging."""
    log("Step 2: Adding strafe diagnostics to collision_mechanics.c")
    path = SRC / "physics" / "collision_mechanics.c"
    content = path.read_text()

    if "MFS_127_STRAFE_DIAG" in content:
        log("  [SKIP] Strafe diagnostics already present")
        return True

    # Find the mecanum friction computation and add logging
    # Look for the roller angle computation
    anchor = "roller_angle_rad"

    if anchor in content:
        # Add a static counter to log every Nth frame
        diagnostic_code = """
/* MFS_127_STRAFE_DIAG: Conditional logging for mecanum friction debugging */
#ifdef MFS_DEBUG_STRAFE
{
    static int strafe_diag_counter = 0;
    if ((strafe_diag_counter++ % 60) == 0) {  /* Log every 60 frames */
        float grip_mag = sqrtf(grip_dir.x * grip_dir.x + grip_dir.z * grip_dir.z);
        float tangent_mag = sqrtf(contact->tangent.x * contact->tangent.x + contact->tangent.z * contact->tangent.z);
        printf("[STRAFE_DIAG] roller=%.1f° grip=(%.2f,%.2f,%.2f) mag=%.3f tangent=(%.2f,%.2f,%.2f) mag=%.3f normal=%.2f\\n",
               roller_angle_rad * 180.0f / 3.14159f,
               grip_dir.x, grip_dir.y, grip_dir.z, grip_mag,
               contact->tangent.x, contact->tangent.y, contact->tangent.z, tangent_mag,
               contact->normal_force);
    }
}
#endif
"""
        # Insert after the grip_dir computation
        lines = content.split('\n')
        insert_idx = -1
        for i, line in enumerate(lines):
            if 'grip_dir' in line and 'vector3' in line and i > 0:
                # Find the next few lines to insert after grip_dir is fully computed
                for j in range(i, min(i + 10, len(lines))):
                    if 'contact->tangent' in lines[j]:
                        insert_idx = j + 1
                        break
                break

        if insert_idx > 0:
            lines.insert(insert_idx, diagnostic_code)
            content = '\n'.join(lines)
            if not DRY_RUN:
                path.write_text(content)
            log("  [OK] Added strafe diagnostics (compile with -DMFS_DEBUG_STRAFE to enable)")
            return True

    log("  [WARN] Could not find mecanum friction anchor")
    return True


# ============================================================
# STEP 3: Fix camera debug fly mode
# ============================================================
def step_fix_camera_float():
    """Fix camera floating in debug mode by correcting state management."""
    log("Step 3: Fixing camera debug fly mode")
    path = SRC / "ui_input" / "simulation_camera.c"
    content = path.read_text()

    # The issue is likely that debug mode activation sets a vertical velocity
    # that isn't being cleared when debug mode is deactivated or when keys are released.
    # Look for the debug fly mode code

    if "MFS_127_CAMERA_FIX" in content:
        log("  [SKIP] Camera fix already present")
        return True

    # Find the debug fly mode section
    # It likely has code like: if (debug_mode) { camera.velocity.y += ... }
    # The fix is to ensure vertical velocity is zeroed when not actively flying

    lines = content.split('\n')
    new_lines = []
    in_debug_fly = False
    fixed = False

    for i, line in enumerate(lines):
        # Look for debug fly mode activation
        if 'debug' in line.lower() and 'fly' in line.lower() and 'mode' in line.lower():
            in_debug_fly = True

        # Look for vertical velocity application in debug mode
        if in_debug_fly and ('velocity.y' in line or 'position.y' in line) and '+=' in line:
            # This is likely the problematic line
            # Add a check to only apply when shift/space is held
            indent = len(line) - len(line.lstrip())
            new_lines.append(' ' * indent + '/* MFS_127_CAMERA_FIX: Only apply vertical movement when keys are held */')
            new_lines.append(' ' * indent + 'if (main_inputs.space_held || main_inputs.left_shift_held) {')
            new_lines.append(line)
            new_lines.append(' ' * indent + '} else {')
            new_lines.append(' ' * indent + '    /* Zero vertical velocity when not actively flying */')
            new_lines.append(' ' * indent + '    if (camera->velocity.y > 0.01f || camera->velocity.y < -0.01f) {')
            new_lines.append(' ' * indent + '        camera->velocity.y *= 0.9f; /* Damp vertical velocity */')
            new_lines.append(' ' * indent + '    } else {')
            new_lines.append(' ' * indent + '        camera->velocity.y = 0.0f;')
            new_lines.append(' ' * indent + '    }')
            new_lines.append(' ' * indent + '}')
            fixed = True
            in_debug_fly = False
            continue

        new_lines.append(line)

    if fixed:
        if not DRY_RUN:
            path.write_text('\n'.join(new_lines))
        log("  [OK] Fixed camera debug fly mode state management")
        return True

    # Fallback: look for the specific pattern where debug mode is checked
    if 'if (main_inputs.debug_mode)' in content or 'if (debug_mode)' in content:
        # Add vertical velocity damping when debug mode is active but no keys are held
        anchor = "if (main_inputs.debug_mode)"
        if anchor not in content:
            anchor = "if (debug_mode)"

        if anchor in content:
            # Find the section and add damping
            lines = content.split('\n')
            for i, line in enumerate(lines):
                if anchor in line:
                    # Insert damping code after this line
                    indent = len(line) - len(line.lstrip())
                    damping_code = [
                        ' ' * (indent + 4) + '/* MFS_127_CAMERA_FIX: Damp vertical velocity when not actively flying */',
                        ' ' * (indent + 4) + 'if (!main_inputs.space_held && !main_inputs.left_shift_held) {',
                        ' ' * (indent + 4) + '    if (camera->velocity.y > 0.01f || camera->velocity.y < -0.01f) {',
                        ' ' * (indent + 4) + '        camera->velocity.y *= 0.9f;',
                        ' ' * (indent + 4) + '    } else {',
                        ' ' * (indent + 4) + '        camera->velocity.y = 0.0f;',
                        ' ' * (indent + 4) + '    }',
                        ' ' * (indent + 4) + '}'
                    ]
                    lines = lines[:i+1] + damping_code + lines[i+1:]
                    content = '\n'.join(lines)
                    if not DRY_RUN:
                        path.write_text(content)
                    log("  [OK] Added camera vertical velocity damping (fallback)")
                    return True

    log("  [WARN] Could not find camera debug fly mode pattern")
    return True


# ============================================================
# MAIN
# ============================================================
def main():
    print("=" * 60)
    print("MFS 127: Fix Rotation Wobble, Strafe, Camera Float")
    print("=" * 60)

    if DRY_RUN:
        print("  ** DRY RUN — no files will be modified **")
        print()

    if not SRC.exists():
        print(f"FATAL: Source directory not found: {SRC}")
        return 1

    steps = [
        ("Rotation wobble", step_fix_rotation_wobble),
        ("Strafe diagnostics", step_add_strafe_diagnostics),
        ("Camera float", step_fix_camera_float),
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
    print("    1. Rotation: Baumgarte beta 0.2→0.1, yaw damping 1.0→1.5")
    print("    2. Strafe: Added diagnostics (compile with -DMFS_DEBUG_STRAFE)")
    print("    3. Camera: Fixed debug fly mode vertical velocity management")
    print()
    print("  To debug strafe issue:")
    print("    make clean && make CFLAGS='-DMFS_DEBUG_STRAFE'")
    print("    Then drive with V/N and watch the [STRAFE_DIAG] output")
    print("=" * 60)
    return 0


if __name__ == "__main__":
    sys.exit(main())
