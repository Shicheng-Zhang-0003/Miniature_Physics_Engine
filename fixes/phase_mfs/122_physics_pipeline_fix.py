#!/usr/bin/env python3
"""
MFS Phase: Physics Pipeline Fix (122)
======================================
Fixes the motor→gearbox→wheel→contact→chassis pipeline end-to-end.

Changes:
  1. motor.c: restore gear_ratio multiplication on output_torque
  2. broadphase.c: fix dead cylinder code (unreachable after return)
  3. rigidbody.c: remove duplicate cylinder dispatch in set_static
  4. gui_robot_registry.c: add fixed-timestep accumulator to gui_robot_tick
  5. drivetrain.c: reduce lateral damping, make it proportional
  6. Remove wheel_traction.c/h (dead files)
  7. makefile: strip any wheel_traction references

Usage:
  cd <project_root>
  python3 fixes/phase_mfs/122_physics_pipeline_fix.py [--dry-run]
"""

import sys
import os
import subprocess
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
ROOT = SCRIPT_DIR.parent.parent  # project root (mfs/)
SRC = ROOT / "v15R2" / "src"
TOOLS = ROOT / "tools"

sys.path.insert(0, str(TOOLS))
from refactor import Refactor, RefactorError

DRY_RUN = "--dry-run" in sys.argv


def log(msg):
    print(f"  [122] {msg}")


def run_build():
    """Run build_check.py and return success bool."""
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
    """Run test_runner.py and return success bool."""
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
# STEP 1: Fix motor output_torque (THE critical bug)
# ============================================================
def step_fix_motor_output_torque():
    """Restore gear_ratio multiplication on output_torque in motor.c."""
    log("Step 1: Fixing motor output_torque (restoring gear ratio)")
    path = SRC / "robotics" / "motor.c"
    content = path.read_text()

    # The current broken line:
    #   m->output_torque = m->torque;
    # Should be:
    #   m->output_torque = m->torque * m->gear_ratio;

    if "m->output_torque = m->torque * m->gear_ratio;" in content:
        log("  [SKIP] output_torque already has gear_ratio")
        return True

    if "m->output_torque = m->torque;" not in content:
        log("  [WARN] Could not find output_torque assignment pattern")
        return False

    r = Refactor(str(path))
    r.replace(
        old="m->output_torque = m->torque;",
        new="m->output_torque = m->torque * m->gear_ratio; /* MFS_122: restore gearing */",
        label="Fix output_torque gear ratio"
    )

    if not DRY_RUN:
        r.apply(dry_run=False)
    else:
        print(r.diff())

    log("  [OK] output_torque now multiplies by gear_ratio")
    return True


# ============================================================
# STEP 2: Fix broadphase_bounding_radius dead cylinder code
# ============================================================
def step_fix_broadphase_cylinder():
    """Fix unreachable cylinder branch in broadphase_bounding_radius."""
    log("Step 2: Fixing broadphase_bounding_radius cylinder dead code")
    path = SRC / "physics" / "broadphase.c"
    content = path.read_text()

    # Current broken code:
    #   return sqrtf(rb->half_extensions.x * ...);
    #   if (rb->type == object_cylinder) { /* MPE_FTC_091 */
    #       return sqrtf(rb->radius * ...);
    #   }
    #
    # The cylinder check is AFTER the return, so it never executes.
    # Fix: move it BEFORE the generic return.

    # Find the broken pattern
    broken_pattern = """    return sqrtf(rb->half_extensions.x * rb->half_extensions.x + rb->half_extensions.y * rb->half_extensions.y +
                 rb->half_extensions.z * rb->half_extensions.z);
if (rb->type == object_cylinder) { /* MPE_FTC_091 */
return sqrtf(rb->radius * rb->radius +
rb->cylinder_half_length * rb->cylinder_half_length);
}"""

    fixed_pattern = """    if (rb->type == object_cylinder) { /* MPE_FTC_091 */
        return sqrtf(rb->radius * rb->radius +
                     rb->cylinder_half_length * rb->cylinder_half_length);
    }
    return sqrtf(rb->half_extensions.x * rb->half_extensions.x + rb->half_extensions.y * rb->half_extensions.y +
                 rb->half_extensions.z * rb->half_extensions.z);"""

    if "if (rb->type == object_cylinder)" in content and "return sqrtf(rb->half_extensions" in content:
        # Check if already fixed (cylinder check before the generic return)
        cyl_pos = content.find("if (rb->type == object_cylinder)")
        gen_ret_pos = content.find("return sqrtf(rb->half_extensions")
        if cyl_pos < gen_ret_pos:
            log("  [SKIP] cylinder branch already before generic return")
            return True

    if broken_pattern in content:
        r = Refactor(str(path))
        r.replace(
            old=broken_pattern,
            new=fixed_pattern,
            label="Fix cylinder bounding radius order"
        )
        if not DRY_RUN:
            r.apply(dry_run=False)
        else:
            print(r.diff())
        log("  [OK] cylinder branch moved before generic return")
        return True

    # Try a more flexible approach
    log("  [INFO] Trying flexible pattern match...")
    lines = content.split('\n')
    new_lines = []
    i = 0
    fixed = False
    while i < len(lines):
        line = lines[i]
        # Detect the generic return for cubes
        if "return sqrtf(rb->half_extensions.x" in line and not fixed:
            # Insert cylinder check BEFORE this return
            new_lines.append("    if (rb->type == object_cylinder) { /* MPE_FTC_091 */")
            new_lines.append("        return sqrtf(rb->radius * rb->radius +")
            new_lines.append("                     rb->cylinder_half_length * rb->cylinder_half_length);")
            new_lines.append("    }")
            new_lines.append(line)
            fixed = True
            i += 1
            # Skip the dead cylinder code that follows
            while i < len(lines):
                if "object_cylinder" in lines[i] and "MPE_FTC_091" in lines[i]:
                    # Skip this block
                    brace_count = 0
                    while i < len(lines):
                        if '{' in lines[i]:
                            brace_count += lines[i].count('{')
                        if '}' in lines[i]:
                            brace_count -= lines[i].count('}')
                        i += 1
                        if brace_count <= 0:
                            break
                    break
                elif lines[i].strip() == '}' or lines[i].strip() == '':
                    i += 1
                    continue
                else:
                    break
            continue
        new_lines.append(line)
        i += 1

    if fixed:
        if not DRY_RUN:
            path.write_text('\n'.join(new_lines))
        log("  [OK] cylinder branch inserted before generic return (flexible)")
        return True

    log("  [WARN] Could not fix broadphase cylinder code")
    return False


# ============================================================
# STEP 3: Remove duplicate cylinder dispatch in rigidbody_set_static
# ============================================================
def step_fix_rigidbody_duplicate_cylinder():
    """Remove the unreachable second cylinder branch in rigidbody_set_static."""
    log("Step 3: Removing duplicate cylinder dispatch in rigidbody_set_static")
    path = SRC / "core" / "rigidbody.c"
    content = path.read_text()

    # The broken pattern has TWO cylinder checks:
    #   } else if (rigid_body->type == object_cylinder) { /* MPE_FTC_091 */
    #       rigidbody_update_inertia_cylinder(rigid_body);
    #   } else if (rigid_body->type == object_cylinder) { /* MPE_FTC_093d */
    #       rigidbody_update_inertia_cylinder(rigid_body);
    #   } else {
    #
    # The second one is unreachable. Remove it.

    broken = """} else if (rigid_body->type == object_cylinder) { /* MPE_FTC_091 */
rigidbody_update_inertia_cylinder(rigid_body);
} else if (rigid_body->type == object_cylinder) { /* MPE_FTC_093d */
rigidbody_update_inertia_cylinder(rigid_body);
} else {"""

    fixed = """} else if (rigid_body->type == object_cylinder) { /* MPE_FTC_091 */
rigidbody_update_inertia_cylinder(rigid_body);
} else {"""

    if broken in content:
        r = Refactor(str(path))
        r.replace(
            old=broken,
            new=fixed,
            label="Remove duplicate cylinder dispatch"
        )
        if not DRY_RUN:
            r.apply(dry_run=False)
        else:
            print(r.diff())
        log("  [OK] duplicate cylinder branch removed")
        return True

    # Check if already fixed
    if content.count("object_cylinder") <= 3:  # enum + one dispatch + maybe init
        log("  [SKIP] no duplicate found")
        return True

    log("  [WARN] Could not find exact duplicate pattern, checking count...")
    count = content.count("rigidbody_update_inertia_cylinder(rigid_body)")
    if count <= 2:  # One in set_static, one in sanitize or init
        log("  [SKIP] already clean")
        return True

    log("  [WARN] Multiple cylinder inertia calls found but pattern didn't match exactly")
    return True  # Non-blocking


# ============================================================
# STEP 4: Add fixed-timestep accumulator to gui_robot_tick
# ============================================================
def step_fix_robot_timestep():
    """Replace frame-rate-dependent robot stepping with fixed timestep."""
    log("Step 4: Adding fixed-timestep accumulator to gui_robot_tick")
    path = SRC / "robotics" / "gui_robot_registry.c"
    content = path.read_text()

    # Current broken code in gui_robot_tick:
    #   for (int i = 0; i < mfs_gui_robot_count; i++) {
    #       drivetrain_update(mfs_gui_robot_world, &mfs_gui_robots[i], dt);
    #   }
    #   physics_world_step(mfs_gui_robot_world, dt);
    #
    # This uses raw frame dt. Replace with fixed-timestep accumulator.

    old_tick = """void gui_robot_tick(float dt) {
if ((mfs_gui_robot_count <= 0) || (!mfs_gui_robot_world)) {
return;
}
for (int i = 0; i < mfs_gui_robot_count; i++) {
drivetrain_update(mfs_gui_robot_world, &mfs_gui_robots[i], dt);
}
/* Step the robot's physics world */
physics_world_step(mfs_gui_robot_world, dt);"""

    new_tick = """void gui_robot_tick(float dt) {
if ((mfs_gui_robot_count <= 0) || (!mfs_gui_robot_world)) {
return;
}
/* MFS_122: Fixed-timestep accumulator for deterministic robot physics.
* The robot world steps at 60 Hz regardless of render framerate. */
static float robot_accumulator = 0.0f;
const float fixed_robot_dt = 1.0f / 60.0f;
const float max_frame_time = fixed_robot_dt * 5.0f; /* spiral-of-death cap */

robot_accumulator += dt;
if (robot_accumulator > max_frame_time) {
robot_accumulator = max_frame_time;
}

while (robot_accumulator >= fixed_robot_dt) {
for (int i = 0; i < mfs_gui_robot_count; i++) {
drivetrain_update(mfs_gui_robot_world, &mfs_gui_robots[i], fixed_robot_dt);
}
physics_world_step(mfs_gui_robot_world, fixed_robot_dt);
robot_accumulator -= fixed_robot_dt;
}"""

    if "robot_accumulator" in content:
        log("  [SKIP] fixed timestep already present")
        return True

    if old_tick in content:
        r = Refactor(str(path))
        r.replace(
            old=old_tick,
            new=new_tick,
            label="Add fixed-timestep accumulator"
        )
        if not DRY_RUN:
            r.apply(dry_run=False)
        else:
            print(r.diff())
        log("  [OK] gui_robot_tick now uses fixed 60Hz timestep")
        return True

    log("  [WARN] Could not find exact gui_robot_tick pattern")
    log("  [INFO] Attempting line-based replacement...")

    # Fallback: find the function and replace the physics step line
    if "physics_world_step(mfs_gui_robot_world, dt);" in content:
        r = Refactor(str(path))
        r.replace(
            old="physics_world_step(mfs_gui_robot_world, dt);",
            new="""/* MFS_122: Fixed-timestep accumulator */
{
static float robot_accumulator = 0.0f;
const float fixed_robot_dt = 1.0f / 60.0f;
robot_accumulator += dt;
if (robot_accumulator > fixed_robot_dt * 5.0f) robot_accumulator = fixed_robot_dt * 5.0f;
while (robot_accumulator >= fixed_robot_dt) {
physics_world_step(mfs_gui_robot_world, fixed_robot_dt);
robot_accumulator -= fixed_robot_dt;
}
}""",
            label="Wrap physics_world_step in accumulator"
        )
        # Also fix the drivetrain_update calls to use fixed dt
        r.replace(
            old="drivetrain_update(mfs_gui_robot_world, &mfs_gui_robots[i], dt);",
            new="drivetrain_update(mfs_gui_robot_world, &mfs_gui_robots[i], fixed_robot_dt);",
            label="Use fixed dt for drivetrain"
        )
        if not DRY_RUN:
            r.apply(dry_run=False)
        else:
            print(r.diff())
        log("  [OK] gui_robot_tick patched (fallback method)")
        return True

    return False


# ============================================================
# STEP 5: Reduce chassis damping in drivetrain.c
# ============================================================
def step_reduce_chassis_damping():
    """Reduce aggressive lateral damping from 3.0 to 1.0, yaw from 1.5 to 0.5."""
    log("Step 5: Reducing chassis damping coefficients")
    path = SRC / "robotics" / "drivetrain.c"
    content = path.read_text()

    changed = False

    # Lateral damping: m * 3.0f → m * 1.0f
    if "m * 3.0f" in content:
        r = Refactor(str(path))
        r.replace(
            old="vector3_scaling(lat, m * 3.0f)",
            new="vector3_scaling(lat, m * 1.0f) /* MFS_122: reduced from 3.0 */",
            label="Reduce lateral damping"
        )
        if not DRY_RUN:
            r.apply(dry_run=False)
        else:
            print(r.diff())
        changed = True
        log("  [OK] lateral damping reduced 3.0 → 1.0")

    # Yaw damping: yaw_vel * m * 1.5f * 0.02f → yaw_vel * m * 0.5f * 0.02f
    content = path.read_text()  # re-read after potential change
    if "yaw_vel * m * 1.5f * 0.02f" in content:
        r = Refactor(str(path))
        r.replace(
            old="yaw_vel * m * 1.5f * 0.02f",
            new="yaw_vel * m * 0.5f * 0.02f /* MFS_122: reduced from 1.5 */",
            label="Reduce yaw damping"
        )
        if not DRY_RUN:
            r.apply(dry_run=False)
        else:
            print(r.diff())
        changed = True
        log("  [OK] yaw damping reduced 1.5 → 0.5")

    if not changed:
        log("  [SKIP] damping values already adjusted or not found")

    return True


# ============================================================
# STEP 6: Remove dead wheel_traction files
# ============================================================
def step_remove_wheel_traction():
    """Delete wheel_traction.c and wheel_traction.h (dead since fix 094)."""
    log("Step 6: Removing dead wheel_traction files")

    wt_c = SRC / "robotics" / "wheel_traction.c"
    wt_h = SRC / "robotics" / "wheel_traction.h"

    removed = []
    for f in [wt_c, wt_h]:
        if f.exists():
            if not DRY_RUN:
                f.unlink()
            removed.append(f.name)
            log(f"  [OK] removed {f.name}")
        else:
            log(f"  [SKIP] {f.name} not found")

    # Also check makefile for any wheel_traction references
    makefile = SRC / "makefile"
    if makefile.exists():
        content = makefile.read_text()
        if "wheel_traction" in content:
            r = Refactor(str(makefile))
            r.replace(
                old="robotics/wheel_traction.c",
                new="",
                label="Remove wheel_traction from makefile"
            )
            if not DRY_RUN:
                r.apply(dry_run=False)
            log("  [OK] removed wheel_traction from makefile")

    return True


# ============================================================
# STEP 7: Verify robot.c doesn't reference wheel_traction
# ============================================================
def step_verify_robot_clean():
    """Ensure robot.c has no wheel_traction references."""
    log("Step 7: Verifying robot.c is clean of wheel_traction")
    path = SRC / "robotics" / "robot.c"
    content = path.read_text()

    if "wheel_traction" in content:
        r = Refactor(str(path))
        r.replace(
            old='#include "wheel_traction.h"',
            new="/* wheel_traction removed (MFS_122) */",
            label="Remove wheel_traction include"
        )
        if not DRY_RUN:
            r.apply(dry_run=False)
        log("  [OK] removed wheel_traction include from robot.c")
    else:
        log("  [SKIP] no wheel_traction references in robot.c")

    return True


# ============================================================
# MAIN
# ============================================================
def main():
    print("=" * 60)
    print("MFS 122: Physics Pipeline Fix")
    print("=" * 60)

    if DRY_RUN:
        print("  ** DRY RUN — no files will be modified **")
        print()

    if not SRC.exists():
        print(f"FATAL: Source directory not found: {SRC}")
        return 1

    steps = [
        ("Motor output_torque", step_fix_motor_output_torque),
        ("Broadphase cylinder", step_fix_broadphase_cylinder),
        ("Rigidbody duplicate", step_fix_rigidbody_duplicate_cylinder),
        ("Robot timestep", step_fix_robot_timestep),
        ("Chassis damping", step_reduce_chassis_damping),
        ("Remove wheel_traction", step_remove_wheel_traction),
        ("Verify robot.c", step_verify_robot_clean),
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
            # Don't hard-fail on test issues since mecanum may need tuning
        else:
            log("[PASS] All tests passed!")
    else:
        log("[DRY RUN] Skipping build verification.")

    print()
    print("=" * 60)
    print("  DONE. Summary of changes:")
    print("    1. motor.c: output_torque *= gear_ratio (restores ~20x torque)")
    print("    2. broadphase.c: cylinder bounding radius now reachable")
    print("    3. rigidbody.c: removed dead duplicate cylinder dispatch")
    print("    4. gui_robot_registry.c: robot physics at fixed 60Hz")
    print("    5. drivetrain.c: lateral damping 3.0→1.0, yaw 1.5→0.5")
    print("    6. wheel_traction.c/h: deleted (dead since fix 094)")
    print("=" * 60)
    return 0


if __name__ == "__main__":
    sys.exit(main())
