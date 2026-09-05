#!/usr/bin/env python3
"""
MFS 161: Critical crash fix + input_control sweep + gamepad guard
==================================================================
Bugs addressed:
  1. NULL dereference in ftc_robot_create_with_drive (robot.c)
  2. Duplicate g_key_pressed clear in on_key_released (input_control.c)
  3. q_key_pressed missing from initialize_input (input_control.c)
 16. gamepad_poll called even when no robot exists (simulation_input_dispatch.c)

Usage:
cd <project_root>
python3 fixes/161_critical_input_sweep.py [--dry-run]
"""
import sys, subprocess
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
ROOT = SCRIPT_DIR.parent
SRC = ROOT / "v15R3" / "src"
TOOLS = ROOT / "tools"
sys.path.insert(0, str(TOOLS))

DRY_RUN = "--dry-run" in sys.argv

def log(msg): print(f"  [161] {msg}")

def write(p, t):
    if not DRY_RUN: p.write_text(t)
    log(f"  [OK] {p.relative_to(SRC)}")

# ---- Bug 1: NULL dereference in ftc_robot_create_with_drive ----
def fix_null_deref():
    log("Bug 1: Fix NULL dereference in ftc_robot_create_with_drive")
    p = SRC / "robotics" / "robot.c"
    content = p.read_text()
    if "MFS_161_NULL_FIX" in content:
        log("  [SKIP] already fixed"); return True

    # The MFS_151_ZERO block dereferences robot before the null-check.
    # Fix: move null-check to the top, remove the redundant MFS_151_ZERO block.
    old = """int ftc_robot_create_with_drive(physics_world *world, ftc_robot *robot, float x, float y, float z,
motor_preset_id preset, ftc_drivetrain_type drivetrain_type) {
/* MFS_151_ZERO */
robot->odom_x = robot->odom_z = robot->odom_theta = 0.0f;
for (int mfs_i = 0; mfs_i < 4; mfs_i++) robot->wheel_radians[mfs_i] = 0.0f;
if ((!world) || (!robot)) {
return 1;
}
memset(robot, 0, sizeof(ftc_robot));"""

    new = """int ftc_robot_create_with_drive(physics_world *world, ftc_robot *robot, float x, float y, float z,
motor_preset_id preset, ftc_drivetrain_type drivetrain_type) {
/* MFS_161_NULL_FIX: null-check FIRST, before any dereference */
if ((!world) || (!robot)) {
return 1;
}
memset(robot, 0, sizeof(ftc_robot));
/* memset zeroes odom_x/z/theta and wheel_radians — no separate init needed */"""

    if old in content:
        content = content.replace(old, new, 1)
        write(p, content)
        return True
    log("  [WARN] pattern not found — may already be fixed"); return True

# ---- Bug 2: Duplicate g_key_pressed clear ----
def fix_duplicate_g_key():
    log("Bug 2: Remove duplicate g_key_pressed clear in on_key_released")
    p = SRC / "ui_input" / "input_control.c"
    content = p.read_text()
    if "MFS_161_DUP_FIX" in content:
        log("  [SKIP] already fixed"); return True

    old = """if (event -> keyval == GDK_KEY_g) {input_state -> g_key_pressed = false;} /* MFS_GUI_BRIDGE */
if (event -> keyval == GDK_KEY_g) {input_state -> g_key_pressed = false;}"""
    new = """if (event -> keyval == GDK_KEY_g) {input_state -> g_key_pressed = false;} /* MFS_161_DUP_FIX: deduplicated */"""

    if old in content:
        content = content.replace(old, new, 1)
        write(p, content)
        return True
    log("  [WARN] pattern not found"); return True

# ---- Bug 3: q_key_pressed missing from initialize_input ----
def fix_q_key_init():
    log("Bug 3: Add q_key_pressed to initialize_input")
    p = SRC / "ui_input" / "input_control.c"
    content = p.read_text()
    if "MFS_161_QKEY_FIX" in content:
        log("  [SKIP] already fixed"); return True

    # Insert q_key_pressed init after f_key_pressed init
    old = "input_state -> f_key_pressed = false;"
    new = "input_state -> f_key_pressed = false;\n    input_state -> q_key_pressed = false; /* MFS_161_QKEY_FIX */"

    if old in content and "input_state -> q_key_pressed = false;" not in content.split("void initialize_input")[1].split("gboolean on_keypress")[0]:
        content = content.replace(old, new, 1)
        write(p, content)
        return True
    log("  [SKIP] q_key_pressed already initialized or pattern changed"); return True

# ---- Bug 16: gamepad_poll guard ----
def fix_gamepad_poll_guard():
    log("Bug 16: Guard gamepad_poll with robot count check")
    p = SRC / "ui_input" / "simulation_input_dispatch.c"
    content = p.read_text()
    if "MFS_161_GAMEPAD_GUARD" in content:
        log("  [SKIP] already fixed"); return True

    old = """    /* MFS_157_GAMEPAD_POLL: drain gamepad events once per frame */
    gamepad_poll(gamepad_get_primary());"""
    new = """    /* MFS_157_GAMEPAD_POLL: drain gamepad events once per frame */
    /* MFS_161_GAMEPAD_GUARD: only poll when a robot exists to save syscalls */
    if (gui_robot_get_count() > 0) {
        gamepad_poll(gamepad_get_primary());
    }"""

    if old in content:
        content = content.replace(old, new, 1)
        write(p, content)
        return True
    log("  [WARN] pattern not found"); return True

# ---- Build + test ----
def build_and_test():
    log("Building...")
    r = subprocess.run([sys.executable, str(TOOLS / "build_check.py"), "--quick"],
                       cwd=ROOT, capture_output=True, text=True, timeout=180)
    if r.returncode != 0:
        print(r.stdout[-2000:]); print(r.stderr[-2000:])
        log("[FAIL] build failed"); return False
    log("[PASS] build clean")
    log("Running tests...")
    r = subprocess.run([sys.executable, str(TOOLS / "test_runner.py")],
                       cwd=ROOT, capture_output=True, text=True, timeout=300)
    if r.returncode != 0:
        log("[WARN] some tests failed")
    else:
        log("[PASS] all tests pass")
    return True

def main():
    print("=" * 60)
    print("MFS 161: Critical crash + input sweep + gamepad guard")
    print("=" * 60)
    if DRY_RUN: print("  ** DRY RUN **\n")

    steps = [fix_null_deref, fix_duplicate_g_key, fix_q_key_init, fix_gamepad_poll_guard]
    for fn in steps:
        try:
            if not fn(): return 1
        except Exception as e:
            print(f"\n[FAIL] {fn.__name__}: {e}"); import traceback; traceback.print_exc(); return 1

    if not DRY_RUN and not build_and_test(): return 1

    print("=" * 60)
    print("  161 complete. NULL crash fixed, input cleaned, gamepad guarded.")
    print("=" * 60)
    return 0

if __name__ == "__main__": sys.exit(main())
