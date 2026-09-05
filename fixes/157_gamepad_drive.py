#!/usr/bin/env python3
"""
MFS 157: Gamepad poll + analog drive combining
================================================
In simulation_input_dispatch.c's robot drive block:
1. Poll the gamepad at the top of the function (once per frame)
2. Read left stick Y (inverted), left stick X, right stick X
3. Add analog values to keyboard digital values
4. Clamp combined result to [-1, 1]
Keyboard still works; gamepad overlays on top. Both can be used together.
"""
import sys, subprocess
from pathlib import Path
SCRIPT_DIR = Path(__file__).resolve().parent
ROOT = SCRIPT_DIR.parent
SRC = ROOT / "v15R3" / "src"
TOOLS = ROOT / "tools"
sys.path.insert(0, str(TOOLS))
from refactor import Refactor, RefactorError
DRY_RUN = "--dry-run" in sys.argv

def log(msg): print(f"  [157] {msg}")

def step_poll():
    log("Step 1: Add gamepad_poll at top of simulation_input_dispatch")
    path = SRC / "ui_input" / "simulation_input_dispatch.c"
    content = path.read_text()
    if "MFS_157_GAMEPAD_POLL" in content:
        log("[SKIP] already present")
        return True
    r = Refactor(str(path))
    r.insert_after(
        anchor="void simulation_input_dispatch(GtkWidget *parent_window) {",
        text=(
            "    /* MFS_157_GAMEPAD_POLL: drain gamepad events once per frame */\n"
            "    gamepad_poll(gamepad_get_primary());"
        ),
        label="Add gamepad_poll call"
    )
    if not DRY_RUN:
        r.apply(dry_run=False)
    else:
        print(r.diff())
    log("[OK] poll wired")
    return True

def step_combine():
    log("Step 2: Combine gamepad analog with keyboard in drive block")
    path = SRC / "ui_input" / "simulation_input_dispatch.c"
    content = path.read_text()
    if "MFS_157_GAMEPAD_DRIVE" in content:
        log("[SKIP] already present")
        return True
    # Insert just before gui_robot_apply_drive
    r = Refactor(str(path))
    r.insert_before(
        anchor="gui_robot_apply_drive(kb_forward, kb_strafe, kb_rotate);",
        text=(
            "        /* MFS_157_GAMEPAD_DRIVE: analog stick overlay */\n"
            "        gamepad_state *mfs_pad = gamepad_get_primary();\n"
            "        if (gamepad_is_connected(mfs_pad)) {\n"
            "            kb_forward += -gamepad_get_axis(mfs_pad, gamepad_axis_left_y);  /* up = forward */\n"
            "            kb_strafe  += gamepad_get_axis(mfs_pad, gamepad_axis_left_x);\n"
            "            kb_rotate  += gamepad_get_axis(mfs_pad, gamepad_axis_right_x);\n"
            "            /* clamp combined input to [-1, 1] */\n"
            "            if (kb_forward >  1.0f) kb_forward =  1.0f;\n"
            "            if (kb_forward < -1.0f) kb_forward = -1.0f;\n"
            "            if (kb_strafe  >  1.0f) kb_strafe  =  1.0f;\n"
            "            if (kb_strafe  < -1.0f) kb_strafe  = -1.0f;\n"
            "            if (kb_rotate  >  1.0f) kb_rotate  =  1.0f;\n"
            "            if (kb_rotate  < -1.0f) kb_rotate  = -1.0f;\n"
            "        }"
        ),
        label="Add gamepad analog overlay"
    )
    if not DRY_RUN:
        r.apply(dry_run=False)
    else:
        print(r.diff())
    log("[OK] drive combining wired")
    return True

def step_build():
    log("Step 3: Build verification")
    r = subprocess.run(
        [sys.executable, str(TOOLS / "build_check.py"), "--quick"],
        cwd=str(ROOT), capture_output=True, text=True, timeout=180
    )
    print(r.stdout[-1500:] if r.stdout else "")
    if r.returncode != 0:
        print(r.stderr[-1500:] if r.stderr else "")
        log("[FAIL] build failed")
        return False
    log("[PASS] build clean")
    return True

def step_tests():
    log("Step 4: Run headless test suite")
    r = subprocess.run(
        [sys.executable, str(TOOLS / "test_runner.py")],
        cwd=str(ROOT), capture_output=True, text=True, timeout=300
    )
    print(r.stdout[-2000:] if r.stdout else "")
    if r.returncode != 0:
        log("[WARN] some tests failed (review above)")
        return False
    log("[PASS] all tests pass")
    return True

def main():
    print("=" * 60)
    print("MFS 157: Gamepad Poll + Analog Drive Combining")
    print("=" * 60)
    if DRY_RUN: print("  ** DRY RUN **\n")
    if not step_poll(): return 1
    if not step_combine(): return 1
    if not DRY_RUN:
        if not step_build(): return 1
        if not step_tests(): return 1
    print("=" * 60)
    print("  157 complete. gamepad analog overlays keyboard drive.")
    print("  left stick Y = forward/back, X = strafe")
    print("  right stick X = rotate")
    print("=" * 60)
    return 0

if __name__ == "__main__":
    sys.exit(main())
