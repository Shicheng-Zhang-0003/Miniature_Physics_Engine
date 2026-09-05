#!/usr/bin/env python3
"""
MFS 158: Fix gamepad Y-axis double inversion
=============================================
The invert_left_y flag in gamepad_get_axis already negates the Y axis
(evdev convention: up = negative). The drive block was negating again,
making stick-up = backward. Remove the redundant negation.

Before: kb_forward += -gamepad_get_axis(mfs_pad, gamepad_axis_left_y);
After:  kb_forward += gamepad_get_axis(mfs_pad, gamepad_axis_left_y);
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

def log(msg): print(f"  [158] {msg}")

def step_fix():
    log("Step 1: Remove redundant Y-axis negation in drive block")
    path = SRC / "ui_input" / "simulation_input_dispatch.c"
    if not path.exists():
        log("[FAIL] simulation_input_dispatch.c not found")
        return False
    content = path.read_text()
    if "MFS_158" in content:
        log("[SKIP] already fixed")
        return True
    old = "kb_forward += -gamepad_get_axis(mfs_pad, gamepad_axis_left_y); /* up = forward */"
    new = "kb_forward += gamepad_get_axis(mfs_pad, gamepad_axis_left_y); /* MFS_158: up = forward (inverted via config) */"
    if old not in content:
        log("[FAIL] anchor not found — drive block may have changed")
        return False
    content = content.replace(old, new, 1)
    if not DRY_RUN:
        path.write_text(content)
    log("[OK] negation removed")
    return True

def step_build():
    log("Step 2: Build verification")
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
    log("Step 3: Run headless test suite")
    r = subprocess.run(
        [sys.executable, str(TOOLS / "test_runner.py")],
        cwd=str(ROOT), capture_output=True, text=True, timeout=300
    )
    print(r.stdout[-2000:] if r.stdout else "")
    if r.returncode != 0:
        log("[WARN] some tests failed")
        return False
    log("[PASS] all tests pass")
    return True

def main():
    print("=" * 60)
    print("MFS 158: Fix Gamepad Y-Axis Double Inversion")
    print("=" * 60)
    if DRY_RUN: print("  ** DRY RUN **\n")
    if not step_fix(): return 1
    if not DRY_RUN:
        if not step_build(): return 1
        if not step_tests(): return 1
    print("=" * 60)
    print("  158 complete. stick up = forward, stick down = backward.")
    print("=" * 60)
    return 0

if __name__ == "__main__":
    sys.exit(main())
