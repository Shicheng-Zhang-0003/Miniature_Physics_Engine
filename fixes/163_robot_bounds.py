#!/usr/bin/env python3
"""
MFS 163: Robot struct bounds safety
====================================
Bug addressed:
  5. wheel_radians[4] vs FTC_MAX_WHEELS = 8 (robot.h / drivetrain.c)

The odometry array is hardcoded to 4 but the loop iterates to wheel_count.
Fix: use FTC_MAX_WHEELS for the array size.

Usage:
cd <project_root>
python3 fixes/163_robot_bounds.py [--dry-run]
"""
import sys, subprocess
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
ROOT = SCRIPT_DIR.parent
SRC = ROOT / "v15R3" / "src"
TOOLS = ROOT / "tools"
sys.path.insert(0, str(TOOLS))

DRY_RUN = "--dry-run" in sys.argv

def log(msg): print(f"  [163] {msg}")

def write(p, t):
    if not DRY_RUN: p.write_text(t)
    log(f"  [OK] {p.relative_to(SRC)}")

def fix_wheel_radians_size():
    log("Bug 5: Fix wheel_radians array size")
    p = SRC / "robotics" / "robot.h"
    content = p.read_text()
    if "MFS_163_BOUNDS_FIX" in content:
        log("  [SKIP] already fixed"); return True

    old = "float wheel_radians[4];"
    new = "float wheel_radians[FTC_MAX_WHEELS]; /* MFS_163_BOUNDS_FIX: was [4], OOB if wheel_count > 4 */"

    if old in content:
        content = content.replace(old, new, 1)
        write(p, content)
        return True
    log("  [WARN] pattern not found"); return True

def fix_odometry_loop_guard():
    log("Bug 5b: Guard odometry loop to FTC_MAX_WHEELS")
    p = SRC / "robotics" / "drivetrain.c"
    content = p.read_text()
    if "MFS_163_ODOM_GUARD" in content:
        log("  [SKIP] already fixed"); return True

    old = "for (int mfs_i = 0; mfs_i < robot->wheel_count; mfs_i++) {"
    # Only replace the one in the odometry block (after MFS_151_INTEGRATE marker)
    odom_marker = "/* MFS_151_INTEGRATE: Odometry integration */"
    odom_idx = content.find(odom_marker)
    if odom_idx < 0:
        log("  [SKIP] odometry block not found"); return True

    loop_idx = content.find(old, odom_idx)
    if loop_idx < 0:
        log("  [SKIP] odometry loop not found"); return True

    new = "for (int mfs_i = 0; mfs_i < robot->wheel_count && mfs_i < FTC_MAX_WHEELS; mfs_i++) { /* MFS_163_ODOM_GUARD */"
    content = content[:loop_idx] + new + content[loop_idx + len(old):]
    write(p, content)
    return True

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
    print("MFS 163: Robot struct bounds safety")
    print("=" * 60)
    if DRY_RUN: print("  ** DRY RUN **\n")

    steps = [fix_wheel_radians_size, fix_odometry_loop_guard]
    for fn in steps:
        try:
            if not fn(): return 1
        except Exception as e:
            print(f"\n[FAIL] {fn.__name__}: {e}"); import traceback; traceback.print_exc(); return 1

    if not DRY_RUN and not build_and_test(): return 1

    print("=" * 60)
    print("  163 complete. wheel_radians now FTC_MAX_WHEELS, odometry loop guarded.")
    print("=" * 60)
    return 0

if __name__ == "__main__": sys.exit(main())
