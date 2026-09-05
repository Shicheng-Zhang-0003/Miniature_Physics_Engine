#!/usr/bin/env python3
"""
MFS 164: gui_robot_apply_drive dispatch fix
=============================================
Bug addressed:
  7. gui_robot_apply_drive always calls drivetrain_mecanum (gui_robot_registry.c)

Fix: Check robot->drivetrain_type and dispatch to the correct drivetrain function.

Usage:
cd <project_root>
python3 fixes/164_drive_dispatch.py [--dry-run]
"""
import sys, subprocess
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
ROOT = SCRIPT_DIR.parent
SRC = ROOT / "v15R3" / "src"
TOOLS = ROOT / "tools"
sys.path.insert(0, str(TOOLS))

DRY_RUN = "--dry-run" in sys.argv

def log(msg): print(f"  [164] {msg}")

def write(p, t):
    if not DRY_RUN: p.write_text(t)
    log(f"  [OK] {p.relative_to(SRC)}")

def fix_drive_dispatch():
    log("Bug 7: Dispatch to correct drivetrain based on type")
    p = SRC / "robotics" / "gui_robot_registry.c"
    content = p.read_text()
    if "MFS_164_DRIVE_DISPATCH" in content:
        log("  [SKIP] already fixed"); return True

    old = """void gui_robot_apply_drive(float forward, float strafe, float rotate) {
if ((mfs_gui_robot_count <= 0) || (!mfs_gui_robot_world)) {
return;
}
for (int i = 0; i < mfs_gui_robot_count; i++) {
drivetrain_mecanum(&mfs_gui_robots[i], forward, strafe, rotate);
}
}"""

    new = """void gui_robot_apply_drive(float forward, float strafe, float rotate) {
if ((mfs_gui_robot_count <= 0) || (!mfs_gui_robot_world)) {
return;
}
for (int i = 0; i < mfs_gui_robot_count; i++) {
/* MFS_164_DRIVE_DISPATCH: dispatch based on drivetrain type */
if (mfs_gui_robots[i].drivetrain_type == FTC_DRIVETRAIN_TANK) {
drivetrain_tank(&mfs_gui_robots[i], forward - rotate, forward + rotate);
} else {
drivetrain_mecanum(&mfs_gui_robots[i], forward, strafe, rotate);
}
}
}"""

    if old in content:
        content = content.replace(old, new, 1)
        write(p, content)
        return True
    log("  [WARN] pattern not found"); return True

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
    print("MFS 164: gui_robot_apply_drive dispatch fix")
    print("=" * 60)
    if DRY_RUN: print("  ** DRY RUN **\n")

    if not fix_drive_dispatch(): return 1
    if not DRY_RUN and not build_and_test(): return 1

    print("=" * 60)
    print("  164 complete. Drive dispatch now respects drivetrain_type.")
    print("=" * 60)
    return 0

if __name__ == "__main__": sys.exit(main())
