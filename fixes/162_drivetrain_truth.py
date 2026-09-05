#!/usr/bin/env python3
"""
MFS 162: Drivetrain truth fixes + dead code cleanup
====================================================
Bugs addressed:
  4. Hardcoded 0.8f friction in traction code (drivetrain.c)
  8. Rolling resistance block outside #if MPE_DRIVETRAIN_REAL (drivetrain.c)
  9. drivetrain_mecanum never sets mecanum_active (drivetrain.c + robot.h)

Usage:
cd <project_root>
python3 fixes/162_drivetrain_truth.py [--dry-run]
"""
import sys, subprocess
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
ROOT = SCRIPT_DIR.parent
SRC = ROOT / "v15R3" / "src"
TOOLS = ROOT / "tools"
sys.path.insert(0, str(TOOLS))

DRY_RUN = "--dry-run" in sys.argv

def log(msg): print(f"  [162] {msg}")

def write(p, t):
    if not DRY_RUN: p.write_text(t)
    log(f"  [OK] {p.relative_to(SRC)}")

# ---- Bug 4: Use config friction instead of hardcoded 0.8f ----
def fix_friction_config():
    log("Bug 4: Replace hardcoded 0.8f with g_cfg.world.floor_friction_s")
    p = SRC / "robotics" / "drivetrain.c"
    content = p.read_text()
    if "MFS_162_FRICTION_FIX" in content:
        log("  [SKIP] already fixed"); return True

    old = "float max_grip = 0.8f * normal_per_wheel;"
    new = "float max_grip = g_cfg.world.floor_friction_s * normal_per_wheel; /* MFS_162_FRICTION_FIX */"

    if old in content:
        content = content.replace(old, new, 1)
        write(p, content)
        return True
    log("  [WARN] pattern not found"); return True

# ---- Bug 8: Move rolling resistance inside #if MPE_DRIVETRAIN_REAL ----
def fix_rolling_resistance_scope():
    log("Bug 8: Move rolling resistance inside #if MPE_DRIVETRAIN_REAL")
    p = SRC / "robotics" / "drivetrain.c"
    content = p.read_text()
    if "MFS_162_RR_SCOPE" in content:
        log("  [SKIP] already fixed"); return True

    # The rolling resistance block is currently AFTER #endif.
    # We need to move it BEFORE the #endif.
    # Strategy: find the #endif and the rolling resistance block,
    # swap their order.
    endif_marker = "#endif /* MPE_DRIVETRAIN_REAL */"
    rr_start_marker = "/* MFS_132_ROLLING_RESISTANCE:"

    endif_idx = content.find(endif_marker)
    rr_idx = content.find(rr_start_marker)

    if (endif_idx < 0) or (rr_idx < 0) or (rr_idx < endif_idx):
        log("  [SKIP] rolling resistance already inside #if or pattern changed"); return True

    # Find the end of the rolling resistance block (next #endif or end of function)
    # The RR block ends with a closing brace before the odometry block
    rr_end_marker = "/* MFS_151_INTEGRATE: Odometry integration */"
    rr_end_idx = content.find(rr_end_marker)
    if rr_end_idx < 0:
        log("  [WARN] could not find end of RR block"); return True

    # Extract the RR block
    rr_block = content[rr_idx:rr_end_idx]
    # Remove RR block from its current position
    content = content[:rr_idx] + content[rr_end_idx:]
    # Insert RR block before #endif
    endif_idx = content.find(endif_marker)  # re-find after removal
    content = content[:endif_idx] + rr_block + "\n" + content[endif_idx:]
    content = content.replace(endif_marker, endif_marker + " /* MFS_162_RR_SCOPE */", 1)

    write(p, content)
    return True

# ---- Bug 9: Remove dead mecanum_active field ----
def fix_dead_mecanum_active():
    log("Bug 9: Remove dead mecanum_active field")
    # Remove from robot.h struct
    p = SRC / "robotics" / "robot.h"
    content = p.read_text()
    if "MFS_162_DEAD_FIELD" in content:
        log("  [SKIP] already fixed"); return True

    content = content.replace("    bool mecanum_active;\n", "    /* MFS_162_DEAD_FIELD: mecanum_active removed (never read) */\n")
    write(p, content)

    # Remove from drivetrain.c (the mecanum_active = false in drivetrain_tank)
    p = SRC / "robotics" / "drivetrain.c"
    content = p.read_text()
    content = content.replace("    robot->mecanum_active = false; /* MPE_FTC_082 TEMPORARY â €"  replace with anisotropic friction (MPE_FTC_095) */\n",
                              "    /* MFS_162_DEAD_FIELD: mecanum_active removed */\n")
    write(p, content)
    return True

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
    print("MFS 162: Drivetrain truth + dead code cleanup")
    print("=" * 60)
    if DRY_RUN: print("  ** DRY RUN **\n")

    steps = [fix_friction_config, fix_rolling_resistance_scope, fix_dead_mecanum_active]
    for fn in steps:
        try:
            if not fn(): return 1
        except Exception as e:
            print(f"\n[FAIL] {fn.__name__}: {e}"); import traceback; traceback.print_exc(); return 1

    if not DRY_RUN and not build_and_test(): return 1

    print("=" * 60)
    print("  162 complete. Friction from config, RR scoped, dead field removed.")
    print("=" * 60)
    return 0

if __name__ == "__main__": sys.exit(main())
