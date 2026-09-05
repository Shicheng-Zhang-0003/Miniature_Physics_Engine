#!/usr/bin/env python3
"""
MFS 160: Fix gamepad strafe direction (left/right reversed)
=============================================================
Negates the left-stick-X read in the dispatch file.
No root_gtk.c changes needed.
"""
import sys, subprocess
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
ROOT = SCRIPT_DIR.parent                      # ← FIX: was .parent.parent
SRC  = ROOT / "v15R3" / "src"
TOOLS = ROOT / "tools"

DRY_RUN = "--dry-run" in sys.argv

def log(msg): print(f"  [160] {msg}")

def step_fix():
    log("Step 1: Negate strafe axis in dispatch")
    p = SRC / "ui_input" / "simulation_input_dispatch.c"
    c = p.read_text()
    if "MFS_160_STRAFE_NEG" in c:
        log("  [SKIP] already applied"); return True

    # post-159 form
    if "drive_strafe  = gamepad_get_axis(mfs_pad, gamepad_axis_left_x);" in c:
        c = c.replace(
            "drive_strafe  = gamepad_get_axis(mfs_pad, gamepad_axis_left_x);",
            "drive_strafe  = -gamepad_get_axis(mfs_pad, gamepad_axis_left_x); /* MFS_160_STRAFE_NEG */")
        if not DRY_RUN: p.write_text(c)
        log("  [OK] negated (post-159 form)"); return True

    # pre-159 fallback (158a form)
    if "kb_strafe  += gamepad_get_axis(mfs_pad, gamepad_axis_left_x);" in c:
        c = c.replace(
            "kb_strafe  += gamepad_get_axis(mfs_pad, gamepad_axis_left_x);",
            "kb_strafe  += -gamepad_get_axis(mfs_pad, gamepad_axis_left_x); /* MFS_160_STRAFE_NEG */")
        if not DRY_RUN: p.write_text(c)
        log("  [OK] negated (pre-159 fallback)"); return True

    log("  [FAIL] strafe line not found – run 159 first"); return False

def step_build():
    log("Step 2: Build check")
    r = subprocess.run(
        [sys.executable, str(TOOLS / "build_check.py"), "--quick"],
        cwd=str(ROOT), capture_output=True, text=True, timeout=180)
    if r.returncode != 0:
        print(r.stdout[-2000:]); print(r.stderr[-2000:])
        log("[FAIL] build failed"); return False
    log("[PASS] build clean"); return True

def main():
    print("=" * 60)
    print("MFS 160: Fix Strafe Direction (corrected paths)")
    print("=" * 60)
    if DRY_RUN: print("  ** DRY RUN **\n")
    if not step_fix(): return 1
    if not DRY_RUN and not step_build(): return 1
    print("=" * 60)
    print("  160 done. Stick right = strafe right.")
    print("=" * 60)
    return 0

if __name__ == "__main__": sys.exit(main())
