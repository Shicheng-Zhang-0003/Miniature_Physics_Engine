#!/usr/bin/env python3
"""
MFS 159: F310 Gamepad-Only Drive (Remove GVBNCH)
=================================================
1. gamepad.h  – remap axes for F310 XInput (right stick X: 2→3, Y: 3→4)
2. simulation_input_dispatch.c – delete GVBNCH keyboard, gamepad-only drive
"""
import sys, subprocess
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
ROOT = SCRIPT_DIR.parent                      # ← FIX: was .parent.parent
SRC  = ROOT / "v15R3" / "src"
TOOLS = ROOT / "tools"
sys.path.insert(0, str(TOOLS))

DRY_RUN = "--dry-run" in sys.argv

def log(msg): print(f"  [159] {msg}")

# ── Step 1: gamepad.h axis remap ──────────────────────────────────
def step_fix_axes():
    log("Step 1: Remap axes in gamepad.h for F310 XInput")
    p = SRC / "ui_input" / "gamepad.h"
    c = p.read_text()
    if "MFS_159_F310_AXES" in c:
        log("  [SKIP] already applied"); return True

    c = c.replace(
        "#define gamepad_axis_right_x   2",
        "#define gamepad_axis_right_x   3  /* MFS_159_F310 */")
    c = c.replace(
        "#define gamepad_axis_right_y   3",
        "#define gamepad_axis_right_y   4  /* MFS_159_F310 */")
    # add trigger defines after the right_y line
    c = c.replace(
        '#define gamepad_axis_right_y   4  /* MFS_159_F310 */',
        '#define gamepad_axis_right_y   4  /* MFS_159_F310 */\n'
        '#define gamepad_axis_left_trigger  2  /* MFS_159_F310 */\n'
        '#define gamepad_axis_right_trigger 5  /* MFS_159_F310 */')

    if not DRY_RUN: p.write_text(c)
    log("  [OK] axes remapped"); return True

# ── Step 2: dispatch – GVBNCH out, gamepad-only in ────────────────
NEW_BLOCK = """\
    /* MFS_159_GAMEPAD_ONLY: F310 gamepad is the sole drive input.
     * Left stick Y = fwd/back, Left stick X = strafe, Right stick X = rotate. */
    if (gui_robot_get_count() > 0) {
        float drive_forward = 0.0f, drive_strafe = 0.0f, drive_rotate = 0.0f;
        gamepad_state *mfs_pad = gamepad_get_primary();
        if (gamepad_is_connected(mfs_pad)) {
            drive_forward = gamepad_get_axis(mfs_pad, gamepad_axis_left_y);
            drive_strafe  = gamepad_get_axis(mfs_pad, gamepad_axis_left_x);
            drive_rotate  = gamepad_get_axis(mfs_pad, gamepad_axis_right_x);
            if (drive_forward >  1.0f) drive_forward =  1.0f;
            if (drive_forward < -1.0f) drive_forward = -1.0f;
            if (drive_strafe  >  1.0f) drive_strafe  =  1.0f;
            if (drive_strafe  < -1.0f) drive_strafe  = -1.0f;
            if (drive_rotate  >  1.0f) drive_rotate  =  1.0f;
            if (drive_rotate  < -1.0f) drive_rotate  = -1.0f;
        }
        gui_robot_apply_drive(drive_forward, drive_strafe, drive_rotate);
    }"""

def step_fix_dispatch():
    log("Step 2: Replace GVBNCH drive block with gamepad-only")
    p = SRC / "ui_input" / "simulation_input_dispatch.c"
    c = p.read_text()
    if "MFS_159_GAMEPAD_ONLY" in c:
        log("  [SKIP] already applied"); return True

    lines = c.split('\n')
    start = end = None
    depth = 0
    for i, ln in enumerate(lines):
        if 'Robot drive keys:' in ln and start is None:
            start = i
        if start is not None and 'if (gui_robot_get_count() > 0)' in ln:
            depth = 0
        if start is not None and 'gui_robot_get_count' in ln:
            depth += ln.count('{') - ln.count('}')
        elif start is not None and depth > 0:
            depth += ln.count('{') - ln.count('}')
            if depth == 0:
                end = i
                break
        # fallback: track from the if-line itself
        if start is not None and end is None and i > start:
            depth += ln.count('{') - ln.count('}')
            if depth <= 0 and '{' in ''.join(lines[start:i+1]):
                end = i
                break

    if start is None or end is None:
        log("  [FAIL] could not locate drive block"); return False

    new_lines = lines[:start] + NEW_BLOCK.split('\n') + lines[end+1:]
    if not DRY_RUN: p.write_text('\n'.join(new_lines))
    log(f"  [OK] replaced lines {start}–{end}"); return True

# ── Step 3: build ─────────────────────────────────────────────────
def step_build():
    log("Step 3: Build check")
    r = subprocess.run(
        [sys.executable, str(TOOLS / "build_check.py"), "--quick"],
        cwd=str(ROOT), capture_output=True, text=True, timeout=180)
    if r.returncode != 0:
        print(r.stdout[-2000:]); print(r.stderr[-2000:])
        log("[FAIL] build failed"); return False
    log("[PASS] build clean"); return True

def main():
    print("=" * 60)
    print("MFS 159: F310 Gamepad-Only Drive (corrected paths)")
    print("=" * 60)
    if DRY_RUN: print("  ** DRY RUN **\n")
    if not step_fix_axes():     return 1
    if not step_fix_dispatch(): return 1
    if not DRY_RUN and not step_build(): return 1
    print("=" * 60)
    print("  159 done. Next: run 160 to fix strafe direction.")
    print("=" * 60)
    return 0

if __name__ == "__main__": sys.exit(main())
