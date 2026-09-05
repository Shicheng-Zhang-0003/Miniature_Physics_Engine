#!/usr/bin/env python3
"""
MFS 156: Wire gamepad init/close into engine lifecycle
=======================================================
1. Adds #include "ui_input/gamepad.h" to mpe_engine.h (so all files see it)
2. Calls gamepad_init() in root_gtk.c after mpe_config_init()
3. Sets invert_left_y = true (stick up = forward)
4. Calls gamepad_close() in on_main_window_destroy before gtk_main_quit()
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

def log(msg): print(f"  [156] {msg}")

def step_engine_include():
    log("Step 1: Add gamepad.h include to mpe_engine.h")
    path = SRC / "mpe_engine.h"
    content = path.read_text()
    if "gamepad.h" in content:
        log("[SKIP] already present")
        return True
    r = Refactor(str(path))
    r.insert_after(
        anchor='#include "ui_input/microvim.h"',
        text='#include "ui_input/gamepad.h" /* MFS_156 */',
        label="Add gamepad.h include"
    )
    if not DRY_RUN:
        r.apply(dry_run=False)
    else:
        print(r.diff())
    log("[OK] include added")
    return True

def step_init():
    log("Step 2: Wire gamepad_init into root_gtk.c startup")
    path = SRC / "root_gtk.c"
    content = path.read_text()
    if "MFS_156_GAMEPAD_INIT" in content:
        log("[SKIP] already present")
        return True
    r = Refactor(str(path))
    r.insert_after(
        anchor='mpe_config_init();',
        text=(
            "    /* MFS_156_GAMEPAD_INIT: open the gamepad device */\n"
            "    gamepad_init(gamepad_get_primary(), NULL);\n"
            "    gamepad_get_primary()->invert_left_y = true; /* stick up = forward */"
        ),
        label="Add gamepad_init call"
    )
    if not DRY_RUN:
        r.apply(dry_run=False)
    else:
        print(r.diff())
    log("[OK] init wired")
    return True

def step_close():
    log("Step 3: Wire gamepad_close into root_gtk.c shutdown")
    path = SRC / "root_gtk.c"
    content = path.read_text()
    if "MFS_156_GAMEPAD_CLOSE" in content:
        log("[SKIP] already present")
        return True
    r = Refactor(str(path))
    r.insert_before(
        anchor="gtk_main_quit();",
        text="    /* MFS_156_GAMEPAD_CLOSE */\n    gamepad_close(gamepad_get_primary());",
        label="Add gamepad_close call"
    )
    if not DRY_RUN:
        r.apply(dry_run=False)
    else:
        print(r.diff())
    log("[OK] close wired")
    return True

def step_build():
    log("Step 4: Build verification")
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

def main():
    print("=" * 60)
    print("MFS 156: Gamepad Init/Close Wiring")
    print("=" * 60)
    if DRY_RUN: print("  ** DRY RUN **\n")
    if not step_engine_include(): return 1
    if not step_init(): return 1
    if not step_close(): return 1
    if not DRY_RUN:
        if not step_build(): return 1
    print("=" * 60)
    print("  156 complete. gamepad opens at startup, closes at exit.")
    print("=" * 60)
    return 0

if __name__ == "__main__":
    sys.exit(main())
