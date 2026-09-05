#!/usr/bin/env python3
"""
MFS 155: Gamepad primary accessor
==================================
Adds gamepad_get_primary() to gamepad.h/gamepad.c so other modules
can access the singleton gamepad state without passing pointers around.
Idempotent: safe to run repeatedly.
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

def log(msg): print(f"  [155] {msg}")

def step_header():
    log("Step 1: Add gamepad_get_primary declaration to gamepad.h")
    path = SRC / "ui_input" / "gamepad.h"
    if not path.exists():
        log("[FAIL] gamepad.h not found (run base gamepad files first)")
        return False
    content = path.read_text()
    if "gamepad_get_primary" in content:
        log("[SKIP] already present")
        return True
    r = Refactor(str(path))
    r.insert_before(
        anchor="#endif /* gamepad_h */",
        text=(
            "/* return a pointer to the engine's primary gamepad state (singleton).\n"
            " * call gamepad_init() on this pointer at startup. */\n"
            "gamepad_state *gamepad_get_primary(void);"
        ),
        label="Add gamepad_get_primary declaration"
    )
    if not DRY_RUN:
        r.apply(dry_run=False)
    else:
        print(r.diff())
    log("[OK] declaration added")
    return True

def step_impl():
    log("Step 2: Add gamepad_get_primary implementation to gamepad.c")
    path = SRC / "ui_input" / "gamepad.c"
    if not path.exists():
        log("[FAIL] gamepad.c not found")
        return False
    content = path.read_text()
    if "gamepad_get_primary" in content:
        log("[SKIP] already present")
        return True
    r = Refactor(str(path))
    impl = (
        "/* MFS_155_GAMEPAD_PRIMARY: singleton gamepad state */\n"
        "static gamepad_state g_primary_gamepad;\n"
        "\n"
        "gamepad_state *gamepad_get_primary(void) {\n"
        "    return &g_primary_gamepad;\n"
        "}\n"
    )
    r.insert_before(
        anchor="bool gamepad_init(",
        text=impl,
        label="Add gamepad_get_primary implementation"
    )
    if not DRY_RUN:
        r.apply(dry_run=False)
    else:
        print(r.diff())
    log("[OK] implementation added")
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

def main():
    print("=" * 60)
    print("MFS 155: Gamepad Primary Accessor")
    print("=" * 60)
    if DRY_RUN: print("  ** DRY RUN **\n")
    if not step_header(): return 1
    if not step_impl(): return 1
    if not DRY_RUN:
        if not step_build(): return 1
    print("=" * 60)
    print("  155 complete. gamepad_get_primary() is now available.")
    print("=" * 60)
    return 0

if __name__ == "__main__":
    sys.exit(main())
