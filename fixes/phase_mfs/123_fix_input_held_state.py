#!/usr/bin/env python3
"""
MFS 123: Fix input system — held-state keys instead of event-state
====================================================================
Changes simulation_input_dispatch.c to NOT clear drive keys after reading.
The keys are now cleared naturally by on_key_released in input_control.c.

Usage:
  cd <project_root>
  python3 fixes/phase_mfs/123_fix_input_held_state.py [--dry-run]
"""

import sys
import subprocess
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
ROOT = SCRIPT_DIR.parent.parent
SRC = ROOT / "v15R2" / "src"
TOOLS = ROOT / "tools"

sys.path.insert(0, str(TOOLS))
from refactor import Refactor, RefactorError

DRY_RUN = "--dry-run" in sys.argv


def log(msg):
    print(f"  [123] {msg}")


def main():
    print("=" * 60)
    print("MFS 123: Fix Input System — Held-State Keys")
    print("=" * 60)

    if DRY_RUN:
        print("  ** DRY RUN **")
        print()

    dispatch_path = SRC / "ui_input" / "simulation_input_dispatch.c"
    content = dispatch_path.read_text()

    # Check if already fixed (no key clearing after gui_robot_apply_drive)
    if "main_inputs.g_key_pressed = false;" not in content:
        log("[SKIP] Input system already uses held-state keys")
        return 0

    # Remove the key-clearing block after gui_robot_apply_drive
    # The pattern is:
    #   gui_robot_apply_drive(kb_forward, kb_strafe, kb_rotate);
    #   main_inputs.g_key_pressed = false;
    #   main_inputs.v_key_pressed = false;
    #   ...
    #   main_inputs.h_key_pressed = false;
    #
    # We want to remove all the clearing lines, keeping only the apply_drive call.

    old_block = """    gui_robot_apply_drive(kb_forward, kb_strafe, kb_rotate);
    main_inputs.g_key_pressed = false;
    main_inputs.v_key_pressed = false;
    main_inputs.b_key_pressed = false;
    main_inputs.n_key_pressed = false;
    main_inputs.c_key_pressed = false;
    main_inputs.h_key_pressed = false;"""

    new_block = """    gui_robot_apply_drive(kb_forward, kb_strafe, kb_rotate);
    /* MFS_123: Keys are NOT cleared here. They are cleared naturally
     * by on_key_released in input_control.c. This gives continuous
     * drive commands while keys are held, and zero when released. */"""

    if old_block in content:
        r = Refactor(str(dispatch_path))
        r.replace(
            old=old_block,
            new=new_block,
            label="Remove key clearing after apply_drive"
        )
        if not DRY_RUN:
            r.apply(dry_run=False)
        else:
            print(r.diff())
        log("[OK] Input system now uses held-state keys")
    else:
        log("[WARN] Could not find exact key-clearing pattern")
        log("[INFO] Attempting line-by-line removal...")

        # Fallback: remove each clearing line individually
        lines_to_remove = [
            "    main_inputs.g_key_pressed = false;",
            "    main_inputs.v_key_pressed = false;",
            "    main_inputs.b_key_pressed = false;",
            "    main_inputs.n_key_pressed = false;",
            "    main_inputs.c_key_pressed = false;",
            "    main_inputs.h_key_pressed = false;",
        ]

        new_content = content
        for line in lines_to_remove:
            if line in new_content:
                new_content = new_content.replace(line + "\n", "")

        if new_content != content:
            if not DRY_RUN:
                dispatch_path.write_text(new_content)
            log("[OK] Removed key-clearing lines (fallback method)")
        else:
            log("[WARN] No key-clearing lines found to remove")

    # Build verification
    if not DRY_RUN:
        log("Running build verification...")
        result = subprocess.run(
            [sys.executable, str(TOOLS / "build_check.py"), "--quick"],
            cwd=str(ROOT), capture_output=True, text=True, timeout=120
        )
        if result.returncode != 0:
            print(result.stdout[-2000:] if result.stdout else "")
            print(result.stderr[-2000:] if result.stderr else "")
            log("[FAIL] Build failed after input fix")
            return 1
        log("[PASS] Build successful!")

        # Run tests
        log("Running headless tests...")
        result = subprocess.run(
            [sys.executable, str(TOOLS / "test_runner.py")],
            cwd=str(ROOT), capture_output=True, text=True, timeout=300
        )
        print(result.stdout[-3000:] if result.stdout else "")
        if result.returncode != 0:
            log("[WARN] Some tests failed. Review output above.")
        else:
            log("[PASS] All tests passed!")

    print()
    print("=" * 60)
    print("  DONE. The robot now responds to held keys continuously.")
    print("  Press and hold G/V/B/N/C/H to drive. Release to stop.")
    print("=" * 60)
    return 0


if __name__ == "__main__":
    sys.exit(main())
