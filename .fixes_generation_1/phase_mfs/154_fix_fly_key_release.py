#!/usr/bin/env python3
"""
MFS 154: Fix debug-mode fly keys (Space/Shift) never clearing on release
=========================================================================
BUG:
  space_key_pressed / shift_key_pressed are set true in on_keypress but
  never cleared in on_key_release. In debug mode the camera fly reads these
  flags directly, so:
    - Press Space -> flag sticks -> flies up forever
    - Press Shift -> both true -> +/- cancel -> stops
    - Both now permanently true -> neither key works until mode/focus reset

FIX:
  Clear both flags in on_key_release so fly is held-state
  (fly while held, stop on release).

Files: v15R3/src/ui_input/input_control.c
Usage:
  cd <project_root>
  python3 fixes/phase_mfs/154_fix_fly_key_release.py [--dry-run]
"""
import sys, re, subprocess
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
ROOT = SCRIPT_DIR.parent.parent
SRC = ROOT / "v15R3" / "src"
DRY_RUN = "--dry-run" in sys.argv

def log(msg): print(f"  [154] {msg}")

TARGET = SRC / "ui_input" / "input_control.c"

# Unique anchor: only on_key_release has GDK_KEY_n + n_key_pressed = false
# on the same line. Capture leading indentation so the insert matches style.
ANCHOR = re.compile(
    r'^([ \t]*)if \(event -> keyval == GDK_KEY_n\) '
    r'\{input_state -> n_key_pressed = false;\}.*$',
    re.MULTILINE,
)

def main():
    print("=" * 62)
    print("MFS 154: Fix debug-mode fly keys (Space/Shift) not clearing")
    print("=" * 62)
    if DRY_RUN:
        print("  ** DRY RUN — no files modified **\n")

    if not TARGET.exists():
        log(f"[FAIL] {TARGET} not found")
        return 1

    content = TARGET.read_text()

    if "MFS_154" in content:
        log("[SKIP] already applied")
        return 0

    m = ANCHOR.search(content)
    if not m:
        log("[FAIL] anchor not found; input_control.c may have changed")
        return 1

    indent = m.group(1)
    insert_block = (
        "\n"
        + indent + "if (event -> keyval == GDK_KEY_space) {input_state -> space_key_pressed = false;} /* MFS_154 */\n"
        + indent + "if (event -> keyval == GDK_KEY_Shift_L) {input_state -> shift_key_pressed = false;} /* MFS_154 */"
    )
    new_content = content[:m.end()] + insert_block + content[m.end():]

    if not DRY_RUN:
        TARGET.write_text(new_content)
        log("[OK] added Space/Shift release handlers to on_key_release")

        log("Build check...")
        r = subprocess.run(
            [sys.executable, str(ROOT / "tools" / "build_check.py"), "--quick"],
            cwd=str(ROOT), capture_output=True, text=True, timeout=180,
        )
        print(r.stdout[-2000:] if r.stdout else "")
        if r.returncode != 0:
            print(r.stderr[-2000:] if r.stderr else "")
            log("[FAIL] build failed")
            return 1
        log("[PASS] build clean")

    print("=" * 62)
    print("  154 complete. Space = fly up, Shift = fly down,")
    print("  releasing either now stops the camera.")
    print("=" * 62)
    return 0

if __name__ == "__main__":
    sys.exit(main())
