#!/usr/bin/env python3
"""
MFS 128: Fix MFS_DEBUG_STRAFE scope error in collision_mechanics.c
====================================================================
The MFS_DEBUG_STRAFE diagnostic block references `grip_dir` which is
declared inside a nested if-block. Move the diagnostic to a position
where grip_dir is in scope, or restructure to declare it at function scope.

Usage:
  cd <project_root>
  python3 fixes/phase_mfs/128_fix_strafe_diag_scope.py [--dry-run]
"""

import sys
import subprocess
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
ROOT = SCRIPT_DIR.parent.parent
SRC = ROOT / "v15R3" / "src"
TOOLS = ROOT / "tools"

sys.path.insert(0, str(TOOLS))

DRY_RUN = "--dry-run" in sys.argv


def log(msg):
    print(f"  [128] {msg}")


def main():
    print("=" * 60)
    print("MFS 128: Fix MFS_DEBUG_STRAFE scope error")
    print("=" * 60)

    if DRY_RUN:
        print("  ** DRY RUN **")
        print()

    path = SRC / "physics" / "collision_mechanics.c"
    content = path.read_text()

    # Check if the diagnostic code exists and has the scope issue
    if "MFS_DEBUG_STRAFE" not in content:
        log("[SKIP] MFS_DEBUG_STRAFE not found in collision_mechanics.c")
        return 0

    if "grip_dir" not in content:
        log("[SKIP] grip_dir not found — different issue")
        return 0

    # The problem: grip_dir is declared inside:
    #   if (axle_proj_len > 0.0001f) { ... vector3 grip_dir = ...; ... }
    # But the diagnostic tries to use it outside that block.
    #
    # Fix: Move the diagnostic INSIDE the block where grip_dir is declared,
    # right after grip_dir is computed and before it's used for tangent_vector.

    # Strategy: Find the MFS_DEBUG_STRAFE block and move it inside the
    # grip_dir scope. The simplest fix is to wrap the diagnostic in its
    # own scope check.

    # Find the diagnostic block
    diag_start_marker = "#ifdef MFS_DEBUG_STRAFE"
    diag_end_marker = "#endif"

    if diag_start_marker not in content:
        log("[SKIP] Could not find MFS_DEBUG_STRAFE block")
        return 0

    # Find the diagnostic block boundaries
    diag_start = content.find(diag_start_marker)
    if diag_start < 0:
        log("[SKIP] MFS_DEBUG_STRAFE not found")
        return 0

    # Find the matching #endif
    diag_end = content.find(diag_end_marker, diag_start)
    if diag_end < 0:
        log("[SKIP] Could not find end of MFS_DEBUG_STRAFE block")
        return 0

    diag_block = content[diag_start:diag_end + len(diag_end_marker)]

    # Remove the old diagnostic block
    content = content[:diag_start] + content[diag_end + len(diag_end_marker):]

    # Now insert the diagnostic INSIDE the grip_dir scope.
    # Find the line: "vector3 grip_dir = vector3_cross(floor_normal, roller_free);"
    grip_decl = "vector3 grip_dir = vector3_cross(floor_normal, roller_free);"
    grip_pos = content.find(grip_decl)

    if grip_pos < 0:
        log("[SKIP] Could not find grip_dir declaration")
        return 0

    # Find the end of that line
    grip_line_end = content.find("\n", grip_pos)
    if grip_line_end < 0:
        grip_line_end = len(content)

    # Insert the diagnostic right after grip_dir declaration
    # But we need to make sure it's inside the if block
    # Find the next line after grip_dir declaration
    insert_pos = grip_line_end + 1

    # Build the diagnostic block that works within the grip_dir scope
    new_diag = """
/* MFS_DEBUG_STRAFE: diagnostic for mecanum strafe debugging */
#ifdef MFS_DEBUG_STRAFE
{
    static int strafe_diag_counter = 0;
    if ((strafe_diag_counter++ % 60) == 0) {
        float grip_len = vector3_length(grip_dir);
        printf("[STRAFE_DIAG] roller_angle=%.2f rad grip_len=%.4f mecanum=%d\\n",
               mecanum_wheel->roller_angle_rad, grip_len, mecanum_wheel->is_mecanum ? 1 : 0);
    }
}
#endif
"""

    content = content[:insert_pos] + new_diag + content[insert_pos:]

    if not DRY_RUN:
        path.write_text(content)
        log("[OK] MFS_DEBUG_STRAFE diagnostic moved inside grip_dir scope")
    else:
        log("[DRY RUN] Would move diagnostic inside grip_dir scope")

    # Build verification
    if not DRY_RUN:
        log("Building with MFS_DEBUG_STRAFE...")
        result = subprocess.run(
            ["make", "-C", str(SRC),
             "CFLAGS=-I/usr/include/gtk-3.0 -I/usr/include/pango-1.0 -I/usr/include/glib-2.0 "
             "-I/usr/lib/x86_64-linux-gnu/glib-2.0/include -I/usr/include/harfbuzz "
             "-I/usr/include/freetype2 -I/usr/include/libpng16 -I/usr/include/libmount "
             "-I/usr/include/blkid -I/usr/include/fribidi -I/usr/include/cairo "
             "-I/usr/include/pixman-1 -I/usr/include/gdk-pixbuf-2.0 -I/usr/include/x86_64-linux-gnu "
             "-I/usr/include/webp -I/usr/include/gio-unix-2.0 -I/usr/include/atk-1.0 "
             "-I/usr/include/at-spi2-atk/2.0 -I/usr/include/at-spi-2.0 -I/usr/include/dbus-1.0 "
             "-I/usr/lib/x86_64-linux-gnu/dbus-1.0/include -pthread -I. -O3 -Wall -Wextra "
             "-MMD -MP -DMFS_DEBUG_STRAFE"],
            cwd=str(SRC), capture_output=True, text=True, timeout=120
        )
        if proc.returncode != 0:
            print(proc.stdout[-2000:] if proc.stdout else "")
            print(proc.stderr[-2000:] if proc.stderr else "")
            log("[FAIL] Build failed")
            return 1
        log("[PASS] Build successful with MFS_DEBUG_STRAFE")

    print()
    print("=" * 60)
    print("  128 complete. MFS_DEBUG_STRAFE diagnostic is now in scope.")
    print("  Build with: make CFLAGS='... -DMFS_DEBUG_STRAFE'")
    print("=" * 60)
    return 0


if __name__ == "__main__":
    sys.exit(main())
