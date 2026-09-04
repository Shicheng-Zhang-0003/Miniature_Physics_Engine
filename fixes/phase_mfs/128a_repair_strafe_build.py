#!/usr/bin/env python3
"""
MFS 128a: Verify build with MFS_DEBUG_STRAFE
=============================================
The 128 script fixed the scope error in collision_mechanics.c but crashed
with a NameError. The C fix was already applied. This script just verifies
the build works.

Usage:
  cd <project_root>
  python3 fixes/phase_mfs/128a_repair_strafe_build.py
"""

import subprocess
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
ROOT = SCRIPT_DIR.parent.parent
SRC = ROOT / "v15R2" / "src"

print("=" * 60)
print("MFS 128a: Verify build with MFS_DEBUG_STRAFE")
print("=" * 60)
print()

print("Building with MFS_DEBUG_STRAFE...")
result = subprocess.run(
    ["make", "-C", str(SRC), "-j4"],
    env={
        **dict(__import__('os').environ),
        "CFLAGS": "-I/usr/include/gtk-3.0 -I/usr/include/pango-1.0 -I/usr/include/glib-2.0 "
                  "-I/usr/lib/x86_64-linux-gnu/glib-2.0/include -I/usr/include/harfbuzz "
                  "-I/usr/include/freetype2 -I/usr/include/libpng16 -I/usr/include/libmount "
                  "-I/usr/include/blkid -I/usr/include/fribidi -I/usr/include/cairo "
                  "-I/usr/include/pixman-1 -I/usr/include/gdk-pixbuf-2.0 -I/usr/include/x86_64-linux-gnu "
                  "-I/usr/include/webp -I/usr/include/gio-unix-2.0 -I/usr/include/atk-1.0 "
                  "-I/usr/include/at-spi2-atk/2.0 -I/usr/include/at-spi-2.0 -I/usr/include/dbus-1.0 "
                  "-I/usr/lib/x86_64-linux-gnu/dbus-1.0/include -pthread -I. -O3 -Wall -Wextra "
                  "-MMD -MP -DMFS_DEBUG_STRAFE"
    },
    capture_output=True, text=True, timeout=180
)

if result.returncode != 0:
    print("[FAIL] Build failed:")
    print(result.stdout[-2000:] if result.stdout else "")
    print(result.stderr[-2000:] if result.stderr else "")
    sys.exit(1)

print("[PASS] Build successful with MFS_DEBUG_STRAFE!")
print()
print("You can now build and run the engine with strafe diagnostics:")
print("  cd v15R2/src")
print("  make CFLAGS=\"$(pkg-config --cflags gtk+-3.0 epoxy) -I. -O3 -Wall -Wextra -MMD -MP -DMFS_DEBUG_STRAFE\"")
print("  ./engine")
print()
print("Then drive with V/N and watch for [STRAFE_DIAG] output.")
sys.exit(0)
