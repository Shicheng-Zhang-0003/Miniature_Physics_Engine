#!/usr/bin/env python3
"""
MFS 150a: Fix Python string format error in 150
=================================================
Python's .format() choked on the C literal `(vector3){1.0f, 0.0f, 0.0f}`.
Fix: remove the .format() call (indentation is already hardcoded in WHEEL_LOCK).

Usage:
    cd <project_root>
    python3 fixes/phase_mfs/150a_fix_wheel_lock_format.py
    python3 fixes/phase_mfs/150_post_solver_wheel_lock.py
"""
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent

def main():
    print("=" * 60)
    print("MFS 150a: Fix Python string format error in 150")
    print("=" * 60)

    path = SCRIPT_DIR / "150_post_solver_wheel_lock.py"
    content = path.read_text()

    old_line = 'lock_lines = WHEEL_LOCK.format(indent=indent).split("\\n")'
    new_line = 'lock_lines = WHEEL_LOCK.split("\\n")'

    if old_line in content:
        content = content.replace(old_line, new_line)
        path.write_text(content)
        print("  [OK] Removed .format() call")
    else:
        print("  [SKIP] Already fixed")

    print("\nNow run:")
    print("  python3 fixes/phase_mfs/150_post_solver_wheel_lock.py")
    return 0

if __name__ == "__main__":
    sys.exit(main())
