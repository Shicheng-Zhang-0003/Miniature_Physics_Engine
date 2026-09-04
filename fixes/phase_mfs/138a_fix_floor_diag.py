#!/usr/bin/env python3
"""
MFS 138a: Fix floor_collision_diag compile error
=================================================
rigidbody has no member 'is_static' — the field is 'static_state'.

Usage:
    cd <project_root>
    python3 fixes/phase_mfs/138a_fix_floor_diag.py
"""
import subprocess
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
ROOT = SCRIPT_DIR.parent.parent
SRC = ROOT / "v15R2" / "src"

def log(msg): print(f"  [138a] {msg}")

def main():
    print("=" * 60)
    print("MFS 138a: Fix floor_collision_diag compile error")
    print("=" * 60)

    path = SRC / "tests" / "floor_collision_diag.c"
    content = path.read_text()

    if ".is_static" in content:
        content = content.replace(".is_static", ".static_state")
        path.write_text(content)
        log("[OK] is_static -> static_state")
    else:
        log("[SKIP] already fixed")

    log("Rebuilding and running diagnostic...")
    result = subprocess.run(
        ["make", "-C", str(SRC), "test_floor_collision_diag"],
        capture_output=True, text=True, timeout=120)
    print(result.stdout[-4000:] if result.stdout else "")
    if result.returncode != 0:
        print(result.stderr[-1500:] if result.stderr else "")
        log("[FAIL] still failing")
        return 1
    return 0

if __name__ == "__main__":
    sys.exit(main())
