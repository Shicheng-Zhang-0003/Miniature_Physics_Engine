#!/usr/bin/env python3
"""
MFS 139a: Fix linker error — replace constraint_pool_clear with constraint_pool_init
=====================================================================================
139 added constraint_pool_clear() calls to the test file but failed to add
the function to constraint.c/h (anchor mismatch: actual code uses
'constraint_pool_init (void)' with a space).

constraint_pool_init() already does exactly what constraint_pool_clear()
would do: marks all constraints inactive and resets the count. Safe to
call multiple times. Just swap the calls.

Usage:
    cd <project_root>
    python3 fixes/phase_mfs/139a_fix_linker_error.py [--dry-run]
"""
import sys, subprocess
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
ROOT = SCRIPT_DIR.parent.parent
SRC = ROOT / "v15R3" / "src"
TOOLS = ROOT / "tools"
sys.path.insert(0, str(TOOLS))
DRY_RUN = "--dry-run" in sys.argv

def log(msg): print(f"  [139a] {msg}")

def main():
    print("=" * 60)
    print("MFS 139a: Fix Linker Error (constraint_pool_clear → constraint_pool_init)")
    print("=" * 60)
    if DRY_RUN: print("  ** DRY RUN **\n")

    # Replace constraint_pool_clear() with constraint_pool_init() in test file
    test_path = SRC / "tests" / "physics_truth_test.c"
    content = test_path.read_text()

    count = content.count("constraint_pool_clear()")
    if count > 0:
        content = content.replace("constraint_pool_clear()", "constraint_pool_init()")
        if not DRY_RUN:
            test_path.write_text(content)
        log(f"[OK] Replaced {count} constraint_pool_clear() → constraint_pool_init()")
    else:
        log("[SKIP] No constraint_pool_clear() calls found")

    if not DRY_RUN:
        log("Building and running physics truth test...")
        r = subprocess.run(
            ["make", "-C", str(SRC), "test_physics_truth"],
            cwd=str(SRC), capture_output=True, text=True, timeout=120)
        print(r.stdout[-4000:] if r.stdout else "")
        if r.returncode != 0:
            print(r.stderr[-1500:] if r.stderr else "")
            log("[FAIL] Build or test failed")
            return 1
        log("[PASS] Physics truth test complete")

    print("=" * 60)
    return 0

if __name__ == "__main__":
    sys.exit(main())
