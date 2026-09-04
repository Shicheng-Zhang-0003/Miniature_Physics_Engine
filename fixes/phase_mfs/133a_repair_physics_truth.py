#!/usr/bin/env python3
"""
MFS 133a: Repair physics_truth_test.c build failure
====================================================
Fixes:
1. contact_cache_clear() -> contact_cache_clear(NULL) (post-131a signature)
2. Fix any missing includes
3. Fix any missing function references

Usage:
    cd <project_root>
    python3 fixes/phase_mfs/133a_repair_physics_truth.py [--dry-run]
"""
import sys
import subprocess
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
ROOT = SCRIPT_DIR.parent.parent
SRC = ROOT / "v15R2" / "src"
TOOLS = ROOT / "tools"
sys.path.insert(0, str(TOOLS))
DRY_RUN = "--dry-run" in sys.argv

def log(msg): print(f"  [133a] {msg}")

def write(p, t):
    if not DRY_RUN: p.write_text(t)
    log(f"  [OK] {p.name}")

# ============================================================
# STEP 1: Show full build output to identify the exact error
# ============================================================
def step_show_build_output():
    log("Step 1: Showing full build output")
    result = subprocess.run(
        ["make", "-C", str(SRC), "test_physics_truth"],
        cwd=str(SRC), capture_output=True, text=True, timeout=120
    )
    print(result.stdout[-4000:] if result.stdout else "")
    if result.returncode != 0:
        print(result.stderr[-2000:] if result.stderr else "")
        log("[INFO] Build failed — showing full output above")
        return True  # Continue to fix
    log("[INFO] Build succeeded — no fix needed")
    return False  # No fix needed

# ============================================================
# STEP 2: Fix contact_cache_clear() calls
# ============================================================
def step_fix_contact_cache_clear():
    log("Step 2: Fixing contact_cache_clear() calls")
    p = SRC / "tests" / "physics_truth_test.c"
    content = p.read_text()

    # Fix contact_cache_clear() -> contact_cache_clear(NULL)
    if "contact_cache_clear()" in content:
        content = content.replace("contact_cache_clear()", "contact_cache_clear(NULL)")
        write(p, content)
        log("  [OK] contact_cache_clear() -> contact_cache_clear(NULL)")
    else:
        log("  [SKIP] contact_cache_clear() already fixed")
    return True

# ============================================================
# STEP 3: Fix missing includes
# ============================================================
def step_fix_includes():
    log("Step 3: Fixing missing includes")
    p = SRC / "tests" / "physics_truth_test.c"
    content = p.read_text()

    # Check if we have the right includes
    if '#include "robotics/robot.h"' not in content:
        content = content.replace(
            '#include "config/mpe_config.h"',
            '#include "config/mpe_config.h"\n#include "robotics/robot.h"'
        )
        log("  [OK] Added robot.h include")
    else:
        log("  [SKIP] robot.h include already present")

    write(p, content)
    return True

# ============================================================
# STEP 4: Re-run the test
# ============================================================
def step_run_test():
    log("Step 4: Re-running physics_truth test")
    result = subprocess.run(
        [sys.executable, str(TOOLS / "test_runner.py"), "physics_truth"],
        cwd=str(ROOT), capture_output=True, text=True, timeout=300
    )
    print(result.stdout[-3000:] if result.stdout else "")
    if result.returncode != 0:
        log("[WARN] physics_truth test still failing — check output above")
        return False
    log("[PASS] physics_truth test passed!")
    return True

# ============================================================
# MAIN
# ============================================================
def main():
    print("=" * 60)
    print("MFS 133a: Repair physics_truth_test.c")
    print("=" * 60)
    if DRY_RUN:
        print("  ** DRY RUN **\n")

    # First show the build output
    needs_fix = step_show_build_output()

    if needs_fix:
        step_fix_contact_cache_clear()
        step_fix_includes()

    step_run_test()

    print()
    print("=" * 60)
    print("  133a complete. Check output above for results.")
    print("=" * 60)
    return 0

if __name__ == "__main__":
    sys.exit(main())
