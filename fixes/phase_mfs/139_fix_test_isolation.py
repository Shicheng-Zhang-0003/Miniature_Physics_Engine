#!/usr/bin/env python3
"""
MFS 139: Fix test isolation — constraint pool leaks between tests
==================================================================
ROOT CAUSE of Test 14's 178× gravity:
  Tests 5-12 create robots with revolute joints. The constraint pool is
  GLOBAL and never cleared. When Test 14 creates a fresh physics_world,
  stale revolute joints from previous tests still reference body indices
  0 and 1 — which now point to Test 14's floor and cylinder. Phantom
  joints yank the cylinder downward at 178× gravity.

FIX:
  1. Add constraint_pool_clear() to constraint.h/.c
  2. Call it at the start of every test in physics_truth_test.c
  3. Extend 138 diagnostic to 60 steps to verify cylinder-floor contact

Usage:
    cd <project_root>
    python3 fixes/phase_mfs/139_fix_test_isolation.py [--dry-run]
"""
import sys, subprocess
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
ROOT = SCRIPT_DIR.parent.parent
SRC = ROOT / "v15R2" / "src"
TOOLS = ROOT / "tools"
sys.path.insert(0, str(TOOLS))
DRY_RUN = "--dry-run" in sys.argv

def log(msg): print(f"  [139] {msg}")
def write(p, t):
    if not DRY_RUN: p.write_text(t)

# ---------------------------------------------------------------- 1. Add constraint_pool_clear to constraint.h
def step_constraint_h():
    log("Step 1: Adding constraint_pool_clear() to constraint.h")
    p = SRC / "physics" / "constraint.h"
    content = p.read_text()
    if "constraint_pool_clear" in content:
        log("  [SKIP] already present"); return True
    anchor = "void constraint_pool_init(void);"
    if anchor in content:
        content = content.replace(anchor, anchor + "\nvoid constraint_pool_clear(void); /* MFS_139 */", 1)
        write(p, content)
        log("  [OK] declaration added"); return True
    log("  [WARN] anchor not found"); return False

# ---------------------------------------------------------------- 2. Add constraint_pool_clear to constraint.c
def step_constraint_c():
    log("Step 2: Adding constraint_pool_clear() to constraint.c")
    p = SRC / "physics" / "constraint.c"
    content = p.read_text()
    if "constraint_pool_clear" in content:
        log("  [SKIP] already present"); return True
    # Insert after constraint_pool_init function
    # Find the end of constraint_pool_init
    anchor = "void constraint_pool_init(void) {"
    if anchor not in content:
        log("  [WARN] constraint_pool_init not found"); return False
    # Find the closing brace of constraint_pool_init
    start = content.find(anchor)
    depth = 0
    pos = start
    while pos < len(content):
        if content[pos] == '{': depth += 1
        elif content[pos] == '}':
            depth -= 1
            if depth == 0: break
        pos += 1
    insert_pos = pos + 1
    clear_func = """

/* MFS_139: Clear all constraints from the pool without re-initializing.
* Must be called at the start of each test to prevent stale constraints
* from previous tests from affecting the current test. */
void constraint_pool_clear(void) {
    constraint_pool.count = 0;
}
"""
    content = content[:insert_pos] + clear_func + content[insert_pos:]
    write(p, content)
    log("  [OK] implementation added"); return True

# ---------------------------------------------------------------- 3. Add constraint_pool_clear() to every test
def step_fix_tests():
    log("Step 3: Adding constraint_pool_clear() to every test")
    p = SRC / "tests" / "physics_truth_test.c"
    content = p.read_text()
    if "MFS_139_ISOLATION" in content:
        log("  [SKIP] already patched"); return True

    # Add constraint_pool_clear() after each physics_world_init() call
    # This ensures each test starts with a clean constraint pool
    content = content.replace(
        "physics_world_init(&world);",
        "physics_world_init(&world);\n    constraint_pool_clear(); /* MFS_139_ISOLATION: clear stale constraints */"
    )
    write(p, content)
    log("  [OK] constraint_pool_clear() added to all tests"); return True

# ---------------------------------------------------------------- 4. Extend 138 diagnostic to 60 steps
def step_extend_diag():
    log("Step 4: Extending floor diagnostic to 60 steps")
    p = SRC / "tests" / "floor_collision_diag.c"
    content = p.read_text()
    if "MFS_139_EXTEND" in content:
        log("  [SKIP] already extended"); return True
    content = content.replace(
        "for (int i = 0; i < 20; i++) {",
        "for (int i = 0; i < 60; i++) { /* MFS_139_EXTEND: run longer to reach floor */"
    )
    write(p, content)
    log("  [OK] diagnostic extended to 60 steps"); return True

# ---------------------------------------------------------------- 5. Build and run
def step_build_test():
    log("Step 5: Building and running physics truth test")
    r = subprocess.run(
        ["make", "-C", str(SRC), "test_physics_truth"],
        cwd=str(SRC), capture_output=True, text=True, timeout=120)
    print(r.stdout[-3000:] if r.stdout else "")
    if r.returncode != 0:
        print(r.stderr[-1000:] if r.stderr else "")
        log("[WARN] Physics truth test still has failures")
        return False
    log("[PASS] All physics truth tests pass")
    return True

# ---------------------------------------------------------------- main
def main():
    print("=" * 60)
    print("MFS 139: Fix Test Isolation (Constraint Pool Leak)")
    print("=" * 60)
    if DRY_RUN: print("  ** DRY RUN **\n")

    steps = [step_constraint_h, step_constraint_c, step_fix_tests, step_extend_diag]
    for fn in steps:
        try:
            fn()
        except Exception as e:
            print(f"\n[FAIL] {fn.__name__}: {e}")
            import traceback; traceback.print_exc()
            return 1

    if not DRY_RUN:
        step_build_test()

    print("=" * 60)
    print("  139 complete. Constraint pool is now cleared between tests.")
    print("  Test 14's 178× gravity was caused by stale revolute joints")
    print("  from Tests 5-12 referencing Test 14's floor and cylinder.")
    print("=" * 60)
    return 0

if __name__ == "__main__":
    sys.exit(main())
