#!/usr/bin/env python3
"""
MFS 140: Fix Test 10 — wake the cube before sliding
=====================================================
Test 10's cube settles for 120 steps and goes to SLEEP. Setting
velocity.x = 2.0 directly does not wake a sleeping body, so the solver
skips it and applies zero kinetic friction (actual_decel=0.0000).

Same bug class as project fix 078 ("driven wheels woken so motor torque
is applied"). The truthful fix: imparting velocity to a resting body
means it is no longer at rest, so wake it — exactly like a real shove.

Usage:
    cd <project_root>
    python3 fixes/phase_mfs/140_wake_sliding_cube.py [--dry-run]
"""
import sys, subprocess
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
ROOT = SCRIPT_DIR.parent.parent
SRC = ROOT / "v15R3" / "src"
TOOLS = ROOT / "tools"
sys.path.insert(0, str(TOOLS))
DRY_RUN = "--dry-run" in sys.argv

def log(msg): print(f"  [140] {msg}")

def main():
    print("=" * 60)
    print("MFS 140: Wake Sliding Cube (Test 10 kinetic friction)")
    print("=" * 60)
    if DRY_RUN: print("  ** DRY RUN **\n")

    # Verify rigidbody_wake exists
    rb_h = SRC / "core" / "rigidbody.h"
    if "rigidbody_wake" not in rb_h.read_text():
        log("[FAIL] rigidbody_wake not declared in rigidbody.h — aborting")
        return 1
    log("[OK] rigidbody_wake is declared")

    test_path = SRC / "tests" / "physics_truth_test.c"
    content = test_path.read_text()

    if "MFS_140_WAKE" in content:
        log("[SKIP] already patched")
    else:
        anchor = "world.bodies[idx].velocity = (vector3){2.0f, 0.0f, 0.0f};"
        if anchor not in content:
            log("[FAIL] velocity-set anchor not found in Test 10")
            return 1
        replacement = (anchor + "\n"
                       "    rigidbody_wake(&world.bodies[idx]); "
                       "/* MFS_140_WAKE: imparting velocity wakes the body (a real push). "
                       "Without this the settled cube stays asleep and the solver skips it, "
                       "so no kinetic friction is applied. */")
        content = content.replace(anchor, replacement, 1)
        if not DRY_RUN:
            test_path.write_text(content)
        log("[OK] rigidbody_wake() added after velocity set")

    if not DRY_RUN:
        log("Building and running physics truth test...")
        r = subprocess.run(
            ["make", "-C", str(SRC), "test_physics_truth"],
            cwd=str(SRC), capture_output=True, text=True, timeout=180)
        print(r.stdout[-3500:] if r.stdout else "")
        if r.returncode != 0:
            print(r.stderr[-1500:] if r.stderr else "")
            log("[WARN] test still failing — see output")
            return 1
        log("[PASS] physics truth test complete")

    print("=" * 60)
    return 0

if __name__ == "__main__":
    sys.exit(main())
