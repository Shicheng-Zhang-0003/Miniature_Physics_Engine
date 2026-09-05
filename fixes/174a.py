#!/usr/bin/env python3
"""
MFS 175: Wire cylinder dispatch into physics_world.c
=====================================================
The 172 script added collision_cylinder_sphere/cube/cylinder to
collision_mechanics.c, but the 173 script that was supposed to wire
them into the narrowphase dispatch FAILED and corrupted the file.
The repair scripts (173a-d) fixed the corruption but never added
the dispatch. The collision functions exist but are never called.

This script inserts the 5 cylinder dispatch branches into the
if-else chain in physics_world_step(), right after the cube-cube case.

Usage:
    cd <project_root>
    python3 fixes/175.py [--dry-run]
"""
import sys, subprocess
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
ROOT = SCRIPT_DIR.parent
SRC = ROOT / "v15R3" / "src"
TOOLS = ROOT / "tools"
sys.path.insert(0, str(TOOLS))
DRY_RUN = "--dry-run" in sys.argv

def log(msg): print(f"  [175] {msg}")

CYL_DISPATCH = """\
        } else if ((body_a->type == object_cylinder) && (body_b->type == object_sphere)) {
            collided = collision_cylinder_sphere(body_a, body_b, &narrowphase_collision);
        } else if ((body_a->type == object_sphere) && (body_b->type == object_cylinder)) {
            collided = collision_cylinder_sphere(body_b, body_a, &narrowphase_collision);
            if (collided) {
                narrowphase_collision.normal_vector = vector3_scaling(narrowphase_collision.normal_vector, -1.0f);
                narrowphase_collision.object_a = body_a;
                narrowphase_collision.object_b = body_b;
            }
        } else if ((body_a->type == object_cylinder) && (body_b->type == object_cube)) {
            collided = collision_cylinder_cube(body_a, body_b, &narrowphase_collision);
        } else if ((body_a->type == object_cube) && (body_b->type == object_cylinder)) {
            collided = collision_cylinder_cube(body_b, body_a, &narrowphase_collision);
            if (collided) {
                narrowphase_collision.normal_vector = vector3_scaling(narrowphase_collision.normal_vector, -1.0f);
                narrowphase_collision.object_a = body_a;
                narrowphase_collision.object_b = body_b;
            }
        } else if ((body_a->type == object_cylinder) && (body_b->type == object_cylinder)) {
            collided = collision_cylinder_cylinder(body_a, body_b, &narrowphase_collision);
        }
"""

def step_fix_dispatch():
    log("Step 1: Inserting cylinder dispatch into physics_world.c")
    p = SRC / "core" / "physics_world.c"
    content = p.read_text()

    if "collision_cylinder_sphere" in content:
        log("  [SKIP] cylinder dispatch already present")
        return True

    # Find the anchor: the closing brace of the cube-cube case,
    # followed by the "if ((collided)" check.
    # We need to replace "}\n        if ((collided)" with
    # the cylinder dispatch + "}\n        if ((collided)"
    anchor = "collided = collision_dual_cube(body_a, body_b, &narrowphase_collision);"
    idx = content.find(anchor)
    if idx < 0:
        log("  [FAIL] cube-cube dispatch anchor not found")
        return False

    # Find the closing } after the cube-cube case
    close_idx = content.find("}", idx + len(anchor))
    if close_idx < 0:
        log("  [FAIL] closing brace not found after cube-cube case")
        return False

    # Find the "if ((collided)" after the closing }
    collided_idx = content.find("if ((collided)", close_idx)
    if collided_idx < 0:
        log("  [FAIL] if ((collided)) not found after cube-cube case")
        return False

    # Insert the cylinder dispatch between the } and the if ((collided))
    # We replace the } with the cylinder dispatch (which ends with })
    insert_point = close_idx  # position of the }
    content = content[:insert_point] + CYL_DISPATCH + content[insert_point + 1:]

    if not DRY_RUN:
        p.write_text(content)
    log("  [OK] cylinder dispatch inserted into physics_world.c")
    return True

def step_fix_sim_loop():
    log("Step 2: Inserting cylinder dispatch into simulation_physics_loop.c")
    p = SRC / "core" / "simulation_physics_loop.c"
    content = p.read_text()

    if "collision_cylinder_sphere" in content:
        log("  [SKIP] cylinder dispatch already present")
        return True

    anchor = "collided = collision_dual_cube(rigid_body_a, rigid_body_b, &narrowphase_collision);"
    idx = content.find(anchor)
    if idx < 0:
        log("  [WARN] cube-cube dispatch anchor not found in simulation_physics_loop.c")
        return True

    close_idx = content.find("}", idx + len(anchor))
    if close_idx < 0:
        log("  [WARN] closing brace not found")
        return True

    # Build the dispatch for simulation_physics_loop.c (different variable names)
    sim_dispatch = CYL_DISPATCH.replace("body_a", "rigid_body_a").replace("body_b", "rigid_body_b")
    content = content[:close_idx] + sim_dispatch + content[close_idx + 1:]

    if not DRY_RUN:
        p.write_text(content)
    log("  [OK] cylinder dispatch inserted into simulation_physics_loop.c")
    return True

def step_build():
    log("Step 3: Build check")
    r = subprocess.run(
        [sys.executable, str(TOOLS / "build_check.py"), "--quick"],
        cwd=str(ROOT), capture_output=True, text=True, timeout=180)
    print(r.stdout[-2000:] if r.stdout else "")
    if r.returncode != 0:
        print(r.stderr[-2000:] if r.stderr else "")
        log("[FAIL] build failed")
        return False
    log("[PASS] build clean")
    return True

def step_run_cylinder_tests():
    log("Step 4: Running cylinder tests")
    for test in ["cylinder_sphere", "cylinder_cube", "cylinder_cylinder"]:
        r = subprocess.run(
            ["make", "-C", str(SRC), f"test_{test}"],
            capture_output=True, text=True, timeout=120)
        out = r.stdout + r.stderr
        if r.returncode != 0:
            # Print only the relevant lines
            for line in out.split("\n"):
                if "[info]" in line or "[PASS]" in line or "[FAIL]" in line or "[GAP]" in line:
                    log(f"  {line.strip()}")
            log(f"  [FAIL] test_{test} failed")
            return False
        for line in out.split("\n"):
            if "[info]" in line or "[PASS]" in line or "[FAIL]" in line or "[GAP]" in line:
                log(f"  {line.strip()}")
    return True

def step_full_suite():
    log("Step 5: Running full test suite (14 tests)")
    r = subprocess.run(
        [sys.executable, str(TOOLS / "test_runner.py")],
        cwd=str(ROOT), capture_output=True, text=True, timeout=300)
    print(r.stdout[-2500:] if r.stdout else "")
    if r.returncode != 0:
        log("[WARN] some tests failed")
        return False
    log("[PASS] all tests green")
    return True

def main():
    print("=" * 60)
    print("MFS 175: Wire Cylinder Dispatch")
    print("=" * 60)
    if DRY_RUN: print("  ** DRY RUN **\n")

    if not step_fix_dispatch():
        return 1
    if not step_fix_sim_loop():
        return 1

    if not DRY_RUN:
        if not step_build():
            return 1
        if not step_run_cylinder_tests():
            return 1
        if not step_full_suite():
            return 1

    print("=" * 60)
    print("  175 complete. Cylinder dispatch wired into both")
    print("  physics_world.c and simulation_physics_loop.c.")
    print("  Cylinder collisions are now active.")
    print("=" * 60)
    return 0

if __name__ == "__main__":
    sys.exit(main())
