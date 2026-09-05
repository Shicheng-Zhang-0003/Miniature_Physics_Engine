#!/usr/bin/env python3
"""
MFS 173: Wire cylinder narrowphase into both dispatch sites
============================================================
Adds cylinder-vs-sphere, cylinder-vs-cube, and cylinder-vs-cylinder
cases to the narrowphase dispatch in:
  1. physics_world.c  → physics_world_step()
  2. simulation_physics_loop.c → simulation_physics_tick()

Each site gets 5 new else-if branches (cyl-sph, sph-cyl, cyl-cube,
cube-cyl, cyl-cyl). The swapped-order cases flip the normal.

Usage:
    cd <project_root>
    python3 fixes/173_cylinder_dispatch.py [--dry-run]
"""
import sys, subprocess
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
ROOT = SCRIPT_DIR.parent
SRC = ROOT / "v15R3" / "src"
TOOLS = ROOT / "tools"
sys.path.insert(0, str(TOOLS))

DRY_RUN = "--dry-run" in sys.argv

def log(msg): print(f"  [173] {msg}")

def write(p, t):
    if not DRY_RUN: p.write_text(t)
    log(f"  [OK] {p.relative_to(SRC)}")

# The 5 new dispatch branches. Appended after the last existing else-if.
DISPATCH_BLOCK = r'''
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
    }'''

# The anchor: the last existing else-if in each dispatch.
# In physics_world.c it's the cube-cube case.
ANCHOR_PW = "collided = collision_dual_cube(body_a, body_b, &narrowphase_collision);"
# In simulation_physics_loop.c the pattern is slightly different.
ANCHOR_SIM = "collided = collision_dual_cube(rigid_body_a, rigid_body_b, &narrowphase_collision);"

# For simulation_physics_loop.c the variable names differ.
DISPATCH_BLOCK_SIM = DISPATCH_BLOCK.replace("body_a", "rigid_body_a").replace("body_b", "rigid_body_b")

def step_wire_physics_world():
    log("Step 1: Wiring dispatch in physics_world.c")
    p = SRC / "core" / "physics_world.c"
    content = p.read_text()
    if "collision_cylinder_sphere" in content:
        log("  [SKIP] already wired")
        return True
    idx = content.find(ANCHOR_PW)
    if idx < 0:
        log("  [FAIL] cube-cube dispatch anchor not found in physics_world.c")
        return False
    # Find the closing brace of that else-if block
    # The pattern is: } else if (...cube...cube...) {\n    collided = ...;\n    }
    # We insert after the closing }
    # Find the line with the anchor, then find the next }
    line_end = content.find("\n", idx)
    # The closing } is on the next line
    close_idx = content.find("}", line_end)
    if close_idx < 0:
        log("  [FAIL] closing brace not found after cube-cube dispatch")
        return False
    content = content[:close_idx] + "}" + DISPATCH_BLOCK + content[close_idx+1:]
    write(p, content)
    return True

def step_wire_sim_loop():
    log("Step 2: Wiring dispatch in simulation_physics_loop.c")
    p = SRC / "core" / "simulation_physics_loop.c"
    content = p.read_text()
    if "collision_cylinder_sphere" in content:
        log("  [SKIP] already wired")
        return True
    idx = content.find(ANCHOR_SIM)
    if idx < 0:
        log("  [FAIL] cube-cube dispatch anchor not found in simulation_physics_loop.c")
        return False
    line_end = content.find("\n", idx)
    close_idx = content.find("}", line_end)
    if close_idx < 0:
        log("  [FAIL] closing brace not found after cube-cube dispatch")
        return False
    content = content[:close_idx] + "}" + DISPATCH_BLOCK_SIM + content[close_idx+1:]
    write(p, content)
    return True

def step_build():
    log("Step 3: Build check")
    r = subprocess.run(
        [sys.executable, str(TOOLS / "build_check.py"), "--quick"],
        cwd=str(ROOT), capture_output=True, text=True, timeout=180)
    if r.returncode != 0:
        print(r.stdout[-2000:] if r.stdout else "")
        print(r.stderr[-2000:] if r.stderr else "")
        log("[FAIL] build failed")
        return False
    log("[PASS] build clean")
    return True

def step_tests():
    log("Step 4: Run headless tests")
    r = subprocess.run(
        [sys.executable, str(TOOLS / "test_runner.py")],
        cwd=str(ROOT), capture_output=True, text=True, timeout=300)
    print(r.stdout[-2000:] if r.stdout else "")
    if r.returncode != 0:
        log("[WARN] some tests failed")
        return False
    log("[PASS] all tests pass")
    return True

def main():
    print("=" * 60)
    print("MFS 173: Wire cylinder dispatch into both sites")
    print("=" * 60)
    if DRY_RUN: print("  ** DRY RUN **\n")
    steps = [step_wire_physics_world, step_wire_sim_loop]
    for fn in steps:
        try:
            if not fn(): return 1
        except Exception as e:
            print(f"\n[FAIL] {fn.__name__}: {e}")
            import traceback; traceback.print_exc()
            return 1
    if not DRY_RUN:
        if not step_build(): return 1
        if not step_tests(): return 1
    print("=" * 60)
    print("  173 complete. Cylinders now collide with spheres, cubes,")
    print("  and other cylinders in both physics_world and GUI paths.")
    print("=" * 60)
    return 0

if __name__ == "__main__":
    sys.exit(main())
