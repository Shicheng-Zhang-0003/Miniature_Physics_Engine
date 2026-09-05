#!/usr/bin/env python3
"""
MFS 173b: Full repair of cylinder dispatch corruption
=====================================================
The 173 script inserted the cylinder dispatch at the wrong location,
breaking physics_world.c (and possibly simulation_physics_loop.c).
This script:
  1. Removes ALL misplaced cylinder dispatch code
  2. Re-inserts it at the correct location inside the dispatch chain
  3. Builds and tests

Usage:
    cd <project_root>
    python3 fixes/173b.py [--dry-run]
"""
import sys, re, subprocess
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
ROOT = SCRIPT_DIR.parent
SRC = ROOT / "v15R3" / "src"
TOOLS = ROOT / "tools"
sys.path.insert(0, str(TOOLS))

DRY_RUN = "--dry-run" in sys.argv

def log(msg): print(f"  [173b] {msg}")

def write(p, t):
    if not DRY_RUN: p.write_text(t)
    log(f"  [OK] {p.relative_to(SRC)}")

# The cylinder dispatch block to insert (correct version, no leading })
CYL_DISPATCH_PW = """    } else if ((body_a->type == object_cylinder) && (body_b->type == object_sphere)) {
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
    }"""

CYL_DISPATCH_SIM = CYL_DISPATCH_PW.replace("body_a", "rigid_body_a").replace("body_b", "rigid_body_b")

def remove_cylinder_dispatch(content):
    """Remove all cylinder dispatch else-if chains from content."""
    # Pattern: } else if ((body_a->type == object_cylinder) ... through the closing }
    # We need to find the start and end of the cylinder dispatch block

    # Find the first occurrence of the cylinder dispatch start
    patterns = [
        r'\}\s*else if \(\(body_a->type == object_cylinder\)',
        r'\}\s*else if \(\(rigid_body_a->type == object_cylinder\)',
    ]

    for pattern in patterns:
        match = re.search(pattern, content)
        if match:
            start = match.start()
            # Find the end: the closing } of the last else-if in the chain
            # Count braces from the start
            depth = 0
            pos = start
            found_first_brace = False
            while pos < len(content):
                if content[pos] == '{':
                    depth += 1
                    found_first_brace = True
                elif content[pos] == '}':
                    depth -= 1
                    if found_first_brace and depth == 0:
                        # This is the closing } of the last else-if
                        end = pos + 1
                        # Remove from start to end
                        content = content[:start] + content[end:]
                        log(f"  Removed cylinder dispatch block ({end - start} chars)")
                        # Check if there's another one (shouldn't be, but just in case)
                        return remove_cylinder_dispatch(content)
                pos += 1

    return content

def insert_cylinder_dispatch_pw(content):
    """Insert cylinder dispatch into physics_world.c at the correct location."""
    # Find the cube-cube dispatch line
    anchor = "collided = collision_dual_cube(body_a, body_b, &narrowphase_collision);"
    idx = content.find(anchor)
    if idx < 0:
        log("  [FAIL] cube-cube anchor not found in physics_world.c")
        return None

    # Find the closing } of the cube-cube else-if block
    # It's the next } after the anchor line
    line_end = content.find("\n", idx)
    close_idx = content.find("}", line_end)
    if close_idx < 0:
        log("  [FAIL] closing brace not found after cube-cube dispatch")
        return None

    # Insert the cylinder dispatch after the closing }
    # The closing } is the end of the cube-cube else-if
    # We need to insert BEFORE this } and replace it with the cylinder dispatch
    # Actually, we insert AFTER the } but the cylinder dispatch starts with } else if
    # So we replace the } with the cylinder dispatch block

    # The correct approach: the cube-cube block ends with }
    # We replace that } with the cylinder dispatch (which starts with } else if)
    content = content[:close_idx] + CYL_DISPATCH_PW + content[close_idx+1:]
    return content

def insert_cylinder_dispatch_sim(content):
    """Insert cylinder dispatch into simulation_physics_loop.c at the correct location."""
    anchor = "collided = collision_dual_cube(rigid_body_a, rigid_body_b, &narrowphase_collision);"
    idx = content.find(anchor)
    if idx < 0:
        log("  [FAIL] cube-cube anchor not found in simulation_physics_loop.c")
        return None

    line_end = content.find("\n", idx)
    close_idx = content.find("}", line_end)
    if close_idx < 0:
        log("  [FAIL] closing brace not found after cube-cube dispatch")
        return None

    content = content[:close_idx] + CYL_DISPATCH_SIM + content[close_idx+1:]
    return content

def fix_physics_world():
    log("Step 1: Repairing physics_world.c")
    p = SRC / "core" / "physics_world.c"
    content = p.read_text()

    # Check if cylinder dispatch is already correctly placed
    if "collision_cylinder_sphere" in content:
        # Check if it's in the right place (inside the dispatch chain)
        # by checking if it's near the cube-cube dispatch
        cube_idx = content.find("collision_dual_cube(body_a, body_b")
        cyl_idx = content.find("collision_cylinder_sphere(body_a, body_b")
        if cube_idx >= 0 and cyl_idx >= 0 and abs(cyl_idx - cube_idx) < 2000:
            log("  [SKIP] cylinder dispatch already correctly placed")
            return True

    # Remove any misplaced cylinder dispatch
    content = remove_cylinder_dispatch(content)

    # Re-insert at the correct location
    content = insert_cylinder_dispatch_pw(content)
    if content is None:
        return False

    write(p, content)
    return True

def fix_sim_loop():
    log("Step 2: Repairing simulation_physics_loop.c")
    p = SRC / "core" / "simulation_physics_loop.c"
    content = p.read_text()

    if "collision_cylinder_sphere" in content:
        cube_idx = content.find("collision_dual_cube(rigid_body_a, rigid_body_b")
        cyl_idx = content.find("collision_cylinder_sphere(rigid_body_a, rigid_body_b")
        if cube_idx >= 0 and cyl_idx >= 0 and abs(cyl_idx - cube_idx) < 2000:
            log("  [SKIP] cylinder dispatch already correctly placed")
            return True

    content = remove_cylinder_dispatch(content)
    content = insert_cylinder_dispatch_sim(content)
    if content is None:
        return False

    write(p, content)
    return True

def build():
    log("Step 3: Build check")
    r = subprocess.run(
        [sys.executable, str(TOOLS / "build_check.py"), "--quick"],
        cwd=str(ROOT), capture_output=True, text=True, timeout=180)
    print(r.stdout[-3000:] if r.stdout else "")
    if r.returncode != 0:
        print(r.stderr[-2000:] if r.stderr else "")
        log("[FAIL] build still broken")
        return False
    log("[PASS] build clean")
    return True

def tests():
    log("Step 4: Running headless tests")
    r = subprocess.run(
        [sys.executable, str(TOOLS / "test_runner.py")],
        cwd=str(ROOT), capture_output=True, text=True, timeout=300)
    print(r.stdout[-2500:] if r.stdout else "")
    if r.returncode != 0:
        log("[WARN] some tests failed")
        return False
    log("[PASS] all tests pass")
    return True

def main():
    print("=" * 60)
    print("MFS 173b: Full cylinder dispatch repair")
    print("=" * 60)
    if DRY_RUN: print("  ** DRY RUN **\n")

    if not fix_physics_world(): return 1
    if not fix_sim_loop(): return 1

    if not DRY_RUN:
        if not build(): return 1
        tests()

    print("=" * 60)
    print("  173b complete. Cylinder dispatch repaired.")
    print("=" * 60)
    return 0

if __name__ == "__main__":
    sys.exit(main())
