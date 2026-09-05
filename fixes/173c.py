#!/usr/bin/env python3
"""
MFS 173c: Repair cylinder dispatch insertion in physics_world.c
================================================================
The 173 script inserted the cylinder else-if chain AFTER the closing }
of the cube-cube block, breaking the if-else chain. This removes the
stray } and reconnects the cylinder dispatch into the chain with
correct indentation.
"""
import sys, subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SRC = ROOT / "v15R3" / "src"
TOOLS = ROOT / "tools"
DRY_RUN = "--dry-run" in sys.argv

def log(msg): print(f"  [173c] {msg}")

PW = SRC / "core" / "physics_world.c"

def repair():
    content = PW.read_text()

    if "MFS_173C_REPAIRED" in content:
        log("[SKIP] already repaired")
        return True

    # The broken pattern: cube-cube block ends with }, then } else if follows
    broken = (
        "            collided = collision_dual_cube(body_a, body_b, &narrowphase_collision);\n"
        "        }\n"
        "    } else if ((body_a->type == object_cylinder) && (body_b->type == object_sphere)) {\n"
        "        collided = collision_cylinder_sphere(body_a, body_b, &narrowphase_collision);\n"
        "    } else if ((body_a->type == object_sphere) && (body_b->type == object_cylinder)) {\n"
        "        collided = collision_cylinder_sphere(body_b, body_a, &narrowphase_collision);\n"
        "        if (collided) {\n"
        "            narrowphase_collision.normal_vector = vector3_scaling(narrowphase_collision.normal_vector, -1.0f);\n"
        "            narrowphase_collision.object_a = body_a;\n"
        "            narrowphase_collision.object_b = body_b;\n"
        "        }\n"
        "    } else if ((body_a->type == object_cylinder) && (body_b->type == object_cube)) {\n"
        "        collided = collision_cylinder_cube(body_a, body_b, &narrowphase_collision);\n"
        "    } else if ((body_a->type == object_cube) && (body_b->type == object_cylinder)) {\n"
        "        collided = collision_cylinder_cube(body_b, body_a, &narrowphase_collision);\n"
        "        if (collided) {\n"
        "            narrowphase_collision.normal_vector = vector3_scaling(narrowphase_collision.normal_vector, -1.0f);\n"
        "            narrowphase_collision.object_a = body_a;\n"
        "            narrowphase_collision.object_b = body_b;\n"
        "        }\n"
        "    } else if ((body_a->type == object_cylinder) && (body_b->type == object_cylinder)) {\n"
        "        collided = collision_cylinder_cylinder(body_a, body_b, &narrowphase_collision);\n"
        "    }\n"
        "        if ((collided)"
    )

    fixed = (
        "            collided = collision_dual_cube(body_a, body_b, &narrowphase_collision);\n"
        "        } else if ((body_a->type == object_cylinder) && (body_b->type == object_sphere)) { /* MFS_173C_REPAIRED */\n"
        "            collided = collision_cylinder_sphere(body_a, body_b, &narrowphase_collision);\n"
        "        } else if ((body_a->type == object_sphere) && (body_b->type == object_cylinder)) {\n"
        "            collided = collision_cylinder_sphere(body_b, body_a, &narrowphase_collision);\n"
        "            if (collided) {\n"
        "                narrowphase_collision.normal_vector = vector3_scaling(narrowphase_collision.normal_vector, -1.0f);\n"
        "                narrowphase_collision.object_a = body_a;\n"
        "                narrowphase_collision.object_b = body_b;\n"
        "            }\n"
        "        } else if ((body_a->type == object_cylinder) && (body_b->type == object_cube)) {\n"
        "            collided = collision_cylinder_cube(body_a, body_b, &narrowphase_collision);\n"
        "        } else if ((body_a->type == object_cube) && (body_b->type == object_cylinder)) {\n"
        "            collided = collision_cylinder_cube(body_b, body_a, &narrowphase_collision);\n"
        "            if (collided) {\n"
        "                narrowphase_collision.normal_vector = vector3_scaling(narrowphase_collision.normal_vector, -1.0f);\n"
        "                narrowphase_collision.object_a = body_a;\n"
        "                narrowphase_collision.object_b = body_b;\n"
        "            }\n"
        "        } else if ((body_a->type == object_cylinder) && (body_b->type == object_cylinder)) {\n"
        "            collided = collision_cylinder_cylinder(body_a, body_b, &narrowphase_collision);\n"
        "        }\n"
        "        if ((collided)"
    )

    if broken in content:
        content = content.replace(broken, fixed, 1)
        if not DRY_RUN:
            PW.write_text(content)
        log("[OK] cylinder dispatch reconnected into if-else chain")
        return True

    log("[WARN] exact broken pattern not found, trying line-level fix")

    # Fallback: fix the stray } between cube-cube and cylinder dispatch
    lines = content.split('\n')
    new_lines = []
    i = 0
    fixed_any = False
    while i < len(lines):
        line = lines[i]
        # Find the stray closing brace before cylinder dispatch
        if (line.strip() == '}' and
            i + 1 < len(lines) and
            '} else if ((body_a->type == object_cylinder)' in lines[i + 1]):
            # Skip this stray }, the next line's "} else if" will connect
            # But we need to fix the indentation of the next line
            next_line = lines[i + 1]
            # Replace "    } else if" with "        } else if"
            fixed_line = next_line.replace('    } else if', '        } else if', 1)
            new_lines.append(fixed_line)
            i += 2
            fixed_any = True
            continue
        # Fix indentation of cylinder dispatch lines
        if line.startswith('    } else if ((body_a->type == object_cylinder)'):
            new_lines.append('        ' + line.lstrip())
            fixed_any = True
        elif line.startswith('        collided = collision_cylinder_'):
            new_lines.append('            ' + line.lstrip())
            fixed_any = True
        elif line.startswith('        if (collided) {') and i > 0 and 'cylinder' in lines[i-1]:
            new_lines.append('            ' + line.lstrip())
            fixed_any = True
        else:
            new_lines.append(line)
        i += 1

    if fixed_any:
        content = '\n'.join(new_lines)
        if not DRY_RUN:
            PW.write_text(content)
        log("[OK] line-level indentation fix applied")
        return True

    log("[FAIL] could not find broken pattern")
    return False


def build():
    log("Build check...")
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
    log("Running tests...")
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
    print("MFS 173c: Repair cylinder dispatch in physics_world.c")
    print("=" * 60)
    if DRY_RUN: print("  ** DRY RUN **\n")

    if not repair():
        return 1

    if not DRY_RUN:
        if not build():
            return 1
        tests()

    print("=" * 60)
    print("  173c complete.")
    print("=" * 60)
    return 0

if __name__ == "__main__":
    sys.exit(main())
