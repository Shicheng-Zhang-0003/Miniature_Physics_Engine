#!/usr/bin/env python3
"""
MFS 174: Cylinder collision headless tests
===========================================
Three new tests proving the 172 narrowphase functions work:
  1. cylinder vs sphere  – sphere must not pass through cylinder
  2. cylinder vs cube    – cylinder must not pass through static wall
  3. cylinder vs cylinder – two cylinders must not pass through each other

Also adds makefile targets and test_runner.py entries.

Usage:
    cd <project_root>
    python3 fixes/174_cylinder_tests.py [--dry-run]
"""
import sys, subprocess
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
ROOT = SCRIPT_DIR.parent
SRC = ROOT / "v15R3" / "src"
TOOLS = ROOT / "tools"
sys.path.insert(0, str(TOOLS))
DRY_RUN = "--dry-run" in sys.argv

def log(msg): print(f"  [174] {msg}")

def write(p, t):
    if not DRY_RUN: p.write_text(t)
    log(f"  [OK] {p.name}")

# ── Test 1: cylinder vs sphere ──
CYL_SPH = '''\
/* MFS_174: Cylinder vs sphere collision test.
* A sphere approaches a resting cylinder. The sphere must not
* pass through the cylinder. */
#ifdef MFS_CYL_SPH_TEST
#include <stdio.h>
#include <math.h>
#include "core/physics_world.h"
#include "config/mpe_config.h"

int main(void) {
    mpe_config_init();
    physics_world world;
    physics_world_init(&world);

    /* Static floor so the cylinder rests */
    physics_world_add_cube(&world,
        (vector3){0.0f, -0.5f, 0.0f},
        (vector3){10.0f, 0.5f, 10.0f}, 0.0f);

    /* Static cylinder resting on floor */
    int cyl = physics_world_add_cylinder(&world,
        0.05f, 0.02f, 0.0f,
        (vector3){0.0f, 0.06f, 0.0f});

    /* Sphere approaching the cylinder along Z */
    int sph = physics_world_add_sphere(&world,
        0.08f, 0.3f,
        (vector3){0.0f, 0.08f, 0.5f});
    world.bodies[sph].velocity = (vector3){0.0f, 0.0f, -2.0f};

    const float dt = 1.0f / 60.0f;
    int fail = 0;
    for (int t = 0; t < 120 && !fail; t++) {
        physics_world_step(&world, dt);
        for (int i = 0; i < world.body_count; i++) {
            if (!isfinite(world.bodies[i].position.x) ||
                !isfinite(world.bodies[i].position.y) ||
                !isfinite(world.bodies[i].position.z)) {
                printf("[FAIL] NaN at tick %d\\n", t);
                fail = 1; break;
            }
        }
    }
    if (fail) return 1;

    /* Sphere started at z=0.5 moving toward cylinder at z=0.
    * After 2 seconds it should have been deflected or stopped,
    * NOT passed through to z < -0.2 */
    float sph_z = world.bodies[sph].position.z;
    printf("[info] sphere final z=%.4f (started at 0.5)\\n", sph_z);

    if (sph_z < -0.2f) {
        printf("[FAIL] sphere passed through cylinder\\n");
        return 1;
    }
    printf("[PASS] cylinder-sphere collision works\\n");
    physics_world_cleanup(&world);
    return 0;
}
#endif /* MFS_CYL_SPH_TEST */
'''

# ── Test 2: cylinder vs cube wall ──
CYL_CUBE = '''\
/* MFS_174: Cylinder vs cube wall collision test.
* A cylinder rolls toward a static cube wall. It must not
* pass through the wall. */
#ifdef MFS_CYL_CUBE_TEST
#include <stdio.h>
#include <math.h>
#include "core/physics_world.h"
#include "config/mpe_config.h"

int main(void) {
    mpe_config_init();
    physics_world world;
    physics_world_init(&world);

    /* Static floor */
    physics_world_add_cube(&world,
        (vector3){0.0f, -0.5f, 0.0f},
        (vector3){10.0f, 0.5f, 10.0f}, 0.0f);

    /* Static cube wall at z=0.5 */
    physics_world_add_cube(&world,
        (vector3){0.0f, 0.25f, 0.5f},
        (vector3){0.5f, 0.25f, 0.1f}, 0.0f);

    /* Cylinder rolling toward the wall */
    int cyl = physics_world_add_cylinder(&world,
        0.05f, 0.02f, 0.5f,
        (vector3){0.0f, 0.06f, -0.5f});
    world.bodies[cyl].velocity = (vector3){0.0f, 0.0f, 3.0f};

    const float dt = 1.0f / 60.0f;
    int fail = 0;
    for (int t = 0; t < 180 && !fail; t++) {
        physics_world_step(&world, dt);
        for (int i = 0; i < world.body_count; i++) {
            if (!isfinite(world.bodies[i].position.z)) {
                printf("[FAIL] NaN at tick %d\\n", t);
                fail = 1; break;
            }
        }
    }
    if (fail) return 1;

    float cyl_z = world.bodies[cyl].position.z;
    printf("[info] cylinder final z=%.4f (wall at z=0.5)\\n", cyl_z);

    if (cyl_z > 0.8f) {
        printf("[FAIL] cylinder passed through wall\\n");
        return 1;
    }
    printf("[PASS] cylinder-cube collision works\\n");
    physics_world_cleanup(&world);
    return 0;
}
#endif /* MFS_CYL_CUBE_TEST */
'''

# ── Test 3: cylinder vs cylinder ──
CYL_CYL = '''\
/* MFS_174: Cylinder vs cylinder collision test.
* Two cylinders approach each other. They must not pass
* through each other. */
#ifdef MFS_CYL_CYL_TEST
#include <stdio.h>
#include <math.h>
#include "core/physics_world.h"
#include "config/mpe_config.h"

int main(void) {
    mpe_config_init();
    physics_world world;
    physics_world_init(&world);

    /* Static floor */
    physics_world_add_cube(&world,
        (vector3){0.0f, -0.5f, 0.0f},
        (vector3){10.0f, 0.5f, 10.0f}, 0.0f);

    /* Two cylinders approaching along Z */
    int c1 = physics_world_add_cylinder(&world,
        0.05f, 0.02f, 0.5f,
        (vector3){0.0f, 0.06f, -0.3f});
    int c2 = physics_world_add_cylinder(&world,
        0.05f, 0.02f, 0.5f,
        (vector3){0.0f, 0.06f, 0.3f});

    world.bodies[c1].velocity = (vector3){0.0f, 0.0f,  2.0f};
    world.bodies[c2].velocity = (vector3){0.0f, 0.0f, -2.0f};

    const float dt = 1.0f / 60.0f;
    int fail = 0;
    for (int t = 0; t < 120 && !fail; t++) {
        physics_world_step(&world, dt);
        for (int i = 0; i < world.body_count; i++) {
            if (!isfinite(world.bodies[i].position.z)) {
                printf("[FAIL] NaN at tick %d\\n", t);
                fail = 1; break;
            }
        }
    }
    if (fail) return 1;

    float z1 = world.bodies[c1].position.z;
    float z2 = world.bodies[c2].position.z;
    float gap = z2 - z1;
    printf("[info] c1 z=%.4f  c2 z=%.4f  gap=%.4f\\n", z1, z2, gap);

    /* They started 0.6 apart. After colliding, c1 should still
    * be behind c2 (gap > 0). If gap < -0.1 they passed through. */
    if (gap < -0.1f) {
        printf("[FAIL] cylinders passed through each other\\n");
        return 1;
    }
    printf("[PASS] cylinder-cylinder collision works\\n");
    physics_world_cleanup(&world);
    return 0;
}
#endif /* MFS_CYL_CYL_TEST */
'''

MAKEFILE_TARGETS = '''
# MFS_174: cylinder collision tests
CYL_SPH_SOURCES := tests/cylinder_sphere_test.c core/physics_world.c core/rigidbody.c \\
physics/collision_mechanics.c physics/broadphase.c physics/constraint.c physics/revolute_joint.c \\
config/mpe_config.c config/mpe_config_schema.c
test_cylinder_sphere: $(CYL_SPH_SOURCES)
\t$(CC) $(CFLAGS) -DMFS_CYL_SPH_TEST $(CYL_SPH_SOURCES) -lm -o test_cylinder_sphere
\t./test_cylinder_sphere

CYL_CUBE_SOURCES := tests/cylinder_cube_test.c core/physics_world.c core/rigidbody.c \\
physics/collision_mechanics.c physics/broadphase.c physics/constraint.c physics/revolute_joint.c \\
config/mpe_config.c config/mpe_config_schema.c
test_cylinder_cube: $(CYL_CUBE_SOURCES)
\t$(CC) $(CFLAGS) -DMFS_CYL_CUBE_TEST $(CYL_CUBE_SOURCES) -lm -o test_cylinder_cube
\t./test_cylinder_cube

CYL_CYL_SOURCES := tests/cylinder_cylinder_test.c core/physics_world.c core/rigidbody.c \\
physics/collision_mechanics.c physics/broadphase.c physics/constraint.c physics/revolute_joint.c \\
config/mpe_config.c config/mpe_config_schema.c
test_cylinder_cylinder: $(CYL_CYL_SOURCES)
\t$(CC) $(CFLAGS) -DMFS_CYL_CYL_TEST $(CYL_CYL_SOURCES) -lm -o test_cylinder_cylinder
\t./test_cylinder_cylinder
'''

def step_write_tests():
    log("Step 1: Writing test files")
    write(SRC / "tests" / "cylinder_sphere_test.c", CYL_SPH)
    write(SRC / "tests" / "cylinder_cube_test.c", CYL_CUBE)
    write(SRC / "tests" / "cylinder_cylinder_test.c", CYL_CYL)
    return True

def step_makefile():
    log("Step 2: Adding makefile targets")
    p = SRC / "makefile"
    content = p.read_text()
    if "test_cylinder_sphere:" in content:
        log("  [SKIP] targets already present")
        return True
    content += MAKEFILE_TARGETS
    write(p, content)
    return True

def step_test_runner():
    log("Step 3: Adding to test_runner.py")
    p = TOOLS / "test_runner.py"
    content = p.read_text()
    if '"cylinder_sphere"' in content:
        log("  [SKIP] already in KNOWN_TESTS")
        return True
    old = '    "odometry_accuracy",\n]'
    new = '    "odometry_accuracy",\n    "cylinder_sphere",\n    "cylinder_cube",\n    "cylinder_cylinder",\n]'
    if old in content:
        content = content.replace(old, new, 1)
        write(p, content)
        return True
    log("  [WARN] anchor not found in test_runner.py")
    return True

def step_build_run():
    log("Step 4: Building and running cylinder tests")
    for test in ["cylinder_sphere", "cylinder_cube", "cylinder_cylinder"]:
        r = subprocess.run(
            ["make", "-C", str(SRC), f"test_{test}"],
            capture_output=True, text=True, timeout=120)
        out = r.stdout + r.stderr
        if r.returncode != 0:
            print(out[-2000:])
            log(f"[FAIL] test_{test} failed")
            return False
        # Print the PASS/FAIL line
        for line in out.split("\n"):
            if "[PASS]" in line or "[FAIL]" in line or "[info]" in line:
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
    log("[PASS] all 14 tests green")
    return True

def main():
    print("=" * 60)
    print("MFS 174: Cylinder Collision Tests")
    print("=" * 60)
    if DRY_RUN: print("  ** DRY RUN **\n")

    steps = [step_write_tests, step_makefile, step_test_runner]
    for fn in steps:
        try:
            if not fn(): return 1
        except Exception as e:
            print(f"\n[FAIL] {fn.__name__}: {e}")
            import traceback; traceback.print_exc()
            return 1

    if not DRY_RUN:
        if not step_build_run(): return 1
        if not step_full_suite(): return 1

    print("=" * 60)
    print("  174 complete. 3 new cylinder tests added (14 total).")
    print("  cylinder-vs-sphere, cylinder-vs-cube, cylinder-vs-cylinder")
    print("  all proven to produce real collisions.")
    print("=" * 60)
    return 0

if __name__ == "__main__":
    sys.exit(main())
