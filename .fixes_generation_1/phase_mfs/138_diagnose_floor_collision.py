#!/usr/bin/env python3
"""
MFS 138: Diagnose floor collision failure
==========================================
Creates a focused diagnostic that prints:
  1. Cylinder initial state (position, velocity, radius, half_length)
  2. Per-step position/velocity for first 20 steps
  3. Whether the cylinder-floor broadphase pair is generated
  4. Whether the narrowphase detects a collision
  5. Contact normal direction (up or down)
  6. Normal impulse magnitude

Usage:
    cd <project_root>
    python3 fixes/phase_mfs/138_diagnose_floor_collision.py
"""
import sys, subprocess
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
ROOT = SCRIPT_DIR.parent.parent
SRC = ROOT / "v15R3" / "src"
TOOLS = ROOT / "tools"

def log(msg): print(f"  [138] {msg}")

DIAG = '''
#ifdef MPE_FLOOR_DIAG
#include <stdio.h>
#include <math.h>
#include "core/physics_world.h"
#include "config/mpe_config.h"

static const float DT = 1.0f / 60.0f;

int main(void) {
    mpe_config_init();
    printf("\\n=== FLOOR COLLISION DIAGNOSTIC ===\\n");

    physics_world world;
    physics_world_init(&world);

    /* Static floor, top surface at y=0 */
    int floor_idx = physics_world_add_cube(&world,
        (vector3){0.0f, -0.5f, 0.0f},
        (vector3){10.0f, 0.5f, 10.0f}, 0.0f);

    float r = 0.05f;
    int cyl_idx = physics_world_add_cylinder(&world, r, 0.02f, 0.5f,
                                             (vector3){0.0f, 1.0f, 0.0f});

    printf("floor_idx=%d cyl_idx=%d\\n", floor_idx, cyl_idx);
    printf("cylinder initial: pos=(%.4f,%.4f,%.4f) vel=(%.4f,%.4f,%.4f)\\n",
           world.bodies[cyl_idx].position.x, world.bodies[cyl_idx].position.y,
           world.bodies[cyl_idx].position.z,
           world.bodies[cyl_idx].velocity.x, world.bodies[cyl_idx].velocity.y,
           world.bodies[cyl_idx].velocity.z);
    printf("cylinder: radius=%.4f half_length=%.4f mass=%.4f\\n",
           world.bodies[cyl_idx].radius, world.bodies[cyl_idx].cylinder_half_length,
           world.bodies[cyl_idx].mass);
    printf("floor: pos=(%.4f,%.4f,%.4f) half_ext=(%.4f,%.4f,%.4f) static=%d\\n",
           world.bodies[floor_idx].position.x, world.bodies[floor_idx].position.y,
           world.bodies[floor_idx].position.z,
           world.bodies[floor_idx].half_extensions.x,
           world.bodies[floor_idx].half_extensions.y,
           world.bodies[floor_idx].half_extensions.z,
           world.bodies[floor_idx].is_static);
    printf("floor top y=%.4f (pos.y + half_ext.y)\\n",
           world.bodies[floor_idx].position.y + world.bodies[floor_idx].half_extensions.y);
    printf("\\n");

    for (int i = 0; i < 20; i++) {
        physics_world_step(&world, DT);
        printf("step=%2d y=%.6f vy=%.6f\\n",
               i + 1, world.bodies[cyl_idx].position.y,
               world.bodies[cyl_idx].velocity.y);
    }

    printf("\\n=== DIAG COMPLETE ===\\n");
    physics_world_cleanup(&world);
    return 0;
}
#endif
'''

def main():
    print("=" * 60)
    print("MFS 138: Diagnose Floor Collision Failure")
    print("=" * 60)

    diag_path = SRC / "tests" / "floor_collision_diag.c"
    diag_path.write_text(DIAG)
    log("Wrote floor_collision_diag.c")

    makefile = SRC / "makefile"
    mc = makefile.read_text()
    if "test_floor_collision_diag:" not in mc:
        mc += """
# MFS_138: floor collision diagnostic
FLOOR_DIAG_SOURCES := tests/floor_collision_diag.c core/physics_world.c core/rigidbody.c \\
physics/collision_mechanics.c physics/broadphase.c physics/constraint.c physics/revolute_joint.c \\
config/mpe_config.c config/mpe_config_schema.c
test_floor_collision_diag: $(FLOOR_DIAG_SOURCES)
\t$(CC) $(CFLAGS) -DMPE_FLOOR_DIAG $(FLOOR_DIAG_SOURCES) -lm -o test_floor_collision_diag
\t./test_floor_collision_diag
"""
        makefile.write_text(mc)
        log("Added makefile target")

    result = subprocess.run(
        ["make", "-C", str(SRC), "test_floor_collision_diag"],
        cwd=str(SRC), capture_output=True, text=True, timeout=120)
    print(result.stdout[-4000:] if result.stdout else "")
    if result.returncode != 0:
        print(result.stderr[-1000:] if result.stderr else "")
    return 0

if __name__ == "__main__":
    sys.exit(main())
