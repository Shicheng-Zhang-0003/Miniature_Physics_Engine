#!/usr/bin/env python3
"""
MFS 148: Idle root-cause diagnostic — mecanum roller angles + spin pattern
===========================================================================
READ-ONLY. Prints each wheel's roller_angle_rad and is_mecanum flag, plus
the idle axle-omega pattern, so we can identify WHY the contact solver
spins stationary wheels. Prime suspect: roller-angle configuration.

Usage:
    cd <project_root>
    python3 fixes/phase_mfs/148_idle_root_cause_diag.py
"""
import subprocess
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
ROOT = SCRIPT_DIR.parent.parent
SRC = ROOT / "v15R3" / "src"

def log(msg): print(f"  [148] {msg}")

DIAG = '''
#ifdef MFS_IDLE_ROOTCAUSE_DIAG
#include <stdio.h>
#include <math.h>
#include "core/physics_world.h"
#include "core/rigidbody.h"
#include "physics/constraint.h"
#include "config/mpe_config.h"
#include "robotics/robot.h"
#include "robotics/drivetrain.h"

static const float DT = 1.0f / 60.0f;

int main(void) {
    mpe_config_init();
    printf("\\n=== IDLE ROOT-CAUSE DIAGNOSTIC ===\\n");

    physics_world world;
    physics_world_init(&world);
    constraint_pool_init();

    physics_world_add_cube(&world,
        (vector3){0.0f, -0.5f, 0.0f},
        (vector3){10.0f, 0.5f, 10.0f}, 0.0f);

    ftc_robot robot;
    if (ftc_robot_create(&world, &robot, 0.0f, ftc_robot_rest_height(), 0.0f, MOTOR_GB_5203_30) != 0) {
        printf("[FAIL] robot create\\n"); return 1;
    }

    const char *names[4] = {"FL", "FR", "BL", "BR"};
    printf("wheel configuration:\\n");
    for (int w = 0; w < robot.wheel_count; w++) {
        int wi = robot.wheel_bodies[w];
        rigidbody *wheel = &world.bodies[wi];
        printf("  [%d]=%s  roller_angle=%8.2f deg  is_mecanum=%d  radius=%.4f\\n",
               w, names[w],
               wheel->roller_angle_rad * 57.2957795f,
               wheel->is_mecanum ? 1 : 0,
               wheel->radius);
    }

    float zero[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    ftc_robot_set_wheel_commands(&robot, zero, 4);

    for (int i = 0; i < 120; i++) {
        drivetrain_update(&world, &robot, DT);
        physics_world_step(&world, DT);
    }

    printf("idle (zero input) after settle:\\n");
    for (int w = 0; w < robot.wheel_count; w++) {
        int wi = robot.wheel_bodies[w];
        rigidbody *wheel = &world.bodies[wi];
        vector3 axle = wheel->cached_axes[0];
        if (vector3_length_squared(axle) < 0.0001f) {
            axle = vector4_rotate_to_vector3(wheel->orientation, (vector3){1.0f, 0.0f, 0.0f});
        }
        float oa = vector3_dot(wheel->angular_velocity, axle);
        printf("  [%d]=%s  axle_omega=%8.4f rad/s\\n", w, names[w], oa);
    }

    rigidbody *ch = &world.bodies[robot.chassis_body];
    printf("chassis vel=(%.5f, %.5f, %.5f) ang_vel_y=%.5f\\n",
           ch->velocity.x, ch->velocity.y, ch->velocity.z, ch->angular_velocity.y);

    physics_world_cleanup(&world);
    return 0;
}
#endif
'''

def main():
    print("=" * 60)
    print("MFS 148: Idle Root-Cause Diagnostic (read-only)")
    print("=" * 60)

    diag_path = SRC / "tests" / "idle_rootcause_diag.c"
    diag_path.write_text(DIAG)
    log("Wrote idle_rootcause_diag.c")

    makefile = SRC / "makefile"
    mc = makefile.read_text()
    if "test_idle_rootcause_diag:" not in mc:
        mc += """
# MFS_148: idle root-cause diagnostic
IDLE_RC_SOURCES := tests/idle_rootcause_diag.c core/physics_world.c core/rigidbody.c \\
physics/collision_mechanics.c physics/broadphase.c physics/constraint.c physics/revolute_joint.c \\
robotics/motor.c robotics/motor_presets.c robotics/battery.c robotics/robot.c robotics/drivetrain.c \\
config/mpe_config.c config/mpe_config_schema.c
test_idle_rootcause_diag: $(IDLE_RC_SOURCES)
\t$(CC) $(CFLAGS) -DMFS_IDLE_ROOTCAUSE_DIAG $(IDLE_RC_SOURCES) -lm -o test_idle_rootcause_diag
\t./test_idle_rootcause_diag
"""
        makefile.write_text(mc)
        log("Added makefile target")

    r = subprocess.run(["make", "-C", str(SRC), "test_idle_rootcause_diag"],
                       capture_output=True, text=True, timeout=180)
    print(r.stdout[-3000:] if r.stdout else "")
    if r.returncode != 0:
        print(r.stderr[-1500:] if r.stderr else "")
        log("[FAIL] build/run failed")
        return 1
    return 0

if __name__ == "__main__":
    sys.exit(main())
