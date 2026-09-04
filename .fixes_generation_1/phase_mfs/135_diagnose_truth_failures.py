#!/usr/bin/env python3
"""
MFS 135: Diagnose the 4 remaining physics truth failures
==========================================================
Prints actual values for Tests 8, 9, 10, 14 so we can see
exactly what the physics is doing wrong.

Usage:
    cd <project_root>
    python3 fixes/phase_mfs/135_diagnose_truth_failures.py
"""
import sys, subprocess
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
ROOT = SCRIPT_DIR.parent.parent
SRC = ROOT / "v15R3" / "src"
TOOLS = ROOT / "tools"

def log(msg): print(f"  [135] {msg}")

DIAG_TEST = '''
#ifdef MPE_PHYSICS_TRUTH_DIAG
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "core/physics_world.h"
#include "physics/constraint.h"
#include "config/mpe_config.h"
#include "robotics/robot.h"
#include "robotics/drivetrain.h"

static const float DT = 1.0f / 60.0f;

int main(void) {
    mpe_config_init();
    printf("\\n=== DIAG: Test 8 — Back-EMF Braking ===\\n");
    {
        physics_world world; physics_world_init(&world); constraint_pool_init();
        ftc_robot robot;
        ftc_robot_create(&world, &robot, 0.0f, ftc_robot_rest_height(), 0.0f, MOTOR_GB_5203_30);
        for (int i = 0; i < 60; i++) {
            drivetrain_tank(&robot, 1.0f, 1.0f);
            drivetrain_update(&world, &robot, DT);
            physics_world_step(&world, DT);
        }
        float rpm_before = robot.wheel_motors[0].rpm;
        float chassis_v_before = world.bodies[robot.chassis_body].velocity.z;
        float zero[4] = {0,0,0,0};
        ftc_robot_set_wheel_commands(&robot, zero, 4);
        for (int i = 0; i < 120; i++) {
            drivetrain_update(&world, &robot, DT);
            physics_world_step(&world, DT);
        }
        float rpm_after = robot.wheel_motors[0].rpm;
        float chassis_v_after = world.bodies[robot.chassis_body].velocity.z;
        printf("  rpm_before=%.2f rpm_after=%.2f (ratio=%.3f)\\n", rpm_before, rpm_after, rpm_after/(rpm_before+0.001f));
        printf("  chassis_v_before=%.4f chassis_v_after=%.4f (ratio=%.3f)\\n", chassis_v_before, chassis_v_after, chassis_v_after/(chassis_v_before+0.001f));
        printf("  motor command=%.4f current=%.4f output_torque=%.4f\\n",
               robot.wheel_motors[0].command, robot.wheel_motors[0].current, robot.wheel_motors[0].output_torque);
        physics_world_cleanup(&world);
    }

    printf("\\n=== DIAG: Test 9 — Static Friction ===\\n");
    {
        physics_world world; physics_world_init(&world);
        physics_world_add_cube(&world, (vector3){0,-0.5f,0}, (vector3){10,0.5f,10}, 0.0f);
        int idx = physics_world_add_cube(&world, (vector3){0,0.5f,0}, (vector3){0.5f,0.5f,0.5f}, 1.0f);
        float mu_s = world.bodies[idx].friction_static;
        float F_below = 0.5f * mu_s * 1.0f * 9.81f;
        printf("  mu_s=%.4f F_below=%.4f threshold=%.4f\\n", mu_s, F_below, mu_s*1.0f*9.81f);
        world.bodies[idx].force_accumulator.x += F_below;
        physics_world_step(&world, DT);
        printf("  vx_after=%.6f (should be ~0)\\n", world.bodies[idx].velocity.x);
        physics_world_cleanup(&world);
    }

    printf("\\n=== DIAG: Test 10 — Kinetic Friction ===\\n");
    {
        physics_world world; physics_world_init(&world);
        physics_world_add_cube(&world, (vector3){0,-0.5f,0}, (vector3){10,0.5f,10}, 0.0f);
        int idx = physics_world_add_cube(&world, (vector3){0,0.5f,0}, (vector3){0.5f,0.5f,0.5f}, 1.0f);
        world.bodies[idx].velocity.x = 2.0f;
        float mu_k = world.bodies[idx].friction_kinetic;
        printf("  mu_k=%.4f expected_decel=%.4f\\n", mu_k, mu_k*9.81f);
        float vx_before = world.bodies[idx].velocity.x;
        for (int i = 0; i < 60; i++) physics_world_step(&world, DT);
        float vx_after = world.bodies[idx].velocity.x;
        float actual_decel = (vx_before - vx_after) / 1.0f;
        printf("  vx_before=%.4f vx_after=%.4f actual_decel=%.4f\\n", vx_before, vx_after, actual_decel);
        physics_world_cleanup(&world);
    }

    printf("\\n=== DIAG: Test 14 — Cylinder Rest Height ===\\n");
    {
        physics_world world; physics_world_init(&world);
        physics_world_add_cube(&world, (vector3){0,-0.5f,0}, (vector3){10,0.5f,10}, 0.0f);
        float r = 0.05f;
        int idx = physics_world_add_cylinder(&world, r, 0.02f, 0.5f, (vector3){0,1.0f,0});
        for (int i = 0; i < 120; i++) physics_world_step(&world, DT);
        float y = world.bodies[idx].position.y;
        printf("  r=%.4f actual_y=%.6f expected_y=%.6f error=%.6f\\n", r, y, r, fabsf(y-r));
        printf("  penetration_slop=%.6f\\n", g_cfg.solver.penetration_slop);
        physics_world_cleanup(&world);
    }

    printf("\\n=== DIAG COMPLETE ===\\n");
    return 0;
}
#endif
'''

def main():
    print("=" * 60)
    print("MFS 135: Diagnose Physics Truth Failures")
    print("=" * 60)

    # Write diag test
    diag_path = SRC / "tests" / "physics_truth_diag.c"
    diag_path.write_text(DIAG_TEST)
    log("Wrote physics_truth_diag.c")

    # Build and run
    makefile = SRC / "makefile"
    mc = makefile.read_text()
    if "test_physics_truth_diag:" not in mc:
        mc += """
# MFS_135: diagnostic
DIAG_SOURCES := tests/physics_truth_diag.c core/physics_world.c core/rigidbody.c \\
physics/collision_mechanics.c physics/broadphase.c physics/constraint.c physics/revolute_joint.c \\
robotics/motor.c robotics/motor_presets.c robotics/battery.c robotics/robot.c robotics/drivetrain.c \\
config/mpe_config.c config/mpe_config_schema.c
test_physics_truth_diag: $(DIAG_SOURCES)
\t$(CC) $(CFLAGS) -DMPE_PHYSICS_TRUTH_DIAG $(DIAG_SOURCES) -lm -o test_physics_truth_diag
\t./test_physics_truth_diag
"""
        makefile.write_text(mc)
        log("Added makefile target")

    result = subprocess.run(
        ["make", "-C", str(SRC), "test_physics_truth_diag"],
        cwd=str(SRC), capture_output=True, text=True, timeout=120)
    print(result.stdout[-4000:] if result.stdout else "")
    if result.returncode != 0:
        print(result.stderr[-1000:] if result.stderr else "")
    return 0

if __name__ == "__main__":
    sys.exit(main())
