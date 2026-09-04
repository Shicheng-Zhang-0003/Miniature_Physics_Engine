#!/usr/bin/env python3
"""
MFS 142: Idle wheel-spin diagnostic
====================================
The reported bug: with no keys pressed (GVBNCH all released), the robot's
wheels still spin in random directions.

This creates a headless robot, lets it settle, then applies ZERO input for
300 frames and watches every wheel's axle angular velocity + chassis drift.
The pattern in the output tells us the cause:
  - all wheels spin up together, growing   -> systematic torque (motor/RR sign)
  - small random jitter, bounded            -> resting-contact solver jitter
  - chassis drifts while wheels turn        -> contact asymmetry
  - wheels stuck at a constant nonzero spin -> stale command / accumulator leak

Usage:
    cd <project_root>
    python3 fixes/phase_mfs/142_idle_spin_diag.py
"""
import subprocess
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
ROOT = SCRIPT_DIR.parent.parent
SRC = ROOT / "v15R2" / "src"

def log(msg): print(f"  [142] {msg}")

DIAG = '''
#ifdef MFS_IDLE_DIAG
#include <stdio.h>
#include <math.h>
#include "core/physics_world.h"
#include "physics/constraint.h"
#include "config/mpe_config.h"
#include "robotics/robot.h"
#include "robotics/drivetrain.h"

static const float DT = 1.0f / 60.0f;

int main(void) {
    mpe_config_init();
    printf("\\n=== IDLE WHEEL-SPIN DIAGNOSTIC ===\\n");

    physics_world world;
    physics_world_init(&world);
    constraint_pool_init();

    /* floor at y=0 */
    physics_world_add_cube(&world,
        (vector3){0.0f, -0.5f, 0.0f},
        (vector3){10.0f, 0.5f, 10.0f}, 0.0f);

    ftc_robot robot;
    int rc = ftc_robot_create(&world, &robot, 0.0f, ftc_robot_rest_height(), 0.0f, MOTOR_GB_5203_30);
    if (rc != 0) { printf("[FAIL] robot create\\n"); return 1; }

    /* Ensure zero commands */
    float zero[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    ftc_robot_set_wheel_commands(&robot, zero, 4);

    /* Settle 120 frames with zero input */
    for (int i = 0; i < 120; i++) {
        drivetrain_update(&world, &robot, DT);
        physics_world_step(&world, DT);
    }
    printf("settled. now monitoring 300 frames of ZERO input:\\n");

    float max_wheel_omega = 0.0f;
    float max_chassis_speed = 0.0f;
    for (int i = 0; i < 300; i++) {
        drivetrain_update(&world, &robot, DT);
        physics_world_step(&world, DT);

        if (i % 30 == 0 || i == 299) {
            printf("  t=%3d cmd=[%.2f %.2f %.2f %.2f] wheel_omega_axle=[",
                   i, robot.wheel_motors[0].command, robot.wheel_motors[1].command,
                   robot.wheel_motors[2].command, robot.wheel_motors[3].command);
            for (int w = 0; w < robot.wheel_count; w++) {
                int wi = robot.wheel_bodies[w];
                rigidbody *wheel = &world.bodies[wi];
                vector3 axle = vector4_rotate_to_vector3(wheel->orientation, (vector3){1.0f, 0.0f, 0.0f});
                float oa = vector3_dot(wheel->angular_velocity, axle);
                if (fabsf(oa) > max_wheel_omega) max_wheel_omega = fabsf(oa);
                printf("%s%.3f", w ? ", " : "", oa);
            }
            rigidbody *ch = &world.bodies[robot.chassis_body];
            float cs = sqrtf(ch->velocity.x * ch->velocity.x + ch->velocity.z * ch->velocity.z);
            if (cs > max_chassis_speed) max_chassis_speed = cs;
            printf("] chassis_speed=%.4f pos=(%.3f,%.3f)\\n", cs, ch->position.x, ch->position.z);
        }
    }

    printf("\\nmax |wheel axle omega| over idle = %.4f rad/s\\n", max_wheel_omega);
    printf("max chassis speed over idle     = %.4f m/s\\n", max_chassis_speed);
    if (max_wheel_omega > 0.5f) {
        printf("VERDICT: wheels SPIN at idle -> real bug, see pattern above\\n");
    } else if (max_wheel_omega > 0.05f) {
        printf("VERDICT: mild jitter at idle (contact solver)\\n");
    } else {
        printf("VERDICT: idle is stable in headless -> bug may be GUI-side\\n");
    }

    physics_world_cleanup(&world);
    return 0;
}
#endif
'''

def main():
    print("=" * 60)
    print("MFS 142: Idle Wheel-Spin Diagnostic")
    print("=" * 60)

    diag_path = SRC / "tests" / "idle_spin_diag.c"
    diag_path.write_text(DIAG)
    log("Wrote idle_spin_diag.c")

    makefile = SRC / "makefile"
    mc = makefile.read_text()
    if "test_idle_spin_diag:" not in mc:
        mc += """
# MFS_142: idle wheel-spin diagnostic
IDLE_DIAG_SOURCES := tests/idle_spin_diag.c core/physics_world.c core/rigidbody.c \\
physics/collision_mechanics.c physics/broadphase.c physics/constraint.c physics/revolute_joint.c \\
robotics/motor.c robotics/motor_presets.c robotics/battery.c robotics/robot.c robotics/drivetrain.c \\
config/mpe_config.c config/mpe_config_schema.c
test_idle_spin_diag: $(IDLE_DIAG_SOURCES)
\t$(CC) $(CFLAGS) -DMFS_IDLE_DIAG $(IDLE_DIAG_SOURCES) -lm -o test_idle_spin_diag
\t./test_idle_spin_diag
"""
        makefile.write_text(mc)
        log("Added makefile target")

    r = subprocess.run(["make", "-C", str(SRC), "test_idle_spin_diag"],
                       capture_output=True, text=True, timeout=180)
    print(r.stdout[-4000:] if r.stdout else "")
    if r.returncode != 0:
        print(r.stderr[-1500:] if r.stderr else "")
        log("[FAIL] build/run failed")
        return 1
    return 0

if __name__ == "__main__":
    sys.exit(main())
