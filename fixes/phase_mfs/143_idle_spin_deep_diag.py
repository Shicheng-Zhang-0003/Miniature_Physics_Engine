#!/usr/bin/env python3
"""
MFS 143: Deep idle-spin diagnostic — asleep vs counteracted vs not-braking
==========================================================================
142 showed wheels hold a constant ~25 rad/s at zero command. This measures,
per wheel at idle:
  - is_sleeping  (are they frozen by the sleep system?)
  - motor command / output_torque / current  (is the motor braking?)
  - axle omega before and after ONE physics step  (does the spin change?)

Verdicts:
  - all wheels asleep                 -> sleep-system bug (linear-only check)
  - awake, out_torque<0, no decel     -> braking counteracted (revolute joint)
  - awake, out_torque~0               -> motor not braking (drivetrain bug)
  - omega drops in-step then restores -> something re-spins each frame

Usage:
    cd <project_root>
    python3 fixes/phase_mfs/143_idle_spin_deep_diag.py
"""
import subprocess
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
ROOT = SCRIPT_DIR.parent.parent
SRC = ROOT / "v15R2" / "src"

def log(msg): print(f"  [143] {msg}")

DIAG = '''
#ifdef MFS_IDLE_DEEP_DIAG
#include <stdio.h>
#include <math.h>
#include "core/physics_world.h"
#include "core/rigidbody.h"
#include "physics/constraint.h"
#include "config/mpe_config.h"
#include "robotics/robot.h"
#include "robotics/drivetrain.h"

static const float DT = 1.0f / 60.0f;

static float axle_omega(physics_world *world, int wi) {
    rigidbody *wheel = &world->bodies[wi];
    vector3 axle = vector4_rotate_to_vector3(wheel->orientation, (vector3){1.0f, 0.0f, 0.0f});
    return vector3_dot(wheel->angular_velocity, axle);
}

int main(void) {
    mpe_config_init();
    printf("\\n=== IDLE SPIN DEEP DIAGNOSTIC ===\\n");

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

    float zero[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    ftc_robot_set_wheel_commands(&robot, zero, 4);

    /* settle */
    for (int i = 0; i < 120; i++) {
        drivetrain_update(&world, &robot, DT);
        physics_world_step(&world, DT);
    }

    printf("after 120-frame settle, per wheel:\\n");
    printf("  %-6s %-7s %-8s %-11s %-10s %-11s\\n",
           "wheel", "asleep", "cmd", "out_torque", "current", "axle_omega");
    for (int w = 0; w < robot.wheel_count; w++) {
        int wi = robot.wheel_bodies[w];
        rigidbody *wheel = &world.bodies[wi];
        printf("  %-6d %-7d %-8.3f %-11.4f %-10.3f %-11.3f\\n",
               w, (int)wheel->is_sleeping,
               robot.wheel_motors[w].command,
               robot.wheel_motors[w].output_torque,
               robot.wheel_motors[w].current,
               axle_omega(&world, wi));
    }

    /* single-step delta */
    float before[4];
    for (int w = 0; w < robot.wheel_count; w++) before[w] = axle_omega(&world, robot.wheel_bodies[w]);
    drivetrain_update(&world, &robot, DT);
    physics_world_step(&world, DT);
    printf("single-step delta (omega_before -> omega_after):\\n");
    for (int w = 0; w < robot.wheel_count; w++) {
        float after = axle_omega(&world, robot.wheel_bodies[w]);
        printf("  wheel[%d]: %.3f -> %.3f  (delta=%+.4f)\\n", w, before[w], after, after - before[w]);
    }

    rigidbody *ch = &world.bodies[robot.chassis_body];
    printf("chassis: asleep=%d lin_speed=%.4f ang_vel_y=%.4f\\n",
           (int)ch->is_sleeping,
           sqrtf(ch->velocity.x * ch->velocity.x + ch->velocity.z * ch->velocity.z),
           ch->angular_velocity.y);

    /* verdict */
    int any_asleep = 0, all_asleep = 1;
    for (int w = 0; w < robot.wheel_count; w++) {
        int s = (int)world.bodies[robot.wheel_bodies[w]].is_sleeping;
        if (s) any_asleep = 1; else all_asleep = 0;
    }
    printf("\\nVERDICT: ");
    if (all_asleep) printf("ALL wheels ASLEEP -> sleep-system bug (linear-only sleep check)\\n");
    else if (any_asleep) printf("SOME wheels asleep -> partial sleep bug\\n");
    else printf("wheels AWAKE -> check single-step delta to see if braking lands\\n");

    physics_world_cleanup(&world);
    return 0;
}
#endif
'''

def main():
    print("=" * 60)
    print("MFS 143: Deep Idle-Spin Diagnostic")
    print("=" * 60)

    diag_path = SRC / "tests" / "idle_spin_deep_diag.c"
    diag_path.write_text(DIAG)
    log("Wrote idle_spin_deep_diag.c")

    makefile = SRC / "makefile"
    mc = makefile.read_text()
    if "test_idle_spin_deep_diag:" not in mc:
        mc += """
# MFS_143: deep idle-spin diagnostic
IDLE_DEEP_SOURCES := tests/idle_spin_deep_diag.c core/physics_world.c core/rigidbody.c \\
physics/collision_mechanics.c physics/broadphase.c physics/constraint.c physics/revolute_joint.c \\
robotics/motor.c robotics/motor_presets.c robotics/battery.c robotics/robot.c robotics/drivetrain.c \\
config/mpe_config.c config/mpe_config_schema.c
test_idle_spin_deep_diag: $(IDLE_DEEP_SOURCES)
\t$(CC) $(CFLAGS) -DMFS_IDLE_DEEP_DIAG $(IDLE_DEEP_SOURCES) -lm -o test_idle_spin_deep_diag
\t./test_idle_spin_deep_diag
"""
        makefile.write_text(mc)
        log("Added makefile target")

    r = subprocess.run(["make", "-C", str(SRC), "test_idle_spin_deep_diag"],
                       capture_output=True, text=True, timeout=180)
    print(r.stdout[-4000:] if r.stdout else "")
    if r.returncode != 0:
        print(r.stderr[-1500:] if r.stderr else "")
        log("[FAIL] build/run failed")
        return 1
    return 0

if __name__ == "__main__":
    sys.exit(main())
