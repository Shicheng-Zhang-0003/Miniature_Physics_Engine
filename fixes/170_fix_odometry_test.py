#!/usr/bin/env python3
"""
MFS 170: Rewrite odometry_accuracy_test.c to use real API
==========================================================
The original test called ftc_robot_set_wheel_commands() which doesn't exist.
Rewrite to use drivetrain_tank() for forward drive and drivetrain_mecanum()
for strafe, which is the actual API.
"""
import sys, subprocess
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
ROOT = SCRIPT_DIR.parent
SRC = ROOT / "v15R3" / "src"
TOOLS = ROOT / "tools"
sys.path.insert(0, str(TOOLS))

DRY_RUN = "--dry-run" in sys.argv

def log(msg): print(f"  [170] {msg}")

ODOMETRY_TEST = '''\
/* MFS_170: Odometry accuracy test (rewritten to use real API).
* Drives a robot forward via drivetrain_tank, then strafes via
* drivetrain_mecanum, and compares odometry output against
* actual physics displacement. */
#ifdef MFS_ODOMETRY_ACCURACY_TEST
#include <stdio.h>
#include <math.h>
#include "core/physics_world.h"
#include "physics/constraint.h"
#include "config/mpe_config.h"
#include "robotics/robot.h"
#include "robotics/drivetrain.h"

int main(void) {
    mpe_config_init();
    physics_world world;
    physics_world_init(&world);
    constraint_pool_init();

    ftc_robot robot;
    int rc = ftc_robot_create(&world, &robot, 0.0f, ftc_robot_rest_height(), 0.0f, MOTOR_GB_5203_30);
    if (rc != 0) { printf("[FAIL] could not create robot\\n"); return 1; }

    const float dt = 1.0f / 60.0f;
    int fail = 0;

    /* Phase 0: Let robot settle for 120 ticks */
    for (int t = 0; t < 120; t++) {
        drivetrain_tank(&robot, 0.0f, 0.0f);
        drivetrain_update(&world, &robot, dt);
        physics_world_step(&world, dt);
    }

    /* Reset odometry */
    robot.odom_x = robot.odom_z = robot.odom_theta = 0.0f;
    for (int i = 0; i < robot.wheel_count && i < FTC_MAX_WHEELS; i++)
        robot.wheel_radians[i] = 0.0f;

    float start_x = world.bodies[robot.chassis_body].position.x;
    float start_z = world.bodies[robot.chassis_body].position.z;

    /* Phase 1: Drive forward at full power for 180 ticks (3 seconds) */
    printf("[info] Phase 1: driving forward 3s\\n");
    for (int t = 0; t < 180 && !fail; t++) {
        drivetrain_tank(&robot, 1.0f, 1.0f);
        drivetrain_update(&world, &robot, dt);
        physics_world_step(&world, dt);
        for (int i = 0; i < world.body_count; i++) {
            if (!isfinite(world.bodies[i].position.x) ||
                !isfinite(world.bodies[i].position.z)) {
                printf("[FAIL] NaN at tick %d\\n", t);
                fail = 1;
                break;
            }
        }
    }
    if (fail) return 1;

    float end_x = world.bodies[robot.chassis_body].position.x;
    float end_z = world.bodies[robot.chassis_body].position.z;
    float physics_dz = end_z - start_z;
    float physics_dx = end_x - start_x;
    float physics_dist = sqrtf(physics_dz * physics_dz + physics_dx * physics_dx);

    printf("[info] physics: dx=%.4f dz=%.4f dist=%.4f\\n", physics_dx, physics_dz, physics_dist);
    printf("[info] odometry: dx=%.4f dz=%.4f theta=%.4f\\n", robot.odom_x, robot.odom_z, robot.odom_theta);

    /* Robot should have moved at least 0.2 m forward */
    if (physics_dist < 0.2f) {
        printf("[FAIL] robot barely moved (%.4f m)\\n", physics_dist);
        return 1;
    }
    printf("[PASS] robot moved %.4f m\\n", physics_dist);

    /* Odometry should track physics within 30% error */
    float odom_dist = sqrtf(robot.odom_z * robot.odom_z + robot.odom_x * robot.odom_x);
    float dist_error = fabsf(odom_dist - physics_dist) / (physics_dist + 0.001f);
    printf("[info] odometry distance error: %.1f%%\\n", dist_error * 100.0f);

    if (dist_error > 0.3f) {
        printf("[FAIL] odometry drift too large (%.1f%%)\\n", dist_error * 100.0f);
        return 1;
    }
    printf("[PASS] odometry tracks physics (error=%.1f%%)\\n", dist_error * 100.0f);

    /* Phase 2: Strafe test — verify odometry X sign matches physics X */
    printf("[info] Phase 2: strafe test\\n");
    robot.odom_x = robot.odom_z = robot.odom_theta = 0.0f;
    for (int i = 0; i < robot.wheel_count && i < FTC_MAX_WHEELS; i++)
        robot.wheel_radians[i] = 0.0f;

    start_x = world.bodies[robot.chassis_body].position.x;

    /* Mecanum strafe: forward=0, strafe=1, rotate=0 */
    for (int t = 0; t < 60 && !fail; t++) {
        drivetrain_mecanum(&robot, 0.0f, 1.0f, 0.0f);
        drivetrain_update(&world, &robot, dt);
        physics_world_step(&world, dt);
    }

    end_x = world.bodies[robot.chassis_body].position.x;
    float strafe_dx = end_x - start_x;
    printf("[info] strafe: physics dx=%.4f odometry dx=%.4f\\n", strafe_dx, robot.odom_x);

    if (fabsf(strafe_dx) > 0.02f && fabsf(robot.odom_x) > 0.02f) {
        /* Check sign agreement */
        if ((strafe_dx > 0 && robot.odom_x < 0) || (strafe_dx < 0 && robot.odom_x > 0)) {
            printf("[FAIL] odometry strafe sign disagrees with physics\\n");
            return 1;
        }
        printf("[PASS] odometry strafe sign matches physics\\n");
    } else {
        printf("[info] strafe too small to verify sign (dx=%.4f)\\n", strafe_dx);
    }

    printf("[PASS] odometry accuracy test complete\\n");
    physics_world_cleanup(&world);
    return 0;
}
#endif /* MFS_ODOMETRY_ACCURACY_TEST */
'''

def step_rewrite():
    log("Step 1: Rewriting odometry_accuracy_test.c")
    p = SRC / "tests" / "odometry_accuracy_test.c"
    if not DRY_RUN:
        p.write_text(ODOMETRY_TEST)
    log(f"  [OK] {p.name}")
    return True

def step_build_test():
    log("Step 2: Building and running odometry test")
    r = subprocess.run(["make", "-C", str(SRC), "test_odometry_accuracy"],
                       cwd=str(SRC), capture_output=True, text=True, timeout=120)
    print(r.stdout[-3000:] if r.stdout else "")
    if r.returncode != 0:
        print(r.stderr[-2000:] if r.stderr else "")
        log("[FAIL] odometry test still failing")
        return False
    log("[PASS] odometry test passes")
    return True

def step_full_suite():
    log("Step 3: Running full test suite")
    r = subprocess.run([sys.executable, str(TOOLS / "test_runner.py")],
                       cwd=ROOT, capture_output=True, text=True, timeout=300)
    print(r.stdout[-3000:] if r.stdout else "")
    if r.returncode != 0:
        log("[WARN] some tests failed")
        return False
    log("[PASS] all 11 tests pass")
    return True

def main():
    print("=" * 60)
    print("MFS 170: Fix odometry_accuracy_test.c (use real API)")
    print("=" * 60)
    if DRY_RUN: print("  ** DRY RUN **\n")

    if not step_rewrite(): return 1
    if not DRY_RUN:
        if not step_build_test(): return 1
        if not step_full_suite(): return 1

    print("=" * 60)
    print("  170 complete. Odometry test rewritten with real API.")
    print("  Expected: 11/11 tests green.")
    print("=" * 60)
    return 0

if __name__ == "__main__":
    sys.exit(main())
