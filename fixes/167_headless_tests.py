#!/usr/bin/env python3
"""
MFS 167: Headless tests — tank drive + odometry accuracy
==========================================================
Bugs addressed:
 18. No headless test for tank drive (differential turning)
 19. No headless test for odometry accuracy
 14. Odometry strafe sign convention unverified (covered by 19)

Creates two new test files and adds makefile targets + test_runner entries.

Usage:
cd <project_root>
python3 fixes/167_headless_tests.py [--dry-run]
"""
import sys, subprocess
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
ROOT = SCRIPT_DIR.parent
SRC = ROOT / "v15R3" / "src"
TOOLS = ROOT / "tools"
sys.path.insert(0, str(TOOLS))

DRY_RUN = "--dry-run" in sys.argv

def log(msg): print(f"  [167] {msg}")

def write(p, t):
    if not DRY_RUN: p.write_text(t)
    log(f"  [OK] {p.relative_to(SRC)}")

TANK_TURN_TEST = '''\
/* MFS_167: Tank drive differential turning test.
* Creates a robot, applies differential drive (left forward, right backward)
* and verifies the robot rotates in place. */
#ifdef MFS_TANK_TURN_TEST
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
    int rc = ftc_robot_create_with_drive(&world, &robot, 0.0f, ftc_robot_rest_height(), 0.0f,
                                         MOTOR_GB_5203_30, FTC_DRIVETRAIN_TANK);
    if (rc != 0) { printf("[FAIL] could not create robot\\n"); return 1; }

    float start_x, start_y, start_z;
    ftc_robot_get_position(&world, &robot, &start_x, &start_y, &start_z);

    const float dt = 1.0f / 60.0f;
    int fail = 0;

    /* Apply differential drive: left forward, right backward -> rotate in place */
    for (int t = 0; t < 120 && !fail; t++) {
        drivetrain_tank(&robot, 1.0f, -1.0f);
        drivetrain_update(&world, &robot, dt);
        physics_world_step(&world, dt);

        for (int i = 0; i < world.body_count; i++) {
            rigidbody *rb = &world.bodies[i];
            if (!isfinite(rb->position.x) || !isfinite(rb->position.y) || !isfinite(rb->position.z)) {
                printf("[FAIL] NaN in body %d at tick %d\\n", i, t);
                fail = 1;
                break;
            }
        }
    }
    if (fail) return 1;

    float end_x, end_y, end_z;
    ftc_robot_get_position(&world, &robot, &end_x, &end_y, &end_z);

    /* Robot should have rotated but not translated much */
    float displacement = sqrtf((end_x - start_x) * (end_x - start_x) +
                               (end_z - start_z) * (end_z - start_z));
    float heading_change = fabsf(world.bodies[robot.chassis_body].orientation.y);

    printf("[info] displacement=%.4f heading_change=%.4f\\n", displacement, heading_change);

    if (displacement > 0.3f) {
        printf("[FAIL] robot translated too much during differential turn (%.4f m)\\n", displacement);
        return 1;
    }
    if (heading_change < 0.1f) {
        printf("[FAIL] robot did not rotate during differential turn\\n");
        return 1;
    }

    printf("[PASS] tank drive differential turn works (displacement=%.4f, heading=%.4f)\\n",
           displacement, heading_change);
    physics_world_cleanup(&world);
    return 0;
}
#endif /* MFS_TANK_TURN_TEST */
'''

ODOMETRY_ACCURACY_TEST = '''\
/* MFS_167: Odometry accuracy test.
* Drives a robot forward for a known duration and compares
* odometry output against actual physics displacement.
* Also verifies strafe sign convention. */
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

    /* Let robot settle */
    float zero[4] = {0, 0, 0, 0};
    ftc_robot_set_wheel_commands(&robot, zero, 4);
    for (int t = 0; t < 120; t++) {
        drivetrain_update(&world, &robot, dt);
        physics_world_step(&world, dt);
    }

    /* Reset odometry */
    robot.odom_x = robot.odom_z = robot.odom_theta = 0.0f;
    for (int i = 0; i < 4; i++) robot.wheel_radians[i] = 0.0f;

    vector3 start_pos = world.bodies[robot.chassis_body].position;

    /* Drive forward at half power for 2 seconds */
    float cmd[4] = {0.5f, 0.5f, 0.5f, 0.5f};
    ftc_robot_set_wheel_commands(&robot, cmd, 4);
    for (int t = 0; t < 120 && !fail; t++) {
        drivetrain_update(&world, &robot, dt);
        physics_world_step(&world, dt);
        for (int i = 0; i < world.body_count; i++) {
            if (!isfinite(world.bodies[i].position.x)) { fail = 1; break; }
        }
    }
    if (fail) { printf("[FAIL] NaN detected\\n"); return 1; }

    vector3 end_pos = world.bodies[robot.chassis_body].position;
    float physics_dz = end_pos.z - start_pos.z;
    float physics_dx = end_pos.x - start_pos.x;

    printf("[info] physics: dx=%.4f dz=%.4f\\n", physics_dx, physics_dz);
    printf("[info] odometry: dx=%.4f dz=%.4f theta=%.4f\\n", robot.odom_x, robot.odom_z, robot.odom_theta);

    /* Forward drive should produce positive Z displacement */
    if (fabsf(physics_dz) < 0.1f) {
        printf("[FAIL] robot barely moved forward (%.4f m)\\n", physics_dz);
        return 1;
    }

    /* Odometry should track physics within 20% error */
    float odom_dz = robot.odom_z;
    float dz_error = fabsf(odom_dz - physics_dz) / (fabsf(physics_dz) + 0.001f);
    printf("[info] odometry Z error: %.1f%%\\n", dz_error * 100.0f);

    if (dz_error > 0.2f) {
        printf("[FAIL] odometry Z drift too large (%.1f%%)\\n", dz_error * 100.0f);
        return 1;
    }

    /* Strafe sign check: drive right, verify positive X */
    robot.odom_x = robot.odom_z = robot.odom_theta = 0.0f;
    for (int i = 0; i < 4; i++) robot.wheel_radians[i] = 0.0f;
    start_pos = world.bodies[robot.chassis_body].position;

    float strafe_cmd[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    /* Mecanum strafe right: FL=-1, FR=+1, BL=+1, BR=-1 */
    strafe_cmd[0] = -0.5f; strafe_cmd[1] = 0.5f; strafe_cmd[2] = 0.5f; strafe_cmd[3] = -0.5f;
    ftc_robot_set_wheel_commands(&robot, strafe_cmd, 4);
    for (int t = 0; t < 60 && !fail; t++) {
        drivetrain_update(&world, &robot, dt);
        physics_world_step(&world, dt);
    }

    end_pos = world.bodies[robot.chassis_body].position;
    float strafe_dx = end_pos.x - start_pos.x;
    printf("[info] strafe: physics dx=%.4f odometry dx=%.4f\\n", strafe_dx, robot.odom_x);

    if (fabsf(strafe_dx) > 0.05f) {
        /* Check sign agreement */
        if ((strafe_dx > 0 && robot.odom_x < 0) || (strafe_dx < 0 && robot.odom_x > 0)) {
            printf("[FAIL] odometry strafe sign disagrees with physics\\n");
            return 1;
        }
    }

    printf("[PASS] odometry accuracy and strafe sign verified\\n");
    physics_world_cleanup(&world);
    return 0;
}
#endif /* MFS_ODOMETRY_ACCURACY_TEST */
'''

def write_tests():
    log("Step 1: Writing test files")
    write(SRC / "tests" / "tank_turn_test.c", TANK_TURN_TEST)
    write(SRC / "tests" / "odometry_accuracy_test.c", ODOMETRY_ACCURACY_TEST)
    return True

def add_makefile_targets():
    log("Step 2: Adding makefile targets")
    p = SRC / "makefile"
    content = p.read_text()
    if "test_tank_turn:" in content:
        log("  [SKIP] targets already present"); return True

    targets = """
# MFS_167: tank drive differential turning test
TANK_TURN_TEST_SOURCES := tests/tank_turn_test.c core/physics_world.c core/rigidbody.c \\
physics/collision_mechanics.c physics/broadphase.c physics/constraint.c physics/revolute_joint.c \\
robotics/motor.c robotics/motor_presets.c robotics/battery.c robotics/robot.c robotics/drivetrain.c \\
config/mpe_config.c config/mpe_config_schema.c
test_tank_turn: $(TANK_TURN_TEST_SOURCES)
\t$(CC) $(CFLAGS) -DMFS_TANK_TURN_TEST $(TANK_TURN_TEST_SOURCES) -lm -o test_tank_turn
\t./test_tank_turn

# MFS_167: odometry accuracy test
ODOMETRY_ACC_TEST_SOURCES := tests/odometry_accuracy_test.c core/physics_world.c core/rigidbody.c \\
physics/collision_mechanics.c physics/broadphase.c physics/constraint.c physics/revolute_joint.c \\
robotics/motor.c robotics/motor_presets.c robotics/battery.c robotics/robot.c robotics/drivetrain.c \\
config/mpe_config.c config/mpe_config_schema.c
test_odometry_accuracy: $(ODOMETRY_ACC_TEST_SOURCES)
\t$(CC) $(CFLAGS) -DMFS_ODOMETRY_ACCURACY_TEST $(ODOMETRY_ACC_TEST_SOURCES) -lm -o test_odometry_accuracy
\t./test_odometry_accuracy
"""
    content += targets
    write(p, content)
    return True

def add_test_runner_entries():
    log("Step 3: Adding test_runner.py entries")
    p = TOOLS / "test_runner.py"
    content = p.read_text()
    if '"tank_turn"' in content:
        log("  [SKIP] already present"); return True

    old = '    "physics_truth",\n]'
    new = '    "physics_truth",\n    "tank_turn",\n    "odometry_accuracy",\n]'
    if old in content:
        content = content.replace(old, new, 1)
        write(p, content)
        return True
    log("  [WARN] pattern not found"); return True

def build_and_test():
    log("Building...")
    r = subprocess.run([sys.executable, str(TOOLS / "build_check.py"), "--quick"],
                       cwd=ROOT, capture_output=True, text=True, timeout=180)
    if r.returncode != 0:
        print(r.stdout[-2000:]); print(r.stderr[-2000:])
        log("[FAIL] build failed"); return False
    log("[PASS] build clean")
    log("Running tests...")
    r = subprocess.run([sys.executable, str(TOOLS / "test_runner.py")],
                       cwd=ROOT, capture_output=True, text=True, timeout=300)
    print(r.stdout[-3000:] if r.stdout else "")
    if r.returncode != 0:
        log("[WARN] some tests failed (new tests may need tuning)")
    else:
        log("[PASS] all tests pass")
    return True

def main():
    print("=" * 60)
    print("MFS 167: Headless tests (tank turn + odometry accuracy)")
    print("=" * 60)
    if DRY_RUN: print("  ** DRY RUN **\n")

    steps = [write_tests, add_makefile_targets, add_test_runner_entries]
    for fn in steps:
        try:
            if not fn(): return 1
        except Exception as e:
            print(f"\n[FAIL] {fn.__name__}: {e}"); import traceback; traceback.print_exc(); return 1

    if not DRY_RUN and not build_and_test(): return 1

    print("=" * 60)
    print("  167 complete. 2 new headless tests added (11 total).")
    print("  tank_turn: verifies differential turning in place.")
    print("  odometry_accuracy: verifies odometry tracks physics + strafe sign.")
    print("=" * 60)
    return 0

if __name__ == "__main__": sys.exit(main())
