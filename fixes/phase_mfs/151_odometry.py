#!/usr/bin/env python3
"""
MFS 151: Wheel Encoders + IMU Odometry (Milestone 4)
=====================================================
Adds dead-reckoning odometry to the robot.
1. robot.h/c: add wheel_radians[4], odom_x, odom_z, odom_theta to ftc_robot.
2. robot.c: zero them in ftc_robot_create.
3. drivetrain.c: integrate wheel omegas and chassis yaw into the odometry state.
   Uses chassis angular_velocity.y as an "IMU" for heading.
4. Adds a headless diagnostic to verify odometry against physics truth.

Usage:
    cd <project_root>
    python3 fixes/phase_mfs/151_odometry.py [--dry-run]
"""
import sys
import subprocess
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
ROOT = SCRIPT_DIR.parent.parent
SRC = ROOT / "v15R2" / "src"
TOOLS = ROOT / "tools"
sys.path.insert(0, str(TOOLS))
DRY_RUN = "--dry-run" in sys.argv

def log(msg): print(f"  [151] {msg}")


def step_add_fields():
    log("Step 1: Adding odometry fields to ftc_robot struct")
    fields = """
    /* MFS_151_ODOMETRY: Wheel encoders and pose estimation */
    float wheel_radians[4];
    float odom_x, odom_z, odom_theta;
"""
    for fname in ["robot.h", "robot.c"]:
        p = SRC / "robotics" / fname
        if not p.exists(): continue
        content = p.read_text()
        if "MFS_151_ODOMETRY" in content:
            log(f"  [SKIP] already in {fname}")
            return True
        if "} ftc_robot;" in content:
            content = content.replace("} ftc_robot;", fields + "} ftc_robot;", 1)
            if not DRY_RUN: p.write_text(content)
            log(f"  [OK] fields added to {fname}")
            return True
    log("  [FAIL] could not find `} ftc_robot;` in robot.h or robot.c")
    return False


def step_zero_fields():
    log("Step 2: Zeroing odometry fields in ftc_robot_create")
    p = SRC / "robotics" / "robot.c"
    content = p.read_text()
    if "MFS_151_ZERO" in content:
        log("  [SKIP] already zeroed")
        return True

    idx = content.find("ftc_robot_create")
    if idx == -1:
        log("  [FAIL] ftc_robot_create not found")
        return False

    b = content.find("{", idx)
    if b == -1: return False

    zero_code = "\n    /* MFS_151_ZERO */\n    robot->odom_x = robot->odom_z = robot->odom_theta = 0.0f;\n    for (int mfs_i = 0; mfs_i < 4; mfs_i++) robot->wheel_radians[mfs_i] = 0.0f;\n"
    content = content[:b+1] + zero_code + content[b+1:]
    if not DRY_RUN: p.write_text(content)
    log("  [OK] zeroed in ftc_robot_create")
    return True


def step_integrate_odometry():
    log("Step 3: Integrating odometry in drivetrain_update")
    p = SRC / "robotics" / "drivetrain.c"
    content = p.read_text()
    if "MFS_151_INTEGRATE" in content:
        log("  [SKIP] already integrated")
        return True

    idx = content.find("drivetrain_update")
    if idx == -1:
        log("  [FAIL] drivetrain_update not found")
        return False

    b = content.find("{", idx)
    if b == -1: return False

    depth = 0
    end_b = -1
    for i in range(b, len(content)):
        if content[i] == '{': depth += 1
        elif content[i] == '}':
            depth -= 1
            if depth == 0:
                end_b = i
                break

    if end_b == -1:
        log("  [FAIL] could not find end of drivetrain_update")
        return False

    odom_code = """
    /* MFS_151_INTEGRATE: Odometry integration */
    {
        float v_fl = 0.0f, v_fr = 0.0f, v_bl = 0.0f, v_br = 0.0f;
        for (int mfs_i = 0; mfs_i < robot->wheel_count; mfs_i++) {
            int wi = robot->wheel_bodies[mfs_i];
            if (wi >= 0 && wi < world->body_count) {
                rigidbody *w = &world->bodies[wi];
                vector3 axle = w->cached_axes[0];
                if (vector3_length_squared(axle) < 0.0001f) {
                    axle = vector4_rotate_to_vector3(w->orientation, (vector3){1.0f, 0.0f, 0.0f});
                }
                float omega = vector3_dot(w->angular_velocity, axle);
                robot->wheel_radians[mfs_i] += omega * dt;
                float v = omega * w->radius;
                if (mfs_i == 0) v_fl = v;
                else if (mfs_i == 1) v_fr = v;
                else if (mfs_i == 2) v_bl = v;
                else if (mfs_i == 3) v_br = v;
            }
        }

        float v_x = (v_fl + v_fr + v_bl + v_br) * 0.25f;
        float v_z = (v_fl - v_fr - v_bl + v_br) * 0.25f;

        float v_theta = 0.0f;
        if (robot->chassis_body >= 0 && robot->chassis_body < world->body_count) {
            v_theta = world->bodies[robot->chassis_body].angular_velocity.y;
        }

        float cos_t = cosf(robot->odom_theta);
        float sin_t = sinf(robot->odom_theta);
        robot->odom_x += (v_x * cos_t - v_z * sin_t) * dt;
        robot->odom_z += (v_x * sin_t + v_z * cos_t) * dt;
        robot->odom_theta += v_theta * dt;
    }
"""
    content = content[:end_b] + odom_code + content[end_b:]
    if not DRY_RUN: p.write_text(content)
    log("  [OK] integrated in drivetrain_update")
    return True


def step_add_diagnostic():
    log("Step 4: Adding odometry diagnostic test")
    diag_path = SRC / "tests" / "odometry_diag.c"

    DIAG = '''
#ifdef MFS_ODOM_DIAG
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
    printf("\\n=== ODOMETRY DIAGNOSTIC ===\\n");

    physics_world world;
    physics_world_init(&world);
    constraint_pool_init();

    physics_world_add_cube(&world, (vector3){0.0f, -0.5f, 0.0f}, (vector3){10.0f, 0.5f, 10.0f}, 0.0f);

    ftc_robot robot;
    ftc_robot_create(&world, &robot, 0.0f, ftc_robot_rest_height(), 0.0f, MOTOR_GB_5203_30);

    float cmd[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    ftc_robot_set_wheel_commands(&robot, cmd, 4);

    for (int i = 0; i < 120; i++) {
        drivetrain_update(&world, &robot, DT);
        physics_world_step(&world, DT);
    }

    robot.odom_x = robot.odom_z = robot.odom_theta = 0.0f;
    for(int i=0; i<4; i++) robot.wheel_radians[i] = 0.0f;

    rigidbody *ch = &world.bodies[robot.chassis_body];
    vector3 start_pos = ch->position;

    printf("Test 1: Drive forward (cmd=0.5) for 2 seconds\\n");
    cmd[0] = cmd[1] = cmd[2] = cmd[3] = 0.5f;
    ftc_robot_set_wheel_commands(&robot, cmd, 4);
    for (int i = 0; i < 120; i++) {
        drivetrain_update(&world, &robot, DT);
        physics_world_step(&world, DT);
    }
    printf("  Physics: dx=%.4f dz=%.4f\\n", ch->position.x - start_pos.x, ch->position.z - start_pos.z);
    printf("  Odom:    dx=%.4f dz=%.4f theta=%.4f\\n", robot.odom_x, robot.odom_z, robot.odom_theta);

    physics_world_cleanup(&world);
    return 0;
}
#endif
'''
    if not DRY_RUN:
        diag_path.write_text(DIAG)
    log("  [OK] Wrote odometry_diag.c")

    makefile = SRC / "makefile"
    mc = makefile.read_text()
    if "test_odometry_diag:" not in mc:
        mc += """
# MFS_151: odometry diagnostic
ODOM_DIAG_SOURCES := tests/odometry_diag.c core/physics_world.c core/rigidbody.c \\
physics/collision_mechanics.c physics/broadphase.c physics/constraint.c physics/revolute_joint.c \\
robotics/motor.c robotics/motor_presets.c robotics/battery.c robotics/robot.c robotics/drivetrain.c \\
config/mpe_config.c config/mpe_config_schema.c
test_odometry_diag: $(ODOM_DIAG_SOURCES)
\t$(CC) $(CFLAGS) -DMFS_ODOM_DIAG $(ODOM_DIAG_SOURCES) -lm -o test_odometry_diag
\t./test_odometry_diag
"""
        if not DRY_RUN:
            makefile.write_text(mc)
        log("  [OK] Added makefile target")
    return True


def step_run_diag():
    log("Step 5: Building and running odometry diagnostic")
    r = subprocess.run(["make", "-C", str(SRC), "test_odometry_diag"],
                       capture_output=True, text=True, timeout=180)
    print(r.stdout[-2500:] if r.stdout else "")
    if r.returncode != 0:
        print(r.stderr[-1500:] if r.stderr else "")
        log("[FAIL] build/run failed")
        return False
    log("[PASS] diagnostic complete")
    return True


def main():
    print("=" * 60)
    print("MFS 151: Wheel Encoders + IMU Odometry (Milestone 4)")
    print("=" * 60)
    if DRY_RUN:
        print("  ** DRY RUN — no files modified **\n")

    if not step_add_fields(): return 1
    if not step_zero_fields(): return 1
    if not step_integrate_odometry(): return 1
    if not step_add_diagnostic(): return 1

    if DRY_RUN:
        log("[DRY RUN] skipping build")
        return 0

    step_run_diag()

    print("=" * 60)
    return 0


if __name__ == "__main__":
    sys.exit(main())
