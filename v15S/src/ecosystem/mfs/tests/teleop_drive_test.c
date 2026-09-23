/* MPE_FTC_077: Teleop drive validation test */
#ifdef MPE_TELEOP_DRIVE_TEST
#include <stdio.h>
#include <math.h>
#include "core/physics_world.h"
#include "physics/constraint.h"
#include "config/mpe_config.h"
#include "modules/ftc/submodules/robot.h"
#include "modules/ftc/submodules/drivetrain.h"
#include "ecosystem/mfs/tests/mfs_test_common.h"

int main(void) {
    physics_world world;
    mfs_test_world(&world); /* 128 iters + tile floor (see header) */

    /* Create robot at origin, using goBILDA 5203 26.9:1 motors (223 RPM) */
    ftc_robot robot;
    int rc = ftc_robot_create(&world, &robot, 0.0f, ftc_robot_rest_height(), 0.0f, MOTOR_GB_5203_26_9);
    if (rc != 0) {
        printf("[FAIL] could not create robot\n");
        return 1;
    }

    float start_x, start_y, start_z;
    ftc_robot_get_position(&world, &robot, &start_x, &start_y, &start_z);

    const float dt = 1.0f / 60.0f;
    int fail = 0;
    int total_ticks = 180; /* 3 seconds */

    for (int t = 0; t < total_ticks; t++) {
        /* Full forward tank drive */
        drivetrain_tank(&robot, 1.0f, 1.0f);
        drivetrain_update(&world, &robot, dt);

        /* Step physics (includes constraints) */
        physics_world_step(&world, dt);

        /* Check for NaN */
        for (int i = 0; i < world.body_count; i++) {
            rigidbody *rb = &world.bodies[i];
            if ((!isfinite(rb->position.x)) || (!isfinite(rb->position.y)) || (!isfinite(rb->position.z)) ||
                (!isfinite(rb->velocity.x)) || (!isfinite(rb->velocity.y)) || (!isfinite(rb->velocity.z))) {
                printf("[FAIL] NaN detected in body %d at tick %d\n", i, t);
                fail = 1;
                break;
            }
        }
        if (fail) {
            break;
        }
    }

    if (!fail) {
        float end_x, end_y, end_z;
        ftc_robot_get_position(&world, &robot, &end_x, &end_y, &end_z);
        float dz = end_z - start_z;
        float dy = end_y - start_y;

        printf("[info] start=(%.3f,%.3f,%.3f) end=(%.3f,%.3f,%.3f)\n", start_x, start_y, start_z, end_x, end_y, end_z);
        printf("[info] displacement z=%.4f  dy=%.4f\n", dz, dy);
        printf("[info] motor RPM: [%.0f, %.0f, %.0f, %.0f]\n", robot.wheel_motors[0].rpm, robot.wheel_motors[1].rpm,
               robot.wheel_motors[2].rpm, robot.wheel_motors[3].rpm);
        printf("[info] battery: %.2fV (%.0f%%)\n", battery_get_voltage(&robot.battery, 0.0f),
               robot.battery.charge_fraction * 100.0f);

        /* Robot should have moved in some direction (z or x) */
        float total_displacement = sqrtf(dz * dz + (end_x - start_x) * (end_x - start_x));
        /* Straight-line drive must hold heading: yaw from quaternion
         * (solver pair-order asymmetry used to spin full-power straights). */
        quaternion q = world.bodies[robot.chassis_body].orientation;
        float heading = fabsf(atan2f(2.0f * (q.w * q.y + q.x * q.z),
                                     1.0f - 2.0f * (q.y * q.y + q.x * q.x)));
        printf("[info] heading drift=%.4f rad\n", heading);
        if (total_displacement < 0.5f /* MPE_FTC_079: require real driving, not just falling */) {
            printf("[FAIL] robot did not move (displacement=%.4f)\n", total_displacement);
            fail = 1;
        } else if (fabsf(dy) > 1.0f) {
            printf("[FAIL] robot flipped or fell (dy=%.4f)\n", dy);
            fail = 1;
        } else if (heading > 0.3f) {
            printf("[FAIL] straight drive yawed (%.4f rad)\n", heading);
            fail = 1;
        } else {
            printf("[PASS] robot drove under motor power (displacement=%.4f, dy=%.4f)\n", total_displacement, dy);
        }
    }

    physics_world_cleanup(&world);
    return fail;
}
#endif /* MPE_TELEOP_DRIVE_TEST */
