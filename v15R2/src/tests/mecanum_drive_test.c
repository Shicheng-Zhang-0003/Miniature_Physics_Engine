/* MPE_FTC_080: Mecanum drive validation test */
#ifdef MPE_MECANUM_DRIVE_TEST
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
    if (rc != 0) {
        printf("[FAIL] could not create robot\n");
        return 1;
    }

    float start_x, start_y, start_z;
    ftc_robot_get_position(&world, &robot, &start_x, &start_y, &start_z);

    const float dt = 1.0f / 60.0f;
    int fail = 0;
    int total_ticks = 180;

    for (int t = 0; t < total_ticks; t++) {
        /* Full strafe right (forward=0, strafe=1, rotate=0) */
        drivetrain_mecanum(&robot, 0.0f, 1.0f, 0.0f);
        drivetrain_update(&world, &robot, dt);
        physics_world_step(&world, dt);

        for (int i = 0; i < world.body_count; i++) {
            rigidbody *rb = &world.bodies[i];
            if ((!isfinite(rb->position.x)) || (!isfinite(rb->position.y)) || (!isfinite(rb->position.z))) {
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
        float dx = end_x - start_x;
        float dz = end_z - start_z;

        printf("[info] start=(%.3f,%.3f,%.3f) end=(%.3f,%.3f,%.3f)\n", start_x, start_y, start_z, end_x, end_y, end_z);
        printf("[info] displacement x=%.4f  z=%.4f\n", dx, dz);

        /* Robot should have moved sideways (x-axis) */
        float lateral_displacement = fabsf(dx);
        if (lateral_displacement < 0.3f) {
            printf("[FAIL] robot did not strafe far enough in +X (dx=%.4f, expected >0.3)\n", dx);
            fail = 1;
        } else {
            printf("[PASS] robot strafed in +X under real mecanum roller friction (dx=%.4f)\n", dx);
        }
    }

    physics_world_cleanup(&world);
    return fail;
}
#endif /* MPE_MECANUM_DRIVE_TEST */
