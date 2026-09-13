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
    if (rc != 0) { printf("[FAIL] could not create robot\n"); return 1; }

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
                printf("[FAIL] NaN in body %d at tick %d\n", i, t);
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

    printf("[info] displacement=%.4f heading_change=%.4f\n", displacement, heading_change);

    if (displacement > 0.3f) {
        printf("[FAIL] robot translated too much during differential turn (%.4f m)\n", displacement);
        return 1;
    }
    if (heading_change < 0.1f) {
        printf("[FAIL] robot did not rotate during differential turn\n");
        return 1;
    }

    printf("[PASS] tank drive differential turn works (displacement=%.4f, heading=%.4f)\n",
           displacement, heading_change);
    physics_world_cleanup(&world);
    return 0;
}
#endif /* MFS_TANK_TURN_TEST */
