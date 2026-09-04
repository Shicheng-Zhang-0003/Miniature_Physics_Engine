
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
    printf("\n=== ODOMETRY DIAGNOSTIC ===\n");

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

    printf("Test 1: Drive forward (cmd=0.5) for 2 seconds\n");
    cmd[0] = cmd[1] = cmd[2] = cmd[3] = 0.5f;
    ftc_robot_set_wheel_commands(&robot, cmd, 4);
    for (int i = 0; i < 120; i++) {
        drivetrain_update(&world, &robot, DT);
        physics_world_step(&world, DT);
    }
    printf("  Physics: dx=%.4f dz=%.4f\n", ch->position.x - start_pos.x, ch->position.z - start_pos.z);
    printf("  Odom:    dx=%.4f dz=%.4f theta=%.4f\n", robot.odom_x, robot.odom_z, robot.odom_theta);

    physics_world_cleanup(&world);
    return 0;
}
#endif
