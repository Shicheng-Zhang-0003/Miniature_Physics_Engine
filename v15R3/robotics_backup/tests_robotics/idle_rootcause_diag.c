
#ifdef MFS_IDLE_ROOTCAUSE_DIAG
#include <stdio.h>
#include <math.h>
#include "core/physics_world.h"
#include "core/rigidbody.h"
#include "physics/constraint.h"
#include "config/mpe_config.h"
#include "robotics/robot.h"
#include "robotics/drivetrain.h"

static const float DT = 1.0f / 60.0f;

int main(void) {
    mpe_config_init();
    printf("\n=== IDLE ROOT-CAUSE DIAGNOSTIC ===\n");

    physics_world world;
    physics_world_init(&world);
    constraint_pool_init();

    physics_world_add_cube(&world,
        (vector3){0.0f, -0.5f, 0.0f},
        (vector3){10.0f, 0.5f, 10.0f}, 0.0f);

    ftc_robot robot;
    if (ftc_robot_create(&world, &robot, 0.0f, ftc_robot_rest_height(), 0.0f, MOTOR_GB_5203_30) != 0) {
        printf("[FAIL] robot create\n"); return 1;
    }

    const char *names[4] = {"FL", "FR", "BL", "BR"};
    printf("wheel configuration:\n");
    for (int w = 0; w < robot.wheel_count; w++) {
        int wi = robot.wheel_bodies[w];
        rigidbody *wheel = &world.bodies[wi];
        printf("  [%d]=%s  roller_angle=%8.2f deg  is_mecanum=%d  radius=%.4f\n",
               w, names[w],
               wheel->roller_angle_rad * 57.2957795f,
               wheel->is_mecanum ? 1 : 0,
               wheel->radius);
    }

    float zero[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    ftc_robot_set_wheel_commands(&robot, zero, 4);

    for (int i = 0; i < 120; i++) {
        drivetrain_update(&world, &robot, DT);
        physics_world_step(&world, DT);
    }

    printf("idle (zero input) after settle:\n");
    for (int w = 0; w < robot.wheel_count; w++) {
        int wi = robot.wheel_bodies[w];
        rigidbody *wheel = &world.bodies[wi];
        vector3 axle = wheel->cached_axes[0];
        if (vector3_length_squared(axle) < 0.0001f) {
            axle = vector4_rotate_to_vector3(wheel->orientation, (vector3){1.0f, 0.0f, 0.0f});
        }
        float oa = vector3_dot(wheel->angular_velocity, axle);
        printf("  [%d]=%s  axle_omega=%8.4f rad/s\n", w, names[w], oa);
    }

    rigidbody *ch = &world.bodies[robot.chassis_body];
    printf("chassis vel=(%.5f, %.5f, %.5f) ang_vel_y=%.5f\n",
           ch->velocity.x, ch->velocity.y, ch->velocity.z, ch->angular_velocity.y);

    physics_world_cleanup(&world);
    return 0;
}
#endif
