
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
    printf("\n=== IDLE WHEEL-SPIN DIAGNOSTIC ===\n");

    physics_world world;
    physics_world_init(&world);
    constraint_pool_init();

    /* floor at y=0 */
    physics_world_add_cube(&world,
        (vector3){0.0f, -0.5f, 0.0f},
        (vector3){10.0f, 0.5f, 10.0f}, 0.0f);

    ftc_robot robot;
    int rc = ftc_robot_create(&world, &robot, 0.0f, ftc_robot_rest_height(), 0.0f, MOTOR_GB_5203_30);
    if (rc != 0) { printf("[FAIL] robot create\n"); return 1; }

    /* Ensure zero commands */
    float zero[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    ftc_robot_set_wheel_commands(&robot, zero, 4);

    /* Settle 120 frames with zero input */
    for (int i = 0; i < 120; i++) {
        drivetrain_update(&world, &robot, DT);
        physics_world_step(&world, DT);
    }
    printf("settled. now monitoring 300 frames of ZERO input:\n");

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
            printf("] chassis_speed=%.4f pos=(%.3f,%.3f)\n", cs, ch->position.x, ch->position.z);
        }
    }

    printf("\nmax |wheel axle omega| over idle = %.4f rad/s\n", max_wheel_omega);
    printf("max chassis speed over idle     = %.4f m/s\n", max_chassis_speed);
    if (max_wheel_omega > 0.5f) {
        printf("VERDICT: wheels SPIN at idle -> real bug, see pattern above\n");
    } else if (max_wheel_omega > 0.05f) {
        printf("VERDICT: mild jitter at idle (contact solver)\n");
    } else {
        printf("VERDICT: idle is stable in headless -> bug may be GUI-side\n");
    }

    physics_world_cleanup(&world);
    return 0;
}
#endif
