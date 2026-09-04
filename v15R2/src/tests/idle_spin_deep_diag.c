
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
    printf("\n=== IDLE SPIN DEEP DIAGNOSTIC ===\n");

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

    float zero[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    ftc_robot_set_wheel_commands(&robot, zero, 4);

    /* settle */
    for (int i = 0; i < 120; i++) {
        drivetrain_update(&world, &robot, DT);
        physics_world_step(&world, DT);
    }

    printf("after 120-frame settle, per wheel:\n");
    printf("  %-6s %-7s %-8s %-11s %-10s %-11s\n",
           "wheel", "asleep", "cmd", "out_torque", "current", "axle_omega");
    for (int w = 0; w < robot.wheel_count; w++) {
        int wi = robot.wheel_bodies[w];
        rigidbody *wheel = &world.bodies[wi];
        printf("  %-6d %-7d %-8.3f %-11.4f %-10.3f %-11.3f\n",
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
    printf("single-step delta (omega_before -> omega_after):\n");
    for (int w = 0; w < robot.wheel_count; w++) {
        float after = axle_omega(&world, robot.wheel_bodies[w]);
        printf("  wheel[%d]: %.3f -> %.3f  (delta=%+.4f)\n", w, before[w], after, after - before[w]);
    }

    rigidbody *ch = &world.bodies[robot.chassis_body];
    printf("chassis: asleep=%d lin_speed=%.4f ang_vel_y=%.4f\n",
           (int)ch->is_sleeping,
           sqrtf(ch->velocity.x * ch->velocity.x + ch->velocity.z * ch->velocity.z),
           ch->angular_velocity.y);

    /* verdict */
    int any_asleep = 0, all_asleep = 1;
    for (int w = 0; w < robot.wheel_count; w++) {
        int s = (int)world.bodies[robot.wheel_bodies[w]].is_sleeping;
        if (s) any_asleep = 1; else all_asleep = 0;
    }
    printf("\nVERDICT: ");
    if (all_asleep) printf("ALL wheels ASLEEP -> sleep-system bug (linear-only sleep check)\n");
    else if (any_asleep) printf("SOME wheels asleep -> partial sleep bug\n");
    else printf("wheels AWAKE -> check single-step delta to see if braking lands\n");

    physics_world_cleanup(&world);
    return 0;
}
#endif
