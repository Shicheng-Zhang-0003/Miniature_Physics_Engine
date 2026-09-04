/* MFS_FTC_DEBUG: probe motor torque magnitude + wheel contact behavior. */
#ifdef MPE_FTC_DEBUG_TEST
#include <stdio.h>
#include <math.h>
#include "core/physics_world.h"
#include "physics/constraint.h"
#include "config/mpe_config.h"
#include "robotics/robot.h"
#include "robotics/drivetrain.h"
#include "robotics/motor.h"

int main(void) {
    mpe_config_init();
    physics_world world;
    physics_world_init(&world);
    constraint_pool_init();

    ftc_robot robot;
    int rc = ftc_robot_create(&world, &robot, 0.0f, ftc_robot_rest_height(), 0.0f, MOTOR_GB_5203_30);
    if (rc != 0) { printf("[FAIL] could not create robot\n"); return 1; }

    const float dt = 1.0f / 60.0f;

    /* One tick of full forward */
    drivetrain_tank(&robot, 1.0f, 1.0f);
    drivetrain_update(&world, &robot, dt);

    printf("=== Motor 0 electrical/mechanical state after 1 tick ===\n");
    motor *m = &robot.wheel_motors[0];
    printf("  command          = %.3f\n", m->command);
    printf("  current          = %.3f A\n", m->current);
    printf("  torque (shaft)   = %.4f N*m\n", m->torque);
    printf("  output_torque    = %.4f N*m   <-- WATCH THIS\n", m->output_torque);
    printf("  gear_ratio       = %.2f\n", m->gear_ratio);
    printf("  efficiency       = %.3f\n", m->efficiency);
    printf("  rpm              = %.1f\n", m->rpm);
    printf("  free_speed_rad_s = %.2f\n", m->free_speed_rad_s);

    float wheel_I = 0.5f * 0.2f * 0.05f * 0.05f; /* 0.5*m*r^2 for the 0.2kg wheel */
    printf("  wheel inertia    = %.6f kg*m^2\n", wheel_I);
    printf("  implied ang accel= %.1f rad/s^2 (output_torque / I)\n", m->output_torque / wheel_I);

    physics_world_step(&world, dt);

    printf("\n=== After physics step ===\n");
    for (int i = 0; i < 4; i++) {
        int wb = robot.wheel_bodies[i];
        printf("  Wheel %d: ang_vel=(%.2f,%.2f,%.2f) lin_vel=(%.3f,%.3f,%.3f) pos.y=%.4f\n",
               i,
               world.bodies[wb].angular_velocity.x,
               world.bodies[wb].angular_velocity.y,
               world.bodies[wb].angular_velocity.z,
               world.bodies[wb].velocity.x,
               world.bodies[wb].velocity.y,
               world.bodies[wb].velocity.z,
               world.bodies[wb].position.y);
    }
    int cb = robot.chassis_body;
    printf("  Chassis: lin_vel=(%.3f,%.3f,%.3f) pos.y=%.4f\n",
           world.bodies[cb].velocity.x, world.bodies[cb].velocity.y,
           world.bodies[cb].velocity.z, world.bodies[cb].position.y);

    printf("\n=== Verdict heuristics ===\n");
    if (m->output_torque > 5.0f)
        printf("  [SUSPECT] output_torque %.2f N*m is HUGE for a 100mm wheel -> gear ratio likely double-applied\n", m->output_torque);
    else
        printf("  [OK] output_torque %.2f N*m looks plausible\n", m->output_torque);

    return 0;
}
#endif /* MPE_FTC_DEBUG_TEST */
