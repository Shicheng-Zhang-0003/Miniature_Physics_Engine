/* MFS_FTC_INTEGRATION: Full FTC robot integration test.
 * Creates a robot, drives forward 2s, turns 90° for 1s,
 * strafes right 1s, then verifies final position is reasonable.
 */
#ifdef MPE_FTC_INTEGRATION_TEST
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
    printf("[info] start pos: (%.3f, %.3f, %.3f)\n", start_x, start_y, start_z);

    const float dt = 1.0f / 60.0f;
    int fail = 0;

    /* Phase 1: Drive forward for 2 seconds (120 ticks) */
    printf("[info] Phase 1: driving forward 2s\n");
    for (int t = 0; t < 120 && !fail; t++) {
        drivetrain_tank(&robot, 1.0f, 1.0f);
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

    float p1_x, p1_y, p1_z;
    ftc_robot_get_position(&world, &robot, &p1_x, &p1_y, &p1_z);
    printf("[info] after forward: (%.3f, %.3f, %.3f)\n", p1_x, p1_y, p1_z);

    /* Phase 2: Turn right for 1 second (60 ticks) */
    printf("[info] Phase 2: turning right 1s\n");
    for (int t = 0; t < 60 && !fail; t++) {
        drivetrain_tank(&robot, 0.5f, -0.5f);
        drivetrain_update(&world, &robot, dt);
        physics_world_step(&world, dt);
    }

    float p2_x, p2_y, p2_z;
    ftc_robot_get_position(&world, &robot, &p2_x, &p2_y, &p2_z);
    printf("[info] after turn: (%.3f, %.3f, %.3f)\n", p2_x, p2_y, p2_z);

    /* Phase 3: Strafe right for 1 second (60 ticks) */
    printf("[info] Phase 3: strafing right 1s\n");
    for (int t = 0; t < 60 && !fail; t++) {
        drivetrain_mecanum(&robot, 0.0f, 1.0f, 0.0f);
        drivetrain_update(&world, &robot, dt);
        physics_world_step(&world, dt);
    }

    float end_x, end_y, end_z;
    ftc_robot_get_position(&world, &robot, &end_x, &end_y, &end_z);
    printf("[info] final pos: (%.3f, %.3f, %.3f)\n", end_x, end_y, end_z);

    if (fail) return 1;

    /* Verify robot moved significantly from start */
    float total_dist = sqrtf((end_x - start_x) * (end_x - start_x) +
                             (end_z - start_z) * (end_z - start_z));
    printf("[info] total displacement: %.4f m\n", total_dist);

    if (total_dist < 0.5f) {
        printf("[FAIL] robot barely moved (%.4f m)\n", total_dist);
        return 1;
    }

    /* Verify robot stayed upright (y didn't change much) */
    float dy = fabsf(end_y - start_y);
    if (dy > 0.5f) {
        printf("[FAIL] robot flipped or fell (dy=%.4f)\n", dy);
        return 1;
    }

    printf("[PASS] FTC integration: robot drove, turned, strafed, stayed upright\n");
    return 0;
}
#endif /* MPE_FTC_INTEGRATION_TEST */
