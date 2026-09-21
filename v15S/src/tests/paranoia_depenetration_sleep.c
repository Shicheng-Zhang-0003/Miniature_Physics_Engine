/* PARANOIA TEST: Depenetration correctness - sleeping bodies must never move. */
#ifdef mpe_paranoia_depenetration_sleep
#include <stdio.h>
#include <math.h>
#include "core/physics_world.h"
#include "physics/constraint.h"
#include "config/mpe_config.h"

int main(void) {
    mpe_config_init();
    int fail = 0;

    /* Test 1: Sleeping stack - depenetration must not wake or move sleeping bodies */
    {
        mpe_config_init();
        physics_world world;
        physics_world_init(&world);
        constraint_pool_init(&world);

        /* Create a 5-cube tower with 1cm overlaps, all initially asleep */
        for (int i = 0; i < 5; i++) {
            int idx = physics_world_add_cube(&world, (vector3){0.0f, 0.5f + (float)i * 0.99f, 0.0f}, (vector3){0.5f, 0.5f, 0.5f}, 1.0f);
            world.bodies[idx].restitution = 0.0f;
            world.bodies[idx].friction_static = 0.8f;
            world.bodies[idx].friction_kinetic = 0.7f;
            world.bodies[idx].is_sleeping = true;
            world.bodies[idx].sleep_timer = 1.0f;
        }

        const float dt = 1.0f / 60.0f;
        int max_wake = 0;

        for (int t = 0; t < 1200; t++) {
            physics_world_step(&world, dt);
            int awake_count = 0;
            for (int i = 0; i < world.body_count; i++) {
                if (!world.bodies[i].is_sleeping) awake_count++;
            }
            if (awake_count > max_wake) max_wake = awake_count;
        }

        printf("[INFO] sleeping_stack max_awake=%d (expected 0)\n", max_wake);
        if (max_wake > 0) { printf("[FAIL] depenetration woke sleeping bodies\n"); fail = 1; }
        else { printf("[PASS] depenetration respects sleep state\n"); }

        physics_world_cleanup(&world);
    }

    /* Test 2: Mixed awake/sleeping - depenetration must only move awake bodies */
    {
        physics_world world;
        physics_world_init(&world);
        constraint_pool_init(&world);

        /* Sleeping cube on floor */
        int sleeping = physics_world_add_cube(&world, (vector3){0.0f, 0.5f, 0.0f}, (vector3){0.5f, 0.5f, 0.5f}, 1.0f);
        world.bodies[sleeping].restitution = 0.0f;
        world.bodies[sleeping].friction_static = 0.8f;
        world.bodies[sleeping].friction_kinetic = 0.7f;
        world.bodies[sleeping].is_sleeping = true;
        world.bodies[sleeping].sleep_timer = 1.0f;

        /* Awake sphere falling on top */
        int awake = physics_world_add_sphere(&world, 0.3f, 1.0f, (vector3){0.0f, 2.0f, 0.0f});
        world.bodies[awake].restitution = 0.0f;
        world.bodies[awake].friction_static = 0.8f;
        world.bodies[awake].friction_kinetic = 0.7f;

        const float dt = 1.0f / 60.0f;
        float sleeping_y_initial = world.bodies[sleeping].position.y;
        int sleeping_woke = 0;

        for (int t = 0; t < 600; t++) {
            physics_world_step(&world, dt);
            if (world.bodies[sleeping].is_sleeping == false) {
                sleeping_woke = 1;
            }
        }

        float sleeping_y_final = world.bodies[sleeping].position.y;
        float y_drift = fabsf(sleeping_y_final - sleeping_y_initial);

        printf("[INFO] mixed_awake_sleep sleeping_woke=%d y_drift=%.6f\n", sleeping_woke, y_drift);
        if (sleeping_woke) { printf("[FAIL] depenetration woke sleeping body\n"); fail = 1; }
        if (y_drift > 0.001f) { printf("[FAIL] depenetration moved sleeping body %.6f\n", y_drift); fail = 1; }
        else { printf("[PASS] depenetration only moves awake bodies\n"); }

        physics_world_cleanup(&world);
    }

    /* Test 3: Deep spawn overlap - depenetration must resolve without exploding */
    {
        physics_world world;
        physics_world_init(&world);
        constraint_pool_init(&world);

        /* Two cubes spawned with 50% overlap */
        int a = physics_world_add_cube(&world, (vector3){0.0f, 0.5f, 0.0f}, (vector3){0.5f, 0.5f, 0.5f}, 1.0f);
        int b = physics_world_add_cube(&world, (vector3){0.0f, 0.5f, 0.0f}, (vector3){0.5f, 0.5f, 0.5f}, 1.0f);
        world.bodies[a].restitution = 0.0f;
        world.bodies[b].restitution = 0.0f;
        world.bodies[a].friction_static = 0.8f;
        world.bodies[b].friction_static = 0.8f;
        world.bodies[a].friction_kinetic = 0.7f;
        world.bodies[b].friction_kinetic = 0.7f;

        const float dt = 1.0f / 60.0f;
        float max_sep = 0.0f, max_vel = 0.0f;
        int nan_count = 0;

        for (int t = 0; t < 1800; t++) {
            physics_world_step(&world, dt);
            rigidbody *ba = &world.bodies[a];
            rigidbody *bb = &world.bodies[b];
            float sep = fabsf(ba->position.y - bb->position.y);
            float vel_a = vector3_length(ba->velocity);
            float vel_b = vector3_length(bb->velocity);
            if (sep > max_sep) max_sep = sep;
            if (vel_a > max_vel) max_vel = vel_a;
            if (vel_b > max_vel) max_vel = vel_b;
            if (!isfinite(ba->position.y) || !isfinite(bb->position.y)) nan_count++;
        }

        printf("[INFO] deep_overlap max_sep=%.3f max_vel=%.3f nan=%d\n", max_sep, max_vel, nan_count);
        if (nan_count > 0) { printf("[FAIL] deep overlap produced NaN\n"); fail = 1; }
        if (max_vel > 50.0f) { printf("[FAIL] deep overlap explosion vel=%.1f\n", max_vel); fail = 1; }
        if (max_sep > 5.0f) { printf("[FAIL] deep overlap excessive separation\n"); fail = 1; }
        else { printf("[PASS] deep overlap resolved smoothly\n"); }

        physics_world_cleanup(&world);
    }

    return fail;
}
#endif