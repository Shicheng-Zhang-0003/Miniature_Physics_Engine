/* PARANOIA TEST: Contact solver - friction, restitution, Poisson, split impulse */
#ifdef mpe_paranoia_contact_solver
#include <stdio.h>
#include <math.h>
#include "core/physics_world.h"
#include "physics/constraint.h"
#include "config/mpe_config.h"

int main(void) {
    mpe_config_init();
    int fail = 0;

    /* Test 1: Poisson restitution - bounce series must follow e^2 exactly */
    {
        physics_world world;
        physics_world_init(&world);
        constraint_pool_init(&world);

        g_cfg.world.gravity = -9.81f;
        g_cfg.world.drag = 1.0f;

        int s = physics_world_add_sphere(&world, 0.5f, 1.0f, (vector3){0.0f, 5.0f, 0.0f});
        world.bodies[s].restitution = 0.8f;
        world.bodies[s].friction_static = 0.0f;
        world.bodies[s].friction_kinetic = 0.0f;
        rigidbody_wake(&world.bodies[s]);

        const float dt = 1.0f / 60.0f;
        float apex_heights[10];
        int apex_count = 0;

        for (int t = 0; t < 2000; t++) {
            physics_world_step(&world, dt);
            rigidbody *b = &world.bodies[s];
            if (b->position.y > 0.0f && (apex_count == 0 || b->position.y > apex_heights[apex_count-1])) {
                apex_heights[apex_count] = b->position.y;
                apex_count++;
            }
            if (apex_count >= 10) break;
        }

        float expected_apex = 5.0f;
        float e = 0.8f;
        int passed = 1;
        for (int i = 0; i < apex_count; i++) {
            expected_apex *= e * e;
            float ratio = apex_heights[i] / expected_apex;
            if (fabsf(ratio - 1.0f) > 0.02f) {
                printf("[FAIL] bounce %d: apex=%.4f expected=%.4f ratio=%.4f\n", i+1, apex_heights[i], expected_apex, ratio);
                fail = 1;
                passed = 0;
            }
        }
        if (passed) printf("[PASS] Poisson restitution e^2 series correct\n");
        physics_world_cleanup(&world);
    }

    /* Test 2: Coulomb friction - static hold on slope */
    {
        physics_world world;
        physics_world_init(&world);
        constraint_pool_init(&world);

        g_cfg.world.gravity = -9.81f;
        g_cfg.world.drag = 1.0f;

        int cube = physics_world_add_cube(&world, (vector3){0.0f, 1.0f, 0.0f}, (vector3){0.5f, 0.5f, 0.5f}, 1.0f);
        world.bodies[cube].restitution = 0.0f;
        world.bodies[cube].friction_static = 1.0f;
        world.bodies[cube].friction_kinetic = 0.8f;
        rigidbody_wake(&world.bodies[cube]);

        /* Tilt floor by creating sloped static cube as floor */
        int floor = physics_world_add_cube(&world, (vector3){0.0f, -0.5f, 0.0f}, (vector3){10.0f, 0.5f, 10.0f}, 0.0f);
        world.bodies[floor].restitution = 0.0f;
        world.bodies[floor].friction_static = 1.0f;
        world.bodies[floor].friction_kinetic = 0.8f;

        /* Rotate floor to 30 degrees (tan 30 = 0.577, mu=1.0 should hold) */
        /* For now test horizontal floor with lateral push */
        world.bodies[cube].velocity = (vector3){0.5f, 0.0f, 0.0f};

        const float dt = 1.0f / 60.0f;
        float max_x = 0.0f;

        for (int t = 0; t < 600; t++) {
            physics_world_step(&world, dt);
            rigidbody *b = &world.bodies[cube];
            if (fabsf(b->position.x) > max_x) max_x = fabsf(b->position.x);
        }

        printf("[INFO] static_friction max_x_drift=%.6f\n", max_x);
        if (max_x > 0.001f) { printf("[FAIL] static friction failed to hold\n"); fail = 1; }
        else { printf("[PASS] static friction holds against 0.5 m/s push\n"); }
        physics_world_cleanup(&world);
    }

    /* Test 3: Friction stopping distance - v^2/(2*mu*g) */
    {
        physics_world world;
        physics_world_init(&world);
        constraint_pool_init(&world);

        g_cfg.world.gravity = -9.81f;
        g_cfg.world.drag = 1.0f;

        int cube = physics_world_add_cube(&world, (vector3){0.0f, 0.5f, 0.0f}, (vector3){0.5f, 0.5f, 0.5f}, 1.0f);
        world.bodies[cube].restitution = 0.0f;
        world.bodies[cube].friction_static = 0.5f;
        world.bodies[cube].friction_kinetic = 0.4f;
        world.bodies[cube].velocity = (vector3){2.0f, 0.0f, 0.0f};
        rigidbody_wake(&world.bodies[cube]);

        const float dt = 1.0f / 60.0f;
        float x_start = 0.0f, x_end = 0.0f;
        int stopped = 0;

        for (int t = 0; t < 600; t++) {
            physics_world_step(&world, dt);
            rigidbody *b = &world.bodies[cube];
            if (!stopped && vector3_length(b->velocity) < 0.01f) {
                stopped = 1;
                x_end = b->position.x;
            }
        }

        float expected = 2.0f * 2.0f / (2.0f * 0.4f * 9.81f);
        float actual = fabsf(x_end - x_start);
        float err = fabsf(actual - expected) / expected;

        printf("[INFO] friction_stop expected=%.4f actual=%.4f err=%.2f%%\n", expected, actual, err*100);
        if (err > 0.05f) { printf("[FAIL] stopping distance error %.2f%%\n", err*100); fail = 1; }
        else { printf("[PASS] Coulomb stopping distance correct\n"); }

        physics_world_cleanup(&world);
    }

    /* Test 4: Split impulse - no velocity change from penetration correction */
    {
        physics_world world;
        physics_world_init(&world);
        constraint_pool_init(&world);

        g_cfg.world.gravity = -9.81f;
        g_cfg.world.drag = 1.0f;

        int cube = physics_world_add_cube(&world, (vector3){0.0f, 1.0f, 0.0f}, (vector3){0.5f, 0.5f, 0.5f}, 1.0f);
        world.bodies[cube].restitution = 0.0f;
        world.bodies[cube].friction_static = 0.0f;
        world.bodies[cube].friction_kinetic = 0.0f;
        world.bodies[cube].velocity = (vector3){0.0f, -5.0f, 0.0f}; /* slam into floor */
        rigidbody_wake(&world.bodies[cube]);

        const float dt = 1.0f / 60.0f;
        float max_vy = 0.0f;

        for (int t = 0; t < 100; t++) {
            physics_world_step(&world, dt);
            rigidbody *b = &world.bodies[cube];
            if (b->velocity.y > max_vy) max_vy = b->velocity.y;
        }

        printf("[INFO] split_impulse max_vy_after_impact=%.4f (expected 0)\n", max_vy);
        if (max_vy > 0.01f) { printf("[FAIL] split impulse created upward velocity\n"); fail = 1; }
        else { printf("[PASS] split impulse produces no velocity change\n"); }

        physics_world_cleanup(&world);
    }

    /* Test 5: Rolling resistance - sphere on horizontal plane should decelerate */
    {
        physics_world world;
        physics_world_init(&world);
        constraint_pool_init(&world);

        g_cfg.world.gravity = -9.81f;
        g_cfg.world.drag = 1.0f;
        g_cfg.world.rolling_resistance_coeff = 0.01f;

        int s = physics_world_add_sphere(&world, 0.5f, 1.0f, (vector3){0.0f, 0.5f, 0.0f});
        world.bodies[s].velocity = (vector3){2.0f, 0.0f, 0.0f};
        world.bodies[s].angular_velocity = (vector3){0.0f, 0.0f, 0.0f};
        rigidbody_wake(&world.bodies[s]);

        const float dt = 1.0f / 60.0f;
        float x_start = 0.0f, x_end = 0.0f;
        int stopped = 0;

        for (int t = 0; t < 6000; t++) {
            physics_world_step(&world, dt);
            rigidbody *b = &world.bodies[s];
            if (!stopped && vector3_length(b->velocity) < 0.01f) {
                stopped = 1;
                x_end = b->position.x;
            }
        }

        float dist = fabsf(x_end - x_start);
        printf("[INFO] rolling_resistance rolled %.2f m\n", dist);
        if (dist < 1.0f || dist > 50.0f) { printf("[FAIL] rolling resistance distance %.2f m unrealistic\n", dist); fail = 1; }
        else { printf("[PASS] rolling resistance deceleration plausible\n"); }

        physics_world_cleanup(&world);
    }

    return fail;
}
#endif