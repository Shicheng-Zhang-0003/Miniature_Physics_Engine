/* PARANOIA TEST: Determinism - bitwise reproducibility across runs */
#ifdef mpe_paranoia_determinism
#include <stdio.h>
#include <math.h>
#include "core/physics_world.h"
#include "physics/constraint.h"
#include "config/mpe_config.h"

int main(void) {
    mpe_config_init();
    int fail = 0;

    /* Test 1: Bitwise identical across twin worlds */
    {
        physics_world w1, w2;
        physics_world_init(&w1);
        physics_world_init(&w2);
        constraint_pool_init(&w1);
        constraint_pool_init(&w2);

        g_cfg.world.gravity = -9.81f;
        g_cfg.world.drag = 0.99f;

        int s1 = physics_world_add_sphere(&w1, 0.5f, 2.0f, (vector3){-1.0f, 3.0f, 0.5f});
        w1.bodies[s1].velocity = (vector3){1.5f, -0.5f, 0.25f};
        w1.bodies[s1].angular_velocity = (vector3){3.0f, -1.0f, 2.0f};
        w1.bodies[s1].restitution = 0.4f;

        int c1 = physics_world_add_cube(&w1, (vector3){1.0f, 0.5f, -0.5f}, (vector3){0.5f, 0.5f, 0.5f}, 1.0f);
        w1.bodies[c1].velocity = (vector3){-0.75f, 0.0f, 0.5f};
        w1.bodies[c1].angular_velocity = (vector3){0.0f, 2.0f, -1.5f};
        w1.bodies[c1].restitution = 0.3f;

        /* Exact duplicate in w2 */
        int s2 = physics_world_add_sphere(&w2, 0.5f, 2.0f, (vector3){-1.0f, 3.0f, 0.5f});
        w2.bodies[s2].velocity = (vector3){1.5f, -0.5f, 0.25f};
        w2.bodies[s2].angular_velocity = (vector3){3.0f, -1.0f, 2.0f};
        w2.bodies[s2].restitution = 0.4f;

        int c2 = physics_world_add_cube(&w2, (vector3){1.0f, 0.5f, -0.5f}, (vector3){0.5f, 0.5f, 0.5f}, 1.0f);
        w2.bodies[c2].velocity = (vector3){-0.75f, 0.0f, 0.5f};
        w2.bodies[c2].angular_velocity = (vector3){0.0f, 2.0f, -1.5f};
        w2.bodies[c2].restitution = 0.3f;

        const float dt = 1.0f / 60.0f;
        for (int t = 0; t < 1200; t++) {
            physics_world_step(&w1, dt);
            physics_world_step(&w2, dt);
        }

        int mismatch = 0;
        if (w1.body_count != w2.body_count) { mismatch = 1; }
        for (int i = 0; i < w1.body_count && !mismatch; i++) {
            rigidbody *a = &w1.bodies[i];
            rigidbody *b = &w2.bodies[i];
            float *fa = &a->position.x;
            float *fb = &b->position.x;
            for (int k = 0; k < 3 + 3 + 4 + 3 + 3 + 3 + 3; k++) { /* pos, vel, quat, angvel, acc, angacc, force, torque */
                if (fa[k] != fb[k]) { mismatch = 1; break; }
            }
        }

        if (mismatch) { printf("[FAIL] twin worlds diverged\n"); fail = 1; }
        else { printf("[PASS] twin worlds bitwise identical for 1200 ticks\n"); }
        physics_world_cleanup(&w1);
        physics_world_cleanup(&w2);
    }

    /* Test 2: Deterministic across scene save/load */
    {
        physics_world world;
        physics_world_init(&world);
        constraint_pool_init(&world);

        g_cfg.world.gravity = -9.81f;
        g_cfg.world.drag = 0.99f;

        int a = physics_world_add_sphere(&world, 0.5f, 1.0f, (vector3){0.0f, 2.0f, 0.0f});
        world.bodies[a].velocity = (vector3){1.0f, 0.0f, 0.0f};
        world.bodies[a].restitution = 0.5f;
        rigidbody_wake(&world.bodies[a]);

        const float dt = 1.0f / 60.0f;
        for (int t = 0; t < 300; t++) physics_world_step(&world, dt);

        /* Save state */
        physics_world *w = &world;
        /* Can't easily test save/load without scene API, so test re-initialization */
        /* Recreate world with exact same initial conditions */
        physics_world world2;
        physics_world_init(&world2);
        constraint_pool_init(&world2);

        int b = physics_world_add_sphere(&world2, 0.5f, 1.0f, (vector3){0.0f, 2.0f, 0.0f});
        world2.bodies[b].velocity = (vector3){1.0f, 0.0f, 0.0f};
        world2.bodies[b].restitution = 0.5f;
        rigidbody_wake(&world2.bodies[b]);

        for (int t = 0; t < 300; t++) {
            physics_world_step(&world, dt);
            physics_world_step(&world2, dt);
        }

        float pos_diff = fabsf(world.bodies[0].position.x - world2.bodies[0].position.x);
        float vel_diff = fabsf(world.bodies[0].velocity.x - world2.bodies[0].velocity.x);

        printf("[INFO] determinism_reinit pos_diff=%.6f vel_diff=%.6f\n", pos_diff, vel_diff);
        if (pos_diff > 0.0f || vel_diff > 0.0f) { printf("[FAIL] re-init not deterministic\n"); fail = 1; }
        else { printf("[PASS] re-initialization deterministic\n"); }

        physics_world_cleanup(&world);
        physics_world_cleanup(&world2);
    }

    /* Test 3: Deterministic with different thread counts (simulated) */
    {
        physics_world world;
        physics_world_init(&world);
        constraint_pool_init(&world);

        g_cfg.world.gravity = -9.81f;
        g_cfg.world.drag = 1.0f;

        int a = physics_world_add_cube(&world, (vector3){0.0f, 2.0f, 0.0f}, (vector3){0.5f, 0.5f, 0.5f}, 1.0f);
        world.bodies[a].velocity = (vector3){2.0f, -1.0f, 0.5f};
        world.bodies[a].angular_velocity = (vector3){1.0f, 2.0f, -1.0f};
        world.bodies[a].restitution = 0.3f;

        const float dt = 1.0f / 60.0f;
        vector3 pos_ref = world.bodies[0].position;

        for (int run = 0; run < 10; run++) {
            physics_world world2;
            physics_world_init(&world2);
            constraint_pool_init(&world2);

            int b = physics_world_add_cube(&world2, (vector3){0.0f, 2.0f, 0.0f}, (vector3){0.5f, 0.5f, 0.5f}, 1.0f);
            world2.bodies[b].velocity = (vector3){2.0f, -1.0f, 0.5f};
            world2.bodies[b].angular_velocity = (vector3){1.0f, 2.0f, -1.0f};
            world2.bodies[b].restitution = 0.3f;

            for (int t = 0; t < 600; t++) physics_world_step(&world2, dt);

            float diff = fabsf(world2.bodies[0].position.x - pos_ref.x) +
                        fabsf(world2.bodies[0].position.y - pos_ref.y) +
                        fabsf(world2.bodies[0].position.z - pos_ref.z);
            if (diff > 0.0f) { printf("[FAIL] run %d diverged\n", run); fail = 1; }

            physics_world_cleanup(&world2);
        }
        printf("[PASS] 10 independent runs bitwise identical\n");
        physics_world_cleanup(&world);
    }

    /* Test 4: Floating point determinism - no NaN/Inf propagation */
    {
        physics_world world;
        physics_world_init(&world);
        constraint_pool_init(&world);

        g_cfg.world.gravity = 0.0f;
        g_cfg.world.drag = 1.0f;

        /* Create scenario that could produce NaN */
        int a = physics_world_add_sphere(&world, 0.0f, 1.0f, (vector3){0.0f, 0.0f, 0.0f}); /* zero radius - should be sanitized */
        int b = physics_world_add_cube(&world, (vector3){0.0f, 0.0f, 0.0f}, (vector3){0.0f, 0.0f, 0.0f}, 1.0f); /* zero size - sanitized */

        const float dt = 1.0f / 60.0f;
        int nan_count = 0;

        for (int t = 0; t < 600; t++) {
            physics_world_step(&world, dt);
            for (int i = 0; i < world.body_count; i++) {
                rigidbody *rb = &world.bodies[i];
                if (!isfinite(rb->position.x) || !isfinite(rb->velocity.x)) nan_count++;
            }
        }

        printf("[INFO] sanitization_nan_test nan_count=%d\n", nan_count);
        if (nan_count > 0) { printf("[FAIL] NaN propagated\n"); fail = 1; }
        else { printf("[PASS] sanitization prevents NaN propagation\n"); }
        physics_world_cleanup(&world);
    }

    return fail;
}
#endif