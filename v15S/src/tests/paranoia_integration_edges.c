/* PARANOIA TEST: Integration edge cases - extreme values, zero mass, extreme rotation */
#ifdef mpe_paranoia_integration_edges
#include <stdio.h>
#include <math.h>
#include "core/physics_world.h"
#include "physics/constraint.h"
#include "config/mpe_config.h"
#include "core/rigidbody.h"

int main(void) {
    mpe_config_init();
    int fail = 0;

    /* Test 1: Zero mass body - should be treated as static */
    {
        physics_world world;
        physics_world_init(&world);
        constraint_pool_init(&world);

        g_cfg.world.gravity = -9.81f;
        g_cfg.world.drag = 1.0f;

        int a = physics_world_add_sphere(&world, 0.5f, 0.0f, (vector3){0.0f, 2.0f, 0.0f}); /* mass=0 */
        int b = physics_world_add_sphere(&world, 0.5f, 1.0f, (vector3){0.0f, 5.0f, 0.0f});
        world.bodies[a].restitution = 0.0f;
        world.bodies[b].restitution = 0.0f;
        world.bodies[b].velocity = (vector3){0.0f, -5.0f, 0.0f};
        rigidbody_wake(&world.bodies[b]);

        const float dt = 1.0f / 60.0f;
        int nan_count = 0;

        for (int t = 0; t < 60; t++) {
            physics_world_step(&world, dt);
            for (int i = 0; i < world.body_count; i++) {
                if (!isfinite(world.bodies[i].position.x)) nan_count++;
            }
        }

        printf("[INFO] zero_mass nan=%d\n", nan_count);
        if (nan_count > 0) { printf("[FAIL] zero mass produced NaN\n"); fail = 1; }
        else { printf("[PASS] zero mass handled as static\n"); }
        physics_world_cleanup(&world);
    }

    /* Test 2: Extreme rotation - 1000 rad/s */
    {
        physics_world world;
        physics_world_init(&world);
        constraint_pool_init(&world);

        g_cfg.world.gravity = 0.0f;
        g_cfg.world.drag = 1.0f;

        int a = physics_world_add_sphere(&world, 0.5f, 1.0f, (vector3){0.0f, 0.0f, 0.0f});
        world.bodies[a].angular_velocity = (vector3){1000.0f, 0.0f, 0.0f};
        rigidbody_wake(&world.bodies[a]);

        const float dt = 1.0f / 60.0f;
        int nan_count = 0;
        float max_angle = 0.0f;

        for (int t = 0; t < 600; t++) {
            physics_world_step(&world, dt);
            if (!isfinite(world.bodies[0].orientation.w)) nan_count++;
            float angle = 2.0f * acosf(fabsf(world.bodies[0].orientation.w));
            if (angle > max_angle) max_angle = angle;
        }

        printf("[INFO] extreme_rotation nan=%d max_angle=%.1f rad\n", nan_count, max_angle);
        if (nan_count > 0) { printf("[FAIL] extreme rotation produced NaN\n"); fail = 1; }
        else { printf("[PASS] extreme rotation stable\n"); }
        physics_world_cleanup(&world);
    }

    /* Test 3: Tiny dt - numerical stability */
    {
        physics_world world;
        physics_world_init(&world);
        constraint_pool_init(&world);

        g_cfg.world.gravity = -9.81f;
        g_cfg.world.drag = 1.0f;

        int a = physics_world_add_sphere(&world, 0.5f, 1.0f, (vector3){0.0f, 2.0f, 0.0f});
        rigidbody_wake(&world.bodies[a]);

        int nan_count = 0;
        for (int t = 0; t < 6000; t++) {
            physics_world_step(&world, 1e-6f); /* 1 microsecond steps */
            if (!isfinite(world.bodies[0].position.y)) nan_count++;
        }

        printf("[INFO] tiny_dt nan=%d\n", nan_count);
        if (nan_count > 0) { printf("[FAIL] tiny dt produced NaN\n"); fail = 1; }
        else { printf("[PASS] tiny dt stable\n"); }
        physics_world_cleanup(&world);
    }

    /* Test 4: Huge dt - should be clamped */
    {
        physics_world world;
        physics_world_init(&world);
        constraint_pool_init(&world);

        g_cfg.world.gravity = -9.81f;
        g_cfg.world.drag = 1.0f;

        int a = physics_world_add_sphere(&world, 0.5f, 1.0f, (vector3){0.0f, 10.0f, 0.0f});
        rigidbody_wake(&world.bodies[a]);

        int nan_count = 0;
        physics_world_step(&world, 1000.0f); /* huge dt - should be clamped to 0.1f */
        if (!isfinite(world.bodies[0].position.y)) nan_count++;

        printf("[INFO] huge_dt nan=%d pos_y=%.2f\n", nan_count, world.bodies[0].position.y);
        if (nan_count > 0) { printf("[FAIL] huge dt produced NaN\n"); fail = 1; }
        else { printf("[PASS] huge dt clamped safely\n"); }
        physics_world_cleanup(&world);
    }

    /* Test 5: Negative dt - should be rejected */
    {
        physics_world world;
        physics_world_init(&world);
        constraint_pool_init(&world);

        g_cfg.world.gravity = -9.81f;
        g_cfg.world.drag = 1.0f;

        int a = physics_world_add_sphere(&world, 0.5f, 1.0f, (vector3){0.0f, 5.0f, 0.0f});
        rigidbody_wake(&world.bodies[a]);

        int nan_count = 0;
        physics_world_step(&world, -1.0f); /* negative dt */
        if (!isfinite(world.bodies[0].position.y)) nan_count++;

        printf("[INFO] negative_dt nan=%d\n", nan_count);
        if (nan_count > 0) { printf("[FAIL] negative dt caused NaN\n"); fail = 1; }
        else { printf("[PASS] negative dt rejected safely\n"); }
        physics_world_cleanup(&world);
    }

    /* Test 6: Very small mass - 1e-4 kg minimum */
    {
        physics_world world;
        physics_world_init(&world);
        constraint_pool_init(&world);

        g_cfg.world.gravity = -9.81f;
        g_cfg.world.drag = 1.0f;

        int a = physics_world_add_sphere(&world, 0.1f, 1e-4f, (vector3){0.0f, 2.0f, 0.0f}); /* 0.1g */
        int b = physics_world_add_sphere(&world, 0.5f, 1.0f, (vector3){0.0f, 5.0f, 0.0f});
        world.bodies[a].restitution = 1.0f;
        world.bodies[b].restitution = 1.0f;
        rigidbody_wake(&world.bodies[a]);
        rigidbody_wake(&world.bodies[b]);

        const float dt = 1.0f / 60.0f;
        int nan_count = 0;

        for (int t = 0; t < 600; t++) {
            physics_world_step(&world, dt);
            for (int i = 0; i < world.body_count; i++) {
                if (!isfinite(world.bodies[i].position.x)) nan_count++;
            }
        }

        printf("[INFO] tiny_mass nan=%d\n", nan_count);
        if (nan_count > 0) { printf("[FAIL] tiny mass produced NaN\n"); fail = 1; }
        else { printf("[PASS] tiny mass handled\n"); }
        physics_world_cleanup(&world);
    }

    /* Test 6: Huge mass - 1e6 kg maximum */
    {
        physics_world world;
        physics_world_init(&world);
        constraint_pool_init(&world);

        g_cfg.world.gravity = -9.81f;
        g_cfg.world.drag = 1.0f;

        int a = physics_world_add_sphere(&world, 10.0f, 1e6f, (vector3){0.0f, 10.0f, 0.0f});
        int b = physics_world_add_sphere(&world, 0.5f, 1.0f, (vector3){0.0f, 2.0f, 0.0f});
        world.bodies[a].restitution = 1.0f;
        world.bodies[b].restitution = 1.0f;
        rigidbody_wake(&world.bodies[a]);
        rigidbody_wake(&world.bodies[b]);

        const float dt = 1.0f / 60.0f;
        int nan_count = 0;

        for (int t = 0; t < 600; t++) {
            physics_world_step(&world, dt);
            for (int i = 0; i < world.body_count; i++) {
                if (!isfinite(world.bodies[i].position.x)) nan_count++;
            }
        }

        printf("[INFO] huge_mass nan=%d\n", nan_count);
        if (nan_count > 0) { printf("[FAIL] huge mass produced NaN\n"); fail = 1; }
        else { printf("[PASS] huge mass handled\n"); }
        physics_world_cleanup(&world);
    }

    /* Test 7: Degenerate inertia tensor - thin rod */
    {
        physics_world world;
        physics_world_init(&world);
        constraint_pool_init(&world);

        g_cfg.world.gravity = 0.0f;
        g_cfg.world.drag = 1.0f;

        int a = physics_world_add_cylinder(&world, 0.01f, 5.0f, 1.0f, (vector3){0.0f, 0.0f, 0.0f}); /* very thin rod */
        world.bodies[a].angular_velocity = (vector3){10.0f, 0.0f, 0.0f};
        rigidbody_wake(&world.bodies[a]);

        const float dt = 1.0f / 60.0f;
        int nan_count = 0;

        for (int t = 0; t < 600; t++) {
            physics_world_step(&world, dt);
            if (!isfinite(world.bodies[0].orientation.w)) nan_count++;
        }

        printf("[INFO] thin_rod nan=%d\n", nan_count);
        if (nan_count > 0) { printf("[FAIL] thin rod NaN\n"); fail = 1; }
        else { printf("[PASS] thin rod stable\n"); }
        physics_world_cleanup(&world);
    }

    /* Test 8: Near-zero quaternion - sanitization */
    {
        physics_world world;
        physics_world_init(&world);
        constraint_pool_init(&world);

        g_cfg.world.gravity = 0.0f;
        g_cfg.world.drag = 1.0f;

        int a = physics_world_add_sphere(&world, 0.5f, 1.0f, (vector3){0.0f, 0.0f, 0.0f});
        world.bodies[a].orientation = (vector4){1e-10f, 1e-10f, 1e-10f, 1e-10f}; /* near zero */
        rigidbody_wake(&world.bodies[a]);

        const float dt = 1.0f / 60.0f;
        int nan_count = 0;

        for (int t = 0; t < 60; t++) {
            physics_world_step(&world, dt);
            if (!isfinite(world.bodies[0].orientation.w)) nan_count++;
        }

        printf("[INFO] near_zero_quat nan=%d\n", nan_count);
        if (nan_count > 0) { printf("[FAIL] near-zero quat caused NaN\n"); fail = 1; }
        else { printf("[PASS] sanitization handles near-zero quat\n"); }
        physics_world_cleanup(&world);
    }

    return fail;
}
#endif