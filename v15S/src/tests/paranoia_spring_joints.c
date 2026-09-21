/* PARANOIA TEST: Spring joints - Hooke energy bounded, no silent softening */
#ifdef mpe_paranoia_spring_joints
#include <stdio.h>
#include <math.h>
#include "core/physics_world.h"
#include "physics/constraint.h"
#include "physics/spring_joint.h"
#include "config/mpe_config.h"

int main(void) {
    mpe_config_init();
    int fail = 0;

    /* Test 1: Hooke energy bounded - spring + damper must not inject energy */
    {
        physics_world world;
        physics_world_init(&world);
        constraint_pool_init(&world);

        g_cfg.world.gravity = -9.81f;
        g_cfg.world.drag = 1.0f;

        int a = physics_world_add_sphere(&world, 0.2f, 1.0f, (vector3){0.0f, 2.0f, 0.0f});
        int b = physics_world_add_sphere(&world, 0.2f, 1.0f, (vector3){0.0f, 5.0f, 0.0f});
        world.bodies[a].restitution = 0.0f;
        world.bodies[b].restitution = 0.0f;
        int joint = add_joint(&world, world.bodies[a].object_id, world.bodies[b].object_id, 3.0f, 100.0f, 1.0f);

        const float dt = 1.0f / 60.0f;
        float E_max = 0.0f, E_min = 1e9f;

        for (int t = 0; t < 3600; t++) {
            physics_world_step(&world, dt);
            float Ea = rb_get_kinetic_energy(&world.bodies[0]);
            float Eb = rb_get_kinetic_energy(&world.bodies[1]);
            float L = vector3_length(vector3_subtraction(world.bodies[0].position, world.bodies[1].position));
            float E_spring = 0.5f * 100.0f * (L - 3.0f) * (L - 3.0f);
            float E = Ea + Eb + E_spring;
            if (E > E_max) E_max = E;
            if (E < E_min) E_min = E;
        }

        float rel_range = (E_max - E_min) / E_max;
        printf("[INFO] spring_energy range=%.6f\n", rel_range);
        if (rel_range > 0.02f) { printf("[FAIL] spring energy unbounded %.2f%%\n", rel_range*100); fail = 1; }
        else { printf("[PASS] spring energy bounded\n"); }
        physics_world_cleanup(&world);
    }

    /* Test 2: Courant stability guard - spring softened only when necessary, never silently */
    {
        physics_world world;
        physics_world_init(&world);
        constraint_pool_init(&world);

        g_cfg.world.gravity = 0.0f;
        g_cfg.world.drag = 1.0f;

        int a = physics_world_add_sphere(&world, 0.5f, 1.0f, (vector3){0.0f, 0.0f, 0.0f});
        int b = physics_world_add_sphere(&world, 0.5f, 1.0f, (vector3){1.0f, 0.0f, 0.0f});
        world.bodies[a].restitution = 0.0f;
        world.bodies[b].restitution = 0.0f;
        int joint = add_joint(&world, world.bodies[a].object_id, world.bodies[b].object_id, 1.0f, 10000.0f, 0.0f); /* extremely stiff */

        const float dt = 1.0f / 60.0f;
        int nan_count = 0;
        float max_vel = 0.0f;

        for (int t = 0; t < 600; t++) {
            physics_world_step(&world, dt);
            for (int i = 0; i < world.body_count; i++) {
                rigidbody *rb = &world.bodies[i];
                if (!isfinite(rb->position.x) || !isfinite(rb->velocity.x)) nan_count++;
                float v = vector3_length(rb->velocity);
                if (v > max_vel) max_vel = v;
            }
        }

        printf("[INFO] stiff_spring nan=%d max_vel=%.2f\n", nan_count, max_vel);
        if (nan_count > 0) { printf("[FAIL] stiff spring produced NaN\n"); fail = 1; }
        if (max_vel > 100.0f) { printf("[FAIL] stiff spring exploded\n"); fail = 1; }
        else { printf("[PASS] Courant guard prevents explosion\n"); }
        physics_world_cleanup(&world);
    }

    /* Test 3: Coincident bodies with L0 > 0 - maximal repulsion */
    {
        physics_world world;
        physics_world_init(&world);
        constraint_pool_init(&world);

        g_cfg.world.gravity = 0.0f;
        g_cfg.world.drag = 1.0f;

        int a = physics_world_add_sphere(&world, 0.5f, 1.0f, (vector3){0.0f, 0.0f, 0.0f});
        int b = physics_world_add_sphere(&world, 0.5f, 1.0f, (vector3){0.0f, 0.0f, 0.0f}); /* exact same position */
        int joint = add_joint(&world, world.bodies[0].object_id, world.bodies[1].object_id, 1.0f, 100.0f, 0.0f);

        const float dt = 1.0f / 60.0f;
        int nan_count = 0;

        for (int t = 0; t < 60; t++) {
            physics_world_step(&world, dt);
            for (int i = 0; i < world.body_count; i++) {
                if (!isfinite(world.bodies[i].position.x)) nan_count++;
            }
        }

        printf("[INFO] coincident_bodies nan=%d\n", nan_count);
        if (nan_count > 0) { printf("[FAIL] coincident bodies produced NaN\n"); fail = 1; }
        else { printf("[PASS] coincident bodies handled\n"); }
        physics_world_cleanup(&world);
    }

    /* Test 4: Spring removed mid-simulation - no dangling references */
    {
        physics_world world;
        physics_world_init(&world);
        constraint_pool_init(&world);

        g_cfg.world.gravity = -9.81f;
        g_cfg.world.drag = 1.0f;

        int a = physics_world_add_sphere(&world, 0.2f, 1.0f, (vector3){0.0f, 2.0f, 0.0f});
        int b = physics_world_add_sphere(&world, 0.2f, 1.0f, (vector3){0.0f, 5.0f, 0.0f});
        int joint = add_joint(&world, world.bodies[0].object_id, world.bodies[1].object_id, 3.0f, 100.0f, 1.0f);

        const float dt = 1.0f / 60.0f;
        for (int t = 0; t < 60; t++) physics_world_step(&world, dt);

        remove_joint(&world, joint);
        for (int t = 0; t < 60; t++) physics_world_step(&world, dt);

        int nan_count = 0;
        for (int i = 0; i < world.body_count; i++) {
            if (!isfinite(world.bodies[i].position.x)) nan_count++;
        }

        if (nan_count > 0) { printf("[FAIL] NaN after spring removal\n"); fail = 1; }
        else { printf("[PASS] spring removal clean\n"); }
        physics_world_cleanup(&world);
    }

    /* Test 5: Spring on sleeping body - should wake or skip correctly */
    {
        physics_world world;
        physics_world_init(&world);
        constraint_pool_init(&world);

        g_cfg.world.gravity = -9.81f;
        g_cfg.world.drag = 1.0f;

        int a = physics_world_add_sphere(&world, 0.5f, 1.0f, (vector3){0.0f, 0.5f, 0.0f});
        world.bodies[a].is_sleeping = true;
        world.bodies[a].sleep_timer = 1.0f;

        int b = physics_world_add_sphere(&world, 0.5f, 1.0f, (vector3){0.0f, 3.0f, 0.0f});
        int joint = add_joint(&world, world.bodies[0].object_id, world.bodies[1].object_id, 2.5f, 50.0f, 1.0f);

        const float dt = 1.0f / 60.0f;
        for (int t = 0; t < 600; t++) physics_world_step(&world, dt);

        /* Sleeping body should have been woken by spring force */
        if (world.bodies[0].is_sleeping) { printf("[FAIL] spring didn't wake sleeper\n"); fail = 1; }
        else { printf("[PASS] spring wakes sleeping bodies\n"); }
        physics_world_cleanup(&world);
    }

    return fail;
}
#endif