/* PARANOIA TEST: Continuous Collision Detection - no tunneling at any speed */
#ifdef mpe_paranoia_ccd
#include <stdio.h>
#include <math.h>
#include "core/physics_world.h"
#include "physics/constraint.h"
#include "config/mpe_config.h"

int main(void) {
    mpe_config_init();
    int fail = 0;

    /* Test 1: Sphere at 144 m/s - must not tunnel through thin wall */
    {
        physics_world world;
        physics_world_init(&world);
        constraint_pool_init(&world);

        g_cfg.world.gravity = 0.0f;
        g_cfg.world.drag = 1.0f;

        /* Thin wall: 0.05m thick */
        int wall = physics_world_add_cube(&world, (vector3){0.0f, 0.0f, 0.0f}, (vector3){0.025f, 5.0f, 5.0f}, 0.0f);
        world.bodies[wall].restitution = 0.0f;

        int sphere = physics_world_add_sphere(&world, 0.1f, 1.0f, (vector3){-2.0f, 0.0f, 0.0f});
        world.bodies[sphere].velocity = (vector3){144.0f, 0.0f, 0.0f};
        world.bodies[sphere].restitution = 0.0f;
        rigidbody_wake(&world.bodies[sphere]);

        const float dt = 1.0f / 60.0f;
        int tunneled = 0;

        for (int t = 0; t < 60; t++) {
            physics_world_step(&world, dt);
            if (world.bodies[sphere].position.x > 0.025f) {
                tunneled = 1;
                break;
            }
        }

        printf("[INFO] ccd_144mps_tunnel=%d\n", tunneled);
        if (tunneled) { printf("[FAIL] CCD failed at 144 m/s\n"); fail = 1; }
        else { printf("[PASS] CCD stops 144 m/s sphere\n"); }
        physics_world_cleanup(&world);
    }

    /* Test 2: Sphere at 60 m/s onto floor - must land exactly */
    {
        physics_world world;
        physics_world_init(&world);
        constraint_pool_init(&world);

        g_cfg.world.gravity = -9.81f;
        g_cfg.world.drag = 1.0f;

        int floor = physics_world_add_cube(&world, (vector3){0.0f, -0.5f, 0.0f}, (vector3){10.0f, 0.5f, 10.0f}, 0.0f);
        int sphere = physics_world_add_sphere(&world, 0.5f, 1.0f, (vector3){0.0f, 50.0f, 0.0f});
        world.bodies[sphere].velocity = (vector3){0.0f, -60.0f, 0.0f};
        world.bodies[sphere].restitution = 0.0f;
        rigidbody_wake(&world.bodies[sphere]);

        const float dt = 1.0f / 60.0f;
        int hit_floor = 0;
        float min_y = 100.0f;

        for (int t = 0; t < 200; t++) {
            physics_world_step(&world, dt);
            float y = world.bodies[1].position.y;
            if (y < min_y) min_y = y;
            if (y <= 0.55f) { hit_floor = 1; break; }
        }

        printf("[INFO] ccd_floor_60mps hit=%d min_y=%.4f\n", hit_floor, min_y);
        if (!hit_floor || min_y < 0.45f) { printf("[FAIL] CCD floor penetration\n"); fail = 1; }
        else { printf("[PASS] CCD floor collision exact\n"); }
        physics_world_cleanup(&world);
    }

    /* Test 3: Fast spinning cylinder - corner must not tunnel through floor */
    {
        physics_world world;
        physics_world_init(&world);
        constraint_pool_init(&world);

        g_cfg.world.gravity = -9.81f;
        g_cfg.world.drag = 1.0f;

        int floor = physics_world_add_cube(&world, (vector3){0.0f, -0.5f, 0.0f}, (vector3){5.0f, 0.5f, 5.0f}, 0.0f);

        int cyl = physics_world_add_cylinder(&world, 0.3f, 0.5f, 1.0f, (vector3){0.0f, 2.0f, 0.0f});
        world.bodies[cyl].angular_velocity = (vector3){0.0f, 0.0f, 100.0f}; /* fast spin */
        world.bodies[cyl].restitution = 0.0f;
        rigidbody_wake(&world.bodies[cyl]);

        const float dt = 1.0f / 60.0f;
        float min_y = 100.0f;

        for (int t = 0; t < 600; t++) {
            physics_world_step(&world, dt);
            if (world.bodies[cyl].position.y < min_y) min_y = world.bodies[cyl].position.y;
        }

        printf("[INFO] ccd_spin_cyl min_y=%.4f\n", min_y);
        if (min_y < 0.2f) { printf("[FAIL] spinning cylinder tunneled\n"); fail = 1; }
        else { printf("[PASS] CCD handles spinning cylinder\n"); }
        physics_world_cleanup(&world);
    }

    /* Test 4: Two fast spheres colliding head-on */
    {
        physics_world world;
        physics_world_init(&world);
        constraint_pool_init(&world);

        g_cfg.world.gravity = 0.0f;
        g_cfg.world.drag = 1.0f;

        int a = physics_world_add_sphere(&world, 0.5f, 1.0f, (vector3){-5.0f, 0.0f, 0.0f});
        int b = physics_world_add_sphere(&world, 0.5f, 1.0f, (vector3){5.0f, 0.0f, 0.0f});
        world.bodies[a].velocity = (vector3){80.0f, 0.0f, 0.0f};
        world.bodies[b].velocity = (vector3){-80.0f, 0.0f, 0.0f};
        world.bodies[a].restitution = 1.0f;
        world.bodies[b].restitution = 1.0f;
        rigidbody_wake(&world.bodies[a]);
        rigidbody_wake(&world.bodies[b]);

        const float dt = 1.0f / 60.0f;
        int collided = 0;

        for (int t = 0; t < 60; t++) {
            physics_world_step(&world, dt);
            float d = vector3_length(vector3_subtraction(world.bodies[0].position, world.bodies[1].position));
            if (d <= 1.0f) { collided = 1; break; }
        }

        printf("[INFO] ccd_fast_spheres collided=%d\n", collided);
        if (!collided) { printf("[FAIL] CCD missed fast sphere-sphere\n"); fail = 1; }
        else { printf("[PASS] CCD catches fast sphere-sphere\n"); }
        physics_world_cleanup(&world);
    }

    /* Test 5: CCD remainder integration - exact parabola after TOI */
    {
        physics_world world;
        physics_world_init(&world);
        constraint_pool_init(&world);

        g_cfg.world.gravity = -9.81f;
        g_cfg.world.drag = 1.0f;

        int sphere = physics_world_add_sphere(&world, 0.1f, 1.0f, (vector3){-1.0f, 10.0f, 0.0f});
        world.bodies[sphere].velocity = (vector3){20.0f, -30.0f, 0.0f};
        world.bodies[sphere].restitution = 0.0f;
        rigidbody_wake(&world.bodies[sphere]);

        const float dt = 1.0f / 60.0f;
        float max_height_err = 0.0f;

        for (int t = 0; t < 300; t++) {
            physics_world_step(&world, dt);
            rigidbody *b = &world.bodies[0];
            float texact = (t + 1) * dt;
            float y_exact = 10.0f - 30.0f * texact - 0.5f * 9.81f * texact * texact;
            float err = fabsf(b->position.y - y_exact);
            if (err > max_height_err) max_height_err = err;
        }

        printf("[INFO] ccd_remainder_parabola max_err=%.4f\n", max_height_err);
        if (max_height_err > 0.001f) { printf("[FAIL] CCD remainder not exact parabola %.4f\n", max_height_err); fail = 1; }
        else { printf("[PASS] CCD remainder integration exact\n"); }
        physics_world_cleanup(&world);
    }

    return fail;
}
#endif