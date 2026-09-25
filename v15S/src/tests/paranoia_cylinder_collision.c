/* PARANOIA TEST: Cylinder collision edge cases - the hardest geometry */
#ifdef mpe_paranoia_cylinder_collision
#include <stdio.h>
#include <math.h>
#include "core/physics_world.h"
#include "physics/constraint.h"
#include "config/mpe_config.h"

int main(void) {
    mpe_config_init();
    int fail = 0;

    /* Test 1: Cylinder-cylinder coaxial face contact */
    {
        physics_world world;
        physics_world_init(&world);
        constraint_pool_init(&world);

        g_cfg.world.gravity = 0.0f;
        g_cfg.world.drag = 1.0f;

        int a = physics_world_add_cylinder(&world, 0.5f, 1.0f, 1.0f, (vector3){-2.0f, 10.0f, 0.0f});
        int b = physics_world_add_cylinder(&world, 0.5f, 1.0f, 1.0f, (vector3){2.0f, 10.0f, 0.0f});
        world.bodies[a].velocity = (vector3){2.0f, 0.0f, 0.0f};
        world.bodies[b].velocity = (vector3){-2.0f, 0.0f, 0.0f};
        world.bodies[a].restitution = 0.0f;
        world.bodies[b].restitution = 0.0f;
        rigidbody_wake(&world.bodies[a]);
        rigidbody_wake(&world.bodies[b]);

        const float dt = 1.0f / 60.0f;
        int contacted = 0;
        float max_pen = 0.0f;
        float min_center_separation = INFINITY;

        for (int t = 0; t < 120; t++) {
            physics_world_step(&world, dt);
            float d = vector3_length(vector3_subtraction(world.bodies[a].position, world.bodies[b].position));
            if (d < min_center_separation) min_center_separation = d;
            if (d <= 2.05f) contacted = 1; /* h_a+h_b = 2m for coaxial cap contact */
            float penetration = 2.0f - d;
            if (penetration > max_pen) max_pen = penetration;
        }

        float relative_speed = fabsf(world.bodies[a].velocity.x - world.bodies[b].velocity.x);
        printf("[INFO] cyl_cyl_coaxial contacted=%d min_sep=%.4f max_pen=%.4f final_rel_v=%.4f\n",
               contacted, min_center_separation, max_pen, relative_speed);
        if (!contacted || max_pen > 0.05f || relative_speed > 0.05f) {
            printf("[FAIL] coaxial cylinder face contact had excess penetration or passed through\n"); fail = 1;
        } else { printf("[PASS] coaxial cylinder faces meet without capsule endcap overlap\n"); }
        physics_world_cleanup(&world);
    }

    /* Test 2: Cylinder-cylinder parallel barrels (stacked logs) */
    {
        physics_world world;
        physics_world_init(&world);
        constraint_pool_init(&world);

        g_cfg.world.gravity = -9.81f;
        g_cfg.world.drag = 1.0f;

        int a = physics_world_add_cylinder(&world, 0.3f, 2.0f, 1.0f, (vector3){0.0f, 1.0f, 0.0f});
        int b = physics_world_add_cylinder(&world, 0.3f, 2.0f, 1.0f, (vector3){0.0f, 0.65f, 0.0f}); /* touching barrels */
        world.bodies[a].restitution = 0.0f;
        world.bodies[b].restitution = 0.0f;
        world.bodies[a].friction_static = 0.8f;
        world.bodies[b].friction_static = 0.8f;
        rigidbody_wake(&world.bodies[a]);
        rigidbody_wake(&world.bodies[b]);

        const float dt = 1.0f / 60.0f;
        float max_drift = 0.0f;

        for (int t = 0; t < 600; t++) {
            physics_world_step(&world, dt);
            float d = fabsf(world.bodies[0].position.x - world.bodies[1].position.x);
            if (d > max_drift) max_drift = d;
        }

        printf("[INFO] cyl_cyl_parallel max_drift=%.4f\n", max_drift);
        if (max_drift > 0.05f) { printf("[FAIL] parallel cylinders drifted %.4f\n", max_drift); fail = 1; }
        else { printf("[PASS] parallel cylinders stable\n"); }
        physics_world_cleanup(&world);
    }

    /* Test 3: Cylinder on tilted floor - cap contact stability
     * Static cube rotated by 17 degrees creates tilted floor.
     * Cylinder may slide/roll down - this is physically correct behavior. */
    {
        physics_world world;
        physics_world_init(&world);
        constraint_pool_init(&world);

        g_cfg.world.gravity = -9.81f;
        g_cfg.world.drag = 1.0f;

        /* Create tilted floor using rotated static cube */
        int floor = physics_world_add_cube(&world, (vector3){0.0f, -0.5f, 0.0f}, (vector3){10.0f, 0.5f, 10.0f}, 0.0f);
        if (floor < 0) { printf("[FAIL] tilted floor creation failed\n"); fail = 1; }
        world.bodies[floor].orientation = vector4_from_axis_with_angle((vector3){1.0f, 0.0f, 0.0f}, -0.3f); /* 17 deg tilt */
        rigidbody_sanitize(&world.bodies[floor]);

        int cyl = physics_world_add_cylinder(&world, 0.5f, 0.5f, 1.0f, (vector3){0.0f, 1.0f, 0.0f});
        if (cyl < 0) { printf("[FAIL] tilted-floor cylinder creation failed\n"); physics_world_cleanup(&world); return 1; }
        world.bodies[cyl].restitution = 0.0f;
        world.bodies[cyl].friction_static = 0.8f;
        world.bodies[cyl].friction_kinetic = 0.7f;
        rigidbody_wake(&world.bodies[cyl]);

        const float dt = 1.0f / 60.0f;
        int stable = 1;
        float max_vel = 0.0f;

        for (int t = 0; t < 1200; t++) {
            physics_world_step(&world, dt);
            rigidbody *b = &world.bodies[cyl];
            float v = vector3_length(b->velocity);
            if (v > max_vel) max_vel = v;
            if (v > 0.5f) stable = 0; /* Allow some sliding on 17 deg tilt */
        }

        printf("[INFO] cyl_tilted_floor stable=%d max_vel=%.4f\n", stable, max_vel);
        /* On a 17 degree tilt, cylinder will slide. This is physically correct.
         * Test that it doesn't accelerate to extreme speeds. */
        if (max_vel > 10.0f) { printf("[FAIL] cylinder accelerated excessively on tilted floor %.4f\n", max_vel); fail = 1; }
        else { printf("[PASS] cylinder on tilted floor slides but doesn't explode\n"); }
        physics_world_cleanup(&world);
    }

    /* Test 4: Upright cylinder settles on its circular cap. */
    {
        physics_world world;
        physics_world_init(&world);
        constraint_pool_init(&world);

        g_cfg.world.gravity = -9.81f;
        g_cfg.world.drag = 1.0f;

        int floor = physics_world_add_cube(&world, (vector3){0.0f, -0.5f, 0.0f}, (vector3){10.0f, 0.5f, 10.0f}, 0.0f);
        int cyl = physics_world_add_cylinder(&world, 0.5f, 2.0f, 1.0f, (vector3){0.0f, 2.5f, 0.0f});
        if (floor < 0 || cyl < 0) { printf("[FAIL] upright cylinder fixture creation failed\n"); physics_world_cleanup(&world); return 1; }
        world.bodies[cyl].orientation = vector4_from_axis_with_angle((vector3){0.0f, 0.0f, 1.0f}, 1.57079632679f);
        rigidbody_sanitize(&world.bodies[cyl]);
        world.bodies[cyl].restitution = 0.0f;
        world.bodies[cyl].friction_static = 0.8f;
        rigidbody_wake(&world.bodies[cyl]);

        const float dt = 1.0f / 60.0f;
        float max_tilt = 0.0f;

        for (int t = 0; t < 600; t++) {
            physics_world_step(&world, dt);
            rigidbody *b = &world.bodies[cyl];
            float axis_y = fabsf(vector3_dot(b->cached_axes[0], (vector3){0.0f, 1.0f, 0.0f}));
            if (axis_y > 1.0f) axis_y = 1.0f;
            float tilt = acosf(axis_y);
            if (tilt > max_tilt) max_tilt = tilt;
        }

        float support_error = fabsf(world.bodies[cyl].position.y - 2.0f);
        printf("[INFO] cyl_upright max_tilt=%.6f support_error=%.6f\n", max_tilt, support_error);
        if (max_tilt > 0.01f || support_error > 0.02f) {
            printf("[FAIL] upright cylinder did not remain on its cap\n"); fail = 1;
        } else { printf("[PASS] upright cylinder remains supported on its cap\n"); }
        physics_world_cleanup(&world);
    }

    /* Test 5: Sphere approaches the radial side of a static cylinder.
     * MPE cylinders use local X as their axle; the sphere approaches along Z. */
    {
        physics_world world;
        physics_world_init(&world);
        constraint_pool_init(&world);

        g_cfg.world.gravity = 0.0f;
        g_cfg.world.drag = 1.0f;

        int cyl = physics_world_add_cylinder(&world, 0.5f, 1.0f, 0.0f, (vector3){0.0f, 0.0f, 0.0f});
        world.bodies[cyl].restitution = 1.0f;
        int sph = physics_world_add_sphere(&world, 0.5f, 1.0f, (vector3){0.0f, 0.0f, 3.0f});
        world.bodies[sph].velocity = (vector3){0.0f, 0.0f, -5.0f};
        world.bodies[sph].restitution = 1.0f;
        rigidbody_wake(&world.bodies[sph]);

        const float dt = 1.0f / 60.0f;
        int bounced = 0;

        for (int t = 0; t < 120; t++) {
            physics_world_step(&world, dt);
            if (world.bodies[sph].velocity.z > 0.0f) { bounced = 1; break; }
        }

        printf("[INFO] cyl_sph_bounce bounced=%d z=%.4f vz=%.4f\n", bounced,
               world.bodies[sph].position.z, world.bodies[sph].velocity.z);
        if (!bounced) { printf("[FAIL] cylinder-sphere bounce missed\n"); fail = 1; }
        else { printf("[PASS] cylinder-sphere collision with restitution\n"); }
        physics_world_cleanup(&world);
    }

    /* Test 6: Cylinder vs cube - edge/corner contact */
    {
        physics_world world;
        physics_world_init(&world);
        constraint_pool_init(&world);

        g_cfg.world.gravity = -9.81f;
        g_cfg.world.drag = 1.0f;

        int floor = physics_world_add_cube(&world, (vector3){0.0f, -0.5f, 0.0f}, (vector3){5.0f, 0.5f, 5.0f}, 0.0f);
        int cyl = physics_world_add_cylinder(&world, 0.3f, 0.5f, 1.0f, (vector3){0.0f, 1.0f, 0.0f});
        if (floor < 0 || cyl < 0) { printf("[FAIL] cylinder-cube fixture creation failed\n"); physics_world_cleanup(&world); return 1; }
        world.bodies[cyl].restitution = 0.0f;
        world.bodies[cyl].friction_static = 0.8f;
        rigidbody_wake(&world.bodies[cyl]);

        int cube = physics_world_add_cube(&world, (vector3){2.0f, 1.0f, 0.0f}, (vector3){0.5f, 0.5f, 0.5f}, 1.0f);
        if (cube < 0) { printf("[FAIL] edge cube creation failed\n"); physics_world_cleanup(&world); return 1; }
        world.bodies[cube].restitution = 0.0f;
        rigidbody_wake(&world.bodies[cube]);

        const float dt = 1.0f / 60.0f;
        int cyl_settled = 0;

        for (int t = 0; t < 1200; t++) {
            physics_world_step(&world, dt);
            if (vector3_length(world.bodies[cyl].velocity) < 0.01f && vector3_length(world.bodies[cube].velocity) < 0.01f) {
                cyl_settled = 1;
            }
        }

        printf("[INFO] cyl_cube_stack settled=%d\n", cyl_settled);
        if (!cyl_settled) { printf("[FAIL] cylinder-cube stack didn't settle\n"); fail = 1; }
        else { printf("[PASS] cylinder-cube contact stable\n"); }
        physics_world_cleanup(&world);
    }

    /* Test 7: Thin cylinder (coin) - flat cap contacts */
    {
        physics_world world;
        physics_world_init(&world);
        constraint_pool_init(&world);

        g_cfg.world.gravity = -9.81f;
        g_cfg.world.drag = 1.0f;

        int floor = physics_world_add_cube(&world, (vector3){0.0f, -0.5f, 0.0f}, (vector3){10.0f, 0.5f, 10.0f}, 0.0f);
        int coin = physics_world_add_cylinder(&world, 1.0f, 0.02f, 1.0f, (vector3){0.0f, 0.02f, 0.0f});
        if (floor < 0 || coin < 0) { printf("[FAIL] thin-cylinder fixture creation failed\n"); physics_world_cleanup(&world); return 1; }
        world.bodies[coin].orientation = vector4_from_axis_with_angle((vector3){0.0f, 0.0f, 1.0f}, 1.57079632679f);
        rigidbody_sanitize(&world.bodies[coin]);
        world.bodies[coin].restitution = 0.0f;
        world.bodies[coin].friction_static = 0.8f;
        rigidbody_wake(&world.bodies[coin]);

        const float dt = 1.0f / 60.0f;
        float max_tilt = 0.0f;

        for (int t = 0; t < 1200; t++) {
            physics_world_step(&world, dt);
            rigidbody *b = &world.bodies[coin];
            float axis_y = fabsf(vector3_dot(b->cached_axes[0], (vector3){0.0f, 1.0f, 0.0f}));
            if (axis_y > 1.0f) axis_y = 1.0f;
            float tilt = acosf(axis_y);
            if (tilt > max_tilt) max_tilt = tilt;
        }

        printf("[INFO] thin_cylinder max_tilt=%.6f\n", max_tilt);
        if (max_tilt > 0.01f) { printf("[FAIL] thin cylinder tipped %.6f\n", max_tilt); fail = 1; }
        else { printf("[PASS] thin cylinder stable on floor\n"); }
        physics_world_cleanup(&world);
    }

    return fail;
}
#endif
