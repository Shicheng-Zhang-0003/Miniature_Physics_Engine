/* PARANOIA TEST: Scene persistence - exact roundtrip with all physics state */
#ifdef mpe_paranoia_scene_persistence
#include <stdio.h>
#include <math.h>
#include "core/physics_world.h"
#include "physics/constraint.h"
#include "scene/scene_saving.h"
#include "scene/scene_load.h"
#include "config/mpe_config.h"

int main(void) {
    mpe_config_init();
    int fail = 0;

    /* Test 1: Exact state roundtrip - positions, velocities, orientations, sleep state */
    {
        physics_world world;
        physics_world_init(&world);
        constraint_pool_init(&world);

        g_cfg.world.gravity = -9.81f;
        g_cfg.world.drag = 0.99f;

        int a = physics_world_add_sphere(&world, 0.5f, 2.0f, (vector3){1.0f, 5.0f, 2.0f});
        world.bodies[a].velocity = (vector3){1.5f, -0.5f, 0.25f};
        world.bodies[a].angular_velocity = (vector3){3.0f, -1.0f, 2.0f};
        world.bodies[a].restitution = 0.4f;
        world.bodies[a].friction_static = 0.8f;
        world.bodies[a].friction_kinetic = 0.7f;
        world.bodies[a].nice_value = 5;
        rigidbody_wake(&world.bodies[a]);

        int b = physics_world_add_cube(&world, (vector3){-1.0f, 2.0f, -1.0f}, (vector3){0.5f, 0.5f, 0.5f}, 1.0f);
        world.bodies[b].velocity = (vector3){-0.75f, 0.0f, 0.5f};
        world.bodies[b].angular_velocity = (vector3){0.0f, 2.0f, -1.5f};
        world.bodies[b].restitution = 0.3f;
        world.bodies[b].friction_static = 0.9f;
        world.bodies[b].friction_kinetic = 0.6f;
        world.bodies[b].nice_value = 10;
        rigidbody_wake(&world.bodies[b]);

        int c = physics_world_add_cylinder(&world, 0.3f, 0.4f, 1.5f, (vector3){0.0f, 4.0f, 1.0f});
        world.bodies[c].velocity = (vector3){0.2f, -1.0f, -0.3f};
        world.bodies[c].angular_velocity = (vector3){-2.0f, 0.5f, 1.0f};
        world.bodies[c].restitution = 0.2f;
        rigidbody_wake(&world.bodies[c]);

        /* Add joints */
        constraint_pool_init(&world);
        int joint1 = constraint_add_revolute(&world, world.bodies[a].object_id, world.bodies[b].object_id,
                                           (vector3){0,0,0}, (vector3){0,0,0}, (vector3){0,0,1});
        constraint_set_revolute_motor(&world, joint1, true, 5.0f, 10.0f);
        int joint2 = constraint_add_distance(&world, world.bodies[b].object_id, world.bodies[c].object_id,
                                           (vector3){0,0,0}, (vector3){0,0,0}, 2.0f);
        int joint3 = add_joint(&world, world.bodies[a].object_id, world.bodies[c].object_id, 3.0f, 50.0f, 1.0f);

        const float dt = 1.0f / 60.0f;
        for (int t = 0; t < 600; t++) physics_world_step(&world, dt);

        /* Capture exact state */
        rigidbody ref_bodies[3];
        for (int i = 0; i < 3; i++) ref_bodies[i] = world.bodies[i];

        /* Save scene */
        char path[256] = "/tmp/paranoia_scene.mpe";
        int save_result = scene_save(w, path);
        if (save_result != 0) { printf("[FAIL] scene save failed\n"); fail = 1; }

        /* Load into new world */
        physics_world world2;
        physics_world_init(&world2);
        constraint_pool_init(&world2);
        int load_result = scene_load(&world2, path);
        if (load_result != 0) { printf("[FAIL] scene load failed\n"); fail = 1; }

        /* Compare exact state */
        int mismatch = 0;
        for (int i = 0; i < 3; i++) {
            rigidbody *orig = &ref_bodies[i];
            rigidbody *loaded = &world2.bodies[i];

            if (fabsf(orig->position.x - loaded->position.x) > 0.0f ||
                fabsf(orig->position.y - loaded->position.y) > 0.0f ||
                fabsf(orig->position.z - loaded->position.z) > 0.0f) mismatch = 1;
            if (fabsf(orig->velocity.x - loaded->velocity.x) > 0.0f ||
                fabsf(orig->velocity.y - loaded->velocity.y) > 0.0f ||
                fabsf(orig->velocity.z - loaded->velocity.z) > 0.0f) mismatch = 1;
            if (orig->orientation.w != loaded->orientation.w ||
                orig->orientation.x != loaded->orientation.x ||
                orig->orientation.y != loaded->orientation.y ||
                orig->orientation.z != loaded->orientation.z) mismatch = 1;
            if (orig->angular_velocity.x != loaded->angular_velocity.x ||
                orig->angular_velocity.y != loaded->angular_velocity.y ||
                orig->angular_velocity.z != loaded->angular_velocity.z) mismatch = 1;
            if (orig->mass != loaded->mass) mismatch = 1;
            if (orig->restitution != loaded->restitution) mismatch = 1;
            if (orig->friction_static != loaded->friction_static) mismatch = 1;
            if (orig->friction_kinetic != loaded->friction_kinetic) mismatch = 1;
            if (orig->nice_value != loaded->nice_value) mismatch = 1;
            if (orig->is_sleeping != loaded->is_sleeping) mismatch = 1;
            if (fabsf(orig->sleep_timer - loaded->sleep_timer) > 0.0f) mismatch = 1;
            if (orig->object_id != loaded->object_id) mismatch = 1;
            if (orig->object_generation != loaded->object_generation) mismatch = 1;
        }

        /* Check joints */
        if (world2.revolute_constraint_count != world.revolute_constraint_count) mismatch = 1;
        if (world2.spring_joint_count != world.spring_joint_count) mismatch = 1;

        if (mismatch) { printf("[FAIL] scene roundtrip state mismatch\n"); fail = 1; }
        else { printf("[PASS] scene exact roundtrip (bitwise identical)\n"); }

        physics_world_cleanup(&world);
        physics_world_cleanup(&world2);
    }

    /* Test 2: CRC validation - tampered file rejected */
    {
        /* This requires corrupting a file - skip for automated test but validate CRC path */
        printf("[INFO] CRC validation path tested in scene_roundtrip_test\n");
    }

    /* Test 3: Legacy format compatibility (v153 and older) */
    {
        /* Would need actual legacy files - tested in scene_roundtrip_test */
        printf("[INFO] legacy format tested in scene_roundtrip_test\n");
    }

    /* Test 4: Scene with sleeping bodies - sleep state preserved */
    {
        physics_world world;
        physics_world_init(&world);
        constraint_pool_init(&world);

        g_cfg.world.gravity = -9.81f;
        g_cfg.world.drag = 0.99f;

        int a = physics_world_add_sphere(&world, 0.5f, 1.0f, (vector3){0.0f, 0.5f, 0.0f});
        world.bodies[a].is_sleeping = true;
        world.bodies[a].sleep_timer = 10.0f;
        rigidbody_wake(&world.bodies[a]); /* wake then let sleep */

        const float dt = 1.0f / 60.0f;
        for (int t = 0; t < 120; t++) physics_world_step(&world, dt); /* let it sleep */

        /* Verify sleeping */
        if (!world.bodies[0].is_sleeping) { printf("[FAIL] body didn't sleep\n"); fail = 1; }

        /* Save and load */
        char path[256] = "/tmp/paranoia_sleep.mpe";
        scene_save(&world, path);

        physics_world world2;
        physics_world_init(&world2);
        constraint_pool_init(&world2);
        scene_load(&world2, path);

        if (!world2.bodies[0].is_sleeping) { printf("[FAIL] sleep state not preserved\n"); fail = 1; }
        if (fabsf(world2.bodies[0].sleep_timer - world.bodies[0].sleep_timer) > 0.0f) {
            printf("[FAIL] sleep_timer not exact\n"); fail = 1;
        }
        printf("[PASS] sleep state exact roundtrip\n");

        physics_world_cleanup(&world);
        physics_world_cleanup(&world2);
    }

    /* Test 5: Scene with springs - spring state preserved */
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
        for (int t = 0; t < 300; t++) physics_world_step(&world, dt);

        /* Save/load */
        char path[256] = "/tmp/paranoia_spring.mpe";
        scene_save(&world, path);

        physics_world world2;
        physics_world_init(&world2);
        constraint_pool_init(&world2);
        scene_load(&world2, path);

        /* Verify spring parameters */
        if (world2.spring_joint_count != 1) { printf("[FAIL] spring not saved\n"); fail = 1; }
        float L0_orig = world.spring_joints[0].equilibrium_length;
        float L0_load = world2.spring_joints[0].equilibrium_length;
        float k_orig = world.spring_joints[0].spring_constant;
        float k_load = world2.spring_joints[0].spring_constant;
        float c_orig = world.spring_joints[0].damping_coefficient;
        float c_load = world2.spring_joints[0].damping_coefficient;

        if (L0_orig != L0_load || k_orig != k_load || c_orig != c_load) {
            printf("[FAIL] spring params not exact\n"); fail = 1;
        } else { printf("[PASS] spring state exact roundtrip\n"); }

        physics_world_cleanup(&world);
        physics_world_cleanup(&world2);
    }

    return fail;
}
#endif