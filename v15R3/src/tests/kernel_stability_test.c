#ifdef MPE_KERNEL_STABILITY_TEST

#include <stdio.h>
#include <math.h>
#include <stdbool.h>

#include "core/physics_world.h"
#include "physics/constraint.h"
#include "config/mpe_config.h"
#include "tests/unit/test_framework.h"

static const float DT = 1.0f / 60.0f;

static bool world_is_finite(physics_world *world) {
    for (int i = 0; i < world->body_count; i++) {
        rigidbody *rb = &world->bodies[i];

        if (!isfinite(rb->position.x) || !isfinite(rb->position.y) || !isfinite(rb->position.z)) {
            return false;
        }
        if (!isfinite(rb->velocity.x) || !isfinite(rb->velocity.y) || !isfinite(rb->velocity.z)) {
            return false;
        }
        if (!isfinite(rb->angular_velocity.x) || !isfinite(rb->angular_velocity.y) ||
            !isfinite(rb->angular_velocity.z)) {
            return false;
        }
        if (!isfinite(rb->orientation.w) || !isfinite(rb->orientation.x) ||
            !isfinite(rb->orientation.y) || !isfinite(rb->orientation.z)) {
            return false;
        }
    }
    return true;
}

static bool world_above_floor(physics_world *world, float min_y) {
    for (int i = 0; i < world->body_count; i++) {
        if (world->bodies[i].position.y < min_y) {
            return false;
        }
    }
    return true;
}

static void configure_body(physics_world *world, int idx) {
    if ((idx < 0) || (idx >= world->body_count)) {
        return;
    }

    rigidbody *rb = &world->bodies[idx];
    rb->friction_static = 0.8f;
    rb->friction_kinetic = 0.7f;
    rb->restitution = 0.0f;
    rb->velocity = vector3_zero();
    rb->angular_velocity = vector3_zero();
}

static void test_stack_stability(void) {
    printf("--- stack stability ---\n");

    physics_world world;
    physics_world_init(&world);

    for (int i = 0; i < 10; i++) {
        float y = 0.5f + (float)i * 0.99f;
        int idx = physics_world_add_cube(&world,
                                         (vector3){20.0f, y, 0.0f},
                                         (vector3){0.5f, 0.5f, 0.5f},
                                         1.0f);
        configure_body(&world, idx);
    }

    bool ok = true;
    for (int t = 0; t < 600; t++) {
        physics_world_step(&world, DT);
        if (!world_is_finite(&world)) {
            ok = false;
            break;
        }
    }

    ASSERT_TRUE(ok, "stack remains finite over 600 ticks");
    ASSERT_TRUE(world_above_floor(&world, -0.5f), "stack does not fall through floor");

    physics_world_cleanup(&world);
}

static void test_sleep_isolation(void) {
    printf("--- sleep isolation ---\n");

    physics_world world;
    physics_world_init(&world);

    int idx = physics_world_add_cube(&world,
                                     (vector3){0.0f, 0.5f, 0.0f},
                                     (vector3){0.5f, 0.5f, 0.5f},
                                     1.0f);
    configure_body(&world, idx);

    world.bodies[idx].is_sleeping = true;
    world.bodies[idx].sleep_timer = 2.0f;

    vector3 position_before = world.bodies[idx].position;

    for (int t = 0; t < 120; t++) {
        physics_world_step(&world, DT);
    }

    ASSERT_TRUE(world.bodies[idx].is_sleeping, "isolated sleeping body stays sleeping");
    ASSERT_FLOAT_EQ(world.bodies[idx].position.x, position_before.x, 0.0001f,
                    "sleeping body does not drift x");
    ASSERT_FLOAT_EQ(world.bodies[idx].position.y, position_before.y, 0.0001f,
                    "sleeping body does not drift y");
    ASSERT_FLOAT_EQ(world.bodies[idx].position.z, position_before.z, 0.0001f,
                    "sleeping body does not drift z");

    physics_world_cleanup(&world);
}

static void test_stress_mixed_objects(void) {
    printf("--- mixed-object stress ---\n");

    physics_world world;
    physics_world_init(&world);

    for (int i = 0; i < 30; i++) {
        float x = -5.0f + (float)(i % 5) * 2.5f;
        float z = -5.0f + (float)((i / 5) % 5) * 2.5f;
        float y = 5.0f + (float)(i / 25) * 2.0f;

        int idx;
        if ((i % 2) == 0) {
            idx = physics_world_add_sphere(&world, 0.35f, 1.0f, (vector3){x, y, z});
        } else {
            idx = physics_world_add_cube(&world,
                                         (vector3){x, y, z},
                                         (vector3){0.4f, 0.4f, 0.4f},
                                         1.5f);
        }
        configure_body(&world, idx);
    }

    bool ok = true;
    for (int t = 0; t < 300; t++) {
        physics_world_step(&world, DT);
        if (!world_is_finite(&world)) {
            ok = false;
            break;
        }
    }

    ASSERT_TRUE(ok, "mixed stress scene remains finite over 300 ticks");
    ASSERT_TRUE(world_above_floor(&world, -2.0f), "mixed stress scene does not fall through floor");

    physics_world_cleanup(&world);
}

int main(void) {
    mpe_config_init();
    constraint_pool_init();

    printf("============================================\n");
    printf("MPE Kernel Integration Test: stability\n");
    printf("============================================\n");

    test_stack_stability();
    test_sleep_isolation();
    test_stress_mixed_objects();

    return mpe_unit_test_summary("kernel_stability_test");
}

#endif /* MPE_KERNEL_STABILITY_TEST */
