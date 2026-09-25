/* Stack-stability truth: a 6-cube tower must stand 10 s with bounded drift
 * and level top (sequential-impulse convergence + unbiased friction). */
#ifdef mpe_stack_test
#include <stdio.h>
#include <math.h>
#include "core/physics_world.h"
#include "physics/constraint.h"
#include "config/mpe_config.h"

int main(void) {
    mpe_config_init();
    physics_world world;
    physics_world_init(&world);
    constraint_pool_init(&world);

    /* A frictional slab is part of the fixture: the implicit world
     * backstop arrests downward motion but cannot support a stable tower. */
    int floor = physics_world_add_cube(&world, (vector3){0.0f, -0.5f, 0.0f},
                                       (vector3){10.0f, 0.5f, 10.0f}, 0.0f);
    if (floor < 0) {
        printf("[FAIL] could not create floor\n");
        physics_world_cleanup(&world);
        return 1;
    }
    world.bodies[floor].restitution = 0.0f;
    world.bodies[floor].friction_static = 0.8f;
    world.bodies[floor].friction_kinetic = 0.6f;

    const float h = 0.4f;
    int cubes[6];
    for (int i = 0; i < 6; i++) {
        cubes[i] = physics_world_add_cube(&world, (vector3){0.0f, h + (float)i * 2.0f * h, 0.0f},
                                          (vector3){h, h, h}, 1.0f);
        if (cubes[i] < 0) {
            printf("[FAIL] could not create cube %d\n", i);
            physics_world_cleanup(&world);
            return 1;
        }
        world.bodies[cubes[i]].restitution = 0.0f;
        world.bodies[cubes[i]].friction_static = 0.8f;
        world.bodies[cubes[i]].friction_kinetic = 0.6f;
    }
    const float dt = 1.0f / 60.0f;
    for (int t = 0; t < 600; t++) {
        physics_world_step(&world, dt);
        for (int i = 0; i < world.body_count; i++) {
            if (!isfinite(world.bodies[i].position.x)) {
                printf("[FAIL] NaN at tick %d\n", t);
                physics_world_cleanup(&world);
                return 1;
            }
        }
    }
    int fail = 0;
    rigidbody *top = &world.bodies[cubes[5]];
    float top_drift = sqrtf(top->position.x * top->position.x + top->position.z * top->position.z);
    printf("[info] top drift=%.4f (limit 0.05)\n", top_drift);
    if (top_drift > 0.05f) {
        printf("[FAIL] tower leans/falls\n");
        fail = 1;
    } else {
        printf("[PASS] tower stands\n");
    }
    for (int i = 0; i < 6; i++) {
        float y_e = h + (float)i * 2.0f * h;
        rigidbody *cube = &world.bodies[cubes[i]];
        if (fabsf(cube->position.y - y_e) > 0.03f) {
            printf("[FAIL] level %d sank/floated (y=%.4f)\n", i, cube->position.y);
            fail = 1;
        }
        /* TRUTH: settled tower must be still, not vibrating. Velocity gates
         * catch solver jitter the position band hides. */
        float lv = vector3_length(cube->velocity);
        float av = vector3_length(cube->angular_velocity);
        if (lv > 0.05f || av > 0.05f) {
            printf("[FAIL] level %d still moving (v=%.4f w=%.4f)\n", i, lv, av);
            fail = 1;
        }
    }
    if (fail == 0) {
        printf("[PASS] all levels hold height\n");
    }
    physics_world_cleanup(&world);
    return fail;
}
#endif /* mpe_stack_test */
