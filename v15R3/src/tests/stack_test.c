/* Stack-stability truth: a 6-cube tower must stand 10 s with bounded drift
 * and level top (sequential-impulse convergence + unbiased friction). */
#ifdef MPE_STACK_TEST
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

    const float h = 0.4f;
    for (int i = 0; i < 6; i++) {
        physics_world_add_cube(&world, (vector3){0.0f, h + (float)i * 2.0f * h, 0.0f}, (vector3){h, h, h}, 1.0f);
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
    float top_drift = sqrtf(world.bodies[5].position.x * world.bodies[5].position.x +
                            world.bodies[5].position.z * world.bodies[5].position.z);
    printf("[info] top drift=%.4f (limit 0.15)\n", top_drift);
    if (top_drift > 0.15f) {
        printf("[FAIL] tower leans/falls\n");
        fail = 1;
    } else {
        printf("[PASS] tower stands\n");
    }
    for (int i = 0; i < 6; i++) {
        float y_e = h + (float)i * 2.0f * h;
        if (fabsf(world.bodies[i].position.y - y_e) > 0.12f) {
            printf("[FAIL] level %d sank/floated (y=%.4f)\n", i, world.bodies[i].position.y);
            fail = 1;
        }
    }
    if (fail == 0) {
        printf("[PASS] all levels hold height\n");
    }
    physics_world_cleanup(&world);
    return fail;
}
#endif /* MPE_STACK_TEST */
