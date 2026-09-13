/* Momentum truth: head-on elastic collision of equal masses must exchange
 * velocities exactly (conservation of linear momentum + e=1). */
#ifdef mpe_momentum_test
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

    /* High above the floor: no gravity-torque/contact interference. */
    int a = physics_world_add_sphere(&world, 0.5f, 1.0f, (vector3){-3.0f, 20.0f, 0.0f});
    int b = physics_world_add_sphere(&world, 0.5f, 1.0f, (vector3){0.0f, 20.0f, 0.0f});
    world.bodies[a].velocity = (vector3){3.0f, 0.0f, 0.0f};
    world.bodies[a].restitution = 1.0f;
    world.bodies[b].restitution = 1.0f;
    rigidbody_wake(&world.bodies[a]);

    const float dt = 1.0f / 60.0f;
    float p0 = 1.0f * 3.0f; /* total momentum, x */
    for (int t = 0; t < 240; t++) {
        physics_world_step(&world, dt);
        for (int i = 0; i < world.body_count; i++) {
            if (!isfinite(world.bodies[i].position.x)) {
                printf("[FAIL] NaN\n");
                physics_world_cleanup(&world);
                return 1;
            }
        }
        /* Stop once cleanly separated after the hit. */
        if ((t > 60) && ((world.bodies[b].position.x - world.bodies[a].position.x) > 2.0f) &&
            (world.bodies[a].velocity.x < world.bodies[b].velocity.x)) {
            break;
        }
    }
    float va = world.bodies[a].velocity.x;
    float vb = world.bodies[b].velocity.x;
    float p1 = va + vb;
    printf("[info] post-hit va=%.4f vb=%.4f (expect 0 / 3)\n", va, vb);
    int fail = 0;
    if (fabsf(va) > 0.3f) {
        printf("[FAIL] striker did not stop (va=%.4f)\n", va);
        fail = 1;
    } else {
        printf("[PASS] striker stops dead\n");
    }
    if (fabsf(vb - 3.0f) > 0.3f) {
        printf("[FAIL] target did not inherit velocity (vb=%.4f)\n", vb);
        fail = 1;
    } else {
        printf("[PASS] target inherits velocity\n");
    }
    if (fabsf(p1 - p0) > 0.3f) {
        printf("[FAIL] momentum not conserved (%.4f vs %.4f)\n", p1, p0);
        fail = 1;
    } else {
        printf("[PASS] total momentum conserved\n");
    }
    physics_world_cleanup(&world);
    return fail;
}
#endif /* mpe_momentum_test */
