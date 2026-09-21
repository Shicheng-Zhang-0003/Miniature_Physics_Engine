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
    /* TRUTH: gravity-free collision (y=20 fall couples floor/boundary/CCD
     * into a momentum measurement; the old 4s fall dropped 78m). Zero-g
     * isolates exchange physics. */
    g_cfg.world.gravity = 0.0f;
    g_cfg.sleep.enable = 0;
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
    /* TRUTH: 10% bands proved e=0.9 as e=1 (2.85 passes as 3). Bands at
     * projectile-grade 5% + KE conservation (elastic exchange preserves
     * va^2+vb^2=9) + transverse silence (no spurious y/z). */
    int fail = 0;
    if (fabsf(va) > 0.05f) {
        printf("[FAIL] striker did not stop (va=%.4f)\n", va);
        fail = 1;
    } else {
        printf("[PASS] striker stops dead\n");
    }
    if (fabsf(vb - 3.0f) > 0.05f) {
        printf("[FAIL] target did not inherit velocity (vb=%.4f)\n", vb);
        fail = 1;
    } else {
        printf("[PASS] target inherits velocity\n");
    }
    if (fabsf(p1 - p0) > 0.05f) {
        printf("[FAIL] momentum not conserved (%.4f vs %.4f)\n", p1, p0);
        fail = 1;
    } else {
        printf("[PASS] total momentum conserved\n");
    }
    {
        float ke1 = va * va + vb * vb;
        if (fabsf(ke1 - 9.0f) > 0.3f) {
            printf("[FAIL] kinetic energy not conserved (%.4f vs 9.0)\n", ke1);
            fail = 1;
        }
        float tmax = 0.0f;
        for (int i = 0; i < world.body_count; i++) {
            float ty = fabsf(world.bodies[i].velocity.y);
            float tz = fabsf(world.bodies[i].velocity.z);
            if (ty > tmax) tmax = ty;
            if (tz > tmax) tmax = tz;
        }
        if (tmax > 0.05f) {
            printf("[FAIL] spurious transverse motion (%.4f)\n", tmax);
            fail = 1;
        }
    }
    physics_world_cleanup(&world);
    return fail;
}
#endif /* mpe_momentum_test */
