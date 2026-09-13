/* Bounce-series truth: successive apexes must decay geometrically as e^2
 * (Poisson restitution per impact). e=0.6 from 3.5 m: apexes ~1.76, ~0.95. */
#ifdef MPE_BOUNCE_SERIES_TEST
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

    int s = physics_world_add_sphere(&world, 0.5f, 1.0f, (vector3){0.0f, 4.0f, 0.0f});
    world.bodies[s].restitution = 0.6f;
    rigidbody_wake(&world.bodies[s]);

    const float dt = 1.0f / 60.0f;
    /* Impact/apogee timing: fall 3.5 m (0.845 s), rise v=0.6*8.29 (0.51 s),
     * fall 1.26 m (0.51 s), rise v=2.98 (0.30 s). */
    float apex1 = 0.0f, apex2 = 0.0f;
    for (int t = 0; t < 300; t++) {
        physics_world_step(&world, dt);
        float y = world.bodies[s].position.y;
        if (!isfinite(y)) {
            printf("[FAIL] NaN\n");
            physics_world_cleanup(&world);
            return 1;
        }
        float time = (float)(t + 1) * dt;
        if ((time > 1.0f) && (time < 1.7f) && (y > apex1)) {
            apex1 = y;
        }
        if ((time > 1.9f) && (time < 2.6f) && (y > apex2)) {
            apex2 = y;
        }
    }
    float e1 = 0.5f + 3.5f * 0.36f;  /* 1.76 */
    float e2 = 0.5f + (e1 - 0.5f) * 0.36f; /* 0.954 */
    printf("[info] apex1=%.4f (expect %.4f) apex2=%.4f (expect %.4f)\n", apex1, e1, apex2, e2);
    int fail = 0;
    if (fabsf(apex1 - e1) / e1 > 0.12f) {
        printf("[FAIL] first bounce height off\n");
        fail = 1;
    } else {
        printf("[PASS] first bounce follows e^2\n");
    }
    if (fabsf(apex2 - e2) / e2 > 0.15f) {
        printf("[FAIL] second bounce height off\n");
        fail = 1;
    } else {
        printf("[PASS] bounce series decays geometrically\n");
    }
    physics_world_cleanup(&world);
    return fail;
}
#endif /* MPE_BOUNCE_SERIES_TEST */
