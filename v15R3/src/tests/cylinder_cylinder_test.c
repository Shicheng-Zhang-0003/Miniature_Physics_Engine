/* MFS_174: Cylinder vs cylinder collision test.
* Two cylinders approach each other. They must not pass
* through each other. */
#ifdef MFS_CYL_CYL_TEST
#include <stdio.h>
#include <math.h>
#include "core/physics_world.h"
#include "config/mpe_config.h"

int main(void) {
    mpe_config_init();
    physics_world world;
    physics_world_init(&world);

    /* Static floor */
    physics_world_add_cube(&world,
        (vector3){0.0f, -0.5f, 0.0f},
        (vector3){10.0f, 0.5f, 10.0f}, 0.0f);

    /* Two cylinders approaching along Z */
    int c1 = physics_world_add_cylinder(&world,
        0.05f, 0.02f, 0.5f,
        (vector3){0.0f, 0.06f, -0.3f});
    int c2 = physics_world_add_cylinder(&world,
        0.05f, 0.02f, 0.5f,
        (vector3){0.0f, 0.06f, 0.3f});

    world.bodies[c1].velocity = (vector3){0.0f, 0.0f,  2.0f};
    world.bodies[c2].velocity = (vector3){0.0f, 0.0f, -2.0f};

    const float dt = 1.0f / 60.0f;
    int fail = 0;
    for (int t = 0; t < 120 && !fail; t++) {
        physics_world_step(&world, dt);
        for (int i = 0; i < world.body_count; i++) {
            if (!isfinite(world.bodies[i].position.z)) {
                printf("[FAIL] NaN at tick %d\n", t);
                fail = 1; break;
            }
        }
    }
    if (fail) return 1;

    float z1 = world.bodies[c1].position.z;
    float z2 = world.bodies[c2].position.z;
    float gap = z2 - z1;
    printf("[info] c1 z=%.4f  c2 z=%.4f  gap=%.4f\n", z1, z2, gap);

    /* They started 0.6 apart. After colliding, c1 should still
    * be behind c2 (gap > 0). If gap < -0.1 they passed through. */
    if (gap < -0.1f) {
        printf("[FAIL] cylinders passed through each other\n");
        return 1;
    }
    printf("[PASS] cylinder-cylinder collision works\n");
    physics_world_cleanup(&world);
    return 0;
}
#endif /* MFS_CYL_CYL_TEST */
