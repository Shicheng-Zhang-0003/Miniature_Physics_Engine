/* MFS_174: Cylinder vs cube wall collision test.
* A cylinder rolls toward a static cube wall. It must not
* pass through the wall. */
#ifdef MFS_CYL_CUBE_TEST
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

    /* Static cube wall at z=0.5 */
    physics_world_add_cube(&world,
        (vector3){0.0f, 0.25f, 0.5f},
        (vector3){0.5f, 0.25f, 0.1f}, 0.0f);

    /* Cylinder rolling toward the wall */
    int cyl = physics_world_add_cylinder(&world,
        0.05f, 0.02f, 0.5f,
        (vector3){0.0f, 0.06f, -0.5f});
    world.bodies[cyl].velocity = (vector3){0.0f, 0.0f, 3.0f};

    const float dt = 1.0f / 60.0f;
    int fail = 0;
    for (int t = 0; t < 180 && !fail; t++) {
        physics_world_step(&world, dt);
        for (int i = 0; i < world.body_count; i++) {
            if (!isfinite(world.bodies[i].position.z)) {
                printf("[FAIL] NaN at tick %d\n", t);
                fail = 1; break;
            }
        }
    }
    if (fail) return 1;

    float cyl_z = world.bodies[cyl].position.z;
    printf("[info] cylinder final z=%.4f (wall at z=0.5)\n", cyl_z);

    if (cyl_z > 0.8f) {
        printf("[FAIL] cylinder passed through wall\n");
        return 1;
    }
    printf("[PASS] cylinder-cube collision works\n");
    physics_world_cleanup(&world);
    return 0;
}
#endif /* MFS_CYL_CUBE_TEST */
