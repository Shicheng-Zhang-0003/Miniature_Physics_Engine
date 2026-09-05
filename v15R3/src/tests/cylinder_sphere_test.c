/* MFS_174: Cylinder vs sphere collision test.
* A sphere approaches a resting cylinder. The sphere must not
* pass through the cylinder. */
#ifdef MFS_CYL_SPH_TEST
#include <stdio.h>
#include <math.h>
#include "core/physics_world.h"
#include "config/mpe_config.h"

int main(void) {
    mpe_config_init();
    physics_world world;
    physics_world_init(&world);

    /* Static floor so the cylinder rests */
    physics_world_add_cube(&world,
        (vector3){0.0f, -0.5f, 0.0f},
        (vector3){10.0f, 0.5f, 10.0f}, 0.0f);

    /* Static cylinder resting on floor */
    int cyl = physics_world_add_cylinder(&world,
        0.05f, 0.02f, 0.0f,
        (vector3){0.0f, 0.06f, 0.0f});

    /* Sphere approaching the cylinder along Z */
    int sph = physics_world_add_sphere(&world,
        0.08f, 0.3f,
        (vector3){0.0f, 0.08f, 0.5f});
    world.bodies[sph].velocity = (vector3){0.0f, 0.0f, -2.0f};

    const float dt = 1.0f / 60.0f;
    int fail = 0;
    for (int t = 0; t < 120 && !fail; t++) {
        physics_world_step(&world, dt);
        for (int i = 0; i < world.body_count; i++) {
            if (!isfinite(world.bodies[i].position.x) ||
                !isfinite(world.bodies[i].position.y) ||
                !isfinite(world.bodies[i].position.z)) {
                printf("[FAIL] NaN at tick %d\n", t);
                fail = 1; break;
            }
        }
    }
    if (fail) return 1;

    /* Sphere started at z=0.5 moving toward cylinder at z=0.
    * After 2 seconds it should have been deflected or stopped,
    * NOT passed through to z < -0.2 */
    float sph_z = world.bodies[sph].position.z;
    printf("[info] sphere final z=%.4f (started at 0.5)\n", sph_z);

    if (sph_z < -0.2f) {
        printf("[FAIL] sphere passed through cylinder\n");
        return 1;
    }
    printf("[PASS] cylinder-sphere collision works\n");
    physics_world_cleanup(&world);
    return 0;
}
#endif /* MFS_CYL_SPH_TEST */
