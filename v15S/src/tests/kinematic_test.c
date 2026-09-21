/* MPE kinematic truth test: a velocity-driven platform ignores gravity and
 * carries a resting body with it through contact. */
#ifdef mpe_kinematic_test
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

    /* Platform: box bottom at y=0, driven +x at 2 m/s, immune to gravity. */
    int p = physics_world_add_cube(&world, (vector3){0, 0.5f, 0}, (vector3){2.0f, 0.5f, 2.0f}, 5.0f);
    rigidbody_set_kinematic(&world.bodies[p], true);
    world.bodies[p].velocity = (vector3){2.0f, 0.0f, 0.0f};

    /* Crate resting on top. */
    int c = physics_world_add_cube(&world, (vector3){0, 1.26f, 0}, (vector3){0.25f, 0.25f, 0.25f}, 1.0f);

    const float dt = 1.0f / 60.0f;
    for (int t = 0; t < 120; t++) {
        /* Re-assert drive (a real driver sets this every tick). */
        world.bodies[p].velocity = (vector3){2.0f, 0.0f, 0.0f};
        physics_world_step(&world, dt);
        if (!isfinite(world.bodies[c].position.x)) {
            printf("[FAIL] NaN\n");
            physics_world_cleanup(&world);
            return 1;
        }
    }
    float plat_y = world.bodies[p].position.y;
    float plat_x = world.bodies[p].position.x;
    float crate_x = world.bodies[c].position.x;
    float crate_y = world.bodies[c].position.y;
    printf("[info] platform x=%.3f y=%.4f | crate x=%.3f y=%.4f\n", plat_x, plat_y, crate_x, crate_y);
    int fail = 0;
    if (fabsf(plat_y - 0.5f) > 0.01f) {
        printf("[FAIL] kinematic platform fell under gravity (y=%.4f)\n", plat_y);
        fail = 1;
    } else {
        printf("[PASS] kinematic platform ignores gravity\n");
    }
    if (plat_x < 3.8f || plat_x > 4.2f) {
        printf("[FAIL] platform did not advance at drive velocity (x=%.3f, expect 4.0)\n", plat_x);
        fail = 1;
    } else {
        printf("[PASS] kinematic platform advances at drive velocity\n");
    }
    /* TRUTH: two-sided carry proof. Old crate_x<1.0 allowed 75% slip. The
     * measured slip is 0.67 (carried, with rolling lag); <1.0 keeps meaning
     * (0.5 would red a working carry). crate_y>0.8 proves it rode ON TOP
     * (1.25): fallen through the platform to the floor (0.25) still drags
     * 1m and used to pass. */
    if (fabsf(crate_x - plat_x) > 1.0f) {
        printf("[FAIL] crate not carried (crate_x=%.3f plat_x=%.3f)\n", crate_x, plat_x);
        fail = 1;
    } else {
        printf("[PASS] contact carries the crate (x=%.3f)\n", crate_x);
    }
    if (crate_y < 0.8f) {
        printf("[FAIL] crate fell through the platform (y=%.4f)\n", crate_y);
        fail = 1;
    }
    physics_world_cleanup(&world);
    return fail;
}
#endif /* mpe_kinematic_test */
