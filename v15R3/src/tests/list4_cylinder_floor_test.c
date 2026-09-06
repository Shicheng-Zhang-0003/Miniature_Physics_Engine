/* LIST4 NEW-01 regression test.
 *
 * Drops a tipped cylinder onto the floor. The old endpoint-only floor
 * collision could miss the true lowest point of a tipped cylinder.
 * The cylinder must not fall through the floor or become NaN.
 */
#ifdef LIST4_CYLINDER_FLOOR_TEST

#include <stdio.h>
#include <math.h>
#include "core/physics_world.h"
#include "config/mpe_config.h"

int main(void) {
    mpe_config_init();

    physics_world world;
    physics_world_init(&world);

    /*
     * Cylinder radius 0.05, half-length 0.02.
     * Start above the implicit physics_world floor at y=0.
     */
    int cyl = physics_world_add_cylinder(&world,
                                         0.05f,
                                         0.02f,
                                         0.5f,
                                         (vector3){0.0f, 0.25f, 0.0f});

    if (cyl < 0) {
        printf("[FAIL] could not create cylinder\n");
        return 1;
    }

    /*
     * Tip the axle.
     * Default axle is local X. Rotate 90 degrees about Z so the axle
     * points closer to world Y. This is the kind of orientation where
     * endpoint-only floor tests can fail.
     */
    world.bodies[cyl].orientation =
        vector4_from_axis_with_angle((vector3){0.0f, 0.0f, 1.0f}, math_pi * 0.5f);

    rigidbody_update_axes(&world.bodies[cyl]);

    const float dt = 1.0f / 60.0f;
    int fail = 0;

    for (int t = 0; t < 600; t++) {
        physics_world_step(&world, dt);

        if (!isfinite(world.bodies[cyl].position.y)) {
            printf("[FAIL] cylinder became NaN at tick %d\n", t);
            fail = 1;
            break;
        }
    }

    if (fail) {
        physics_world_cleanup(&world);
        return 1;
    }

    float final_y = world.bodies[cyl].position.y;

    printf("[info] tipped cylinder final y=%.4f\n", final_y);

    if (final_y < -0.05f) {
        printf("[FAIL] tipped cylinder fell through the floor\n");
        physics_world_cleanup(&world);
        return 1;
    }

    printf("[PASS] LIST4 cylinder floor contact holds\n");

    physics_world_cleanup(&world);
    return 0;
}

#endif /* LIST4_CYLINDER_FLOOR_TEST */
