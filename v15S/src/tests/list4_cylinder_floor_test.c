/* LIST4 NEW-01 regression test.
 *
 * Drops a tipped cylinder onto the floor. The old endpoint-only floor
 * collision could miss the true lowest point of a tipped cylinder.
 * The cylinder must not fall through the floor or become NaN.
 */
#ifdef list4_cylinder_floor_test

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
     * Tip the axle 90 degrees about Y so it points UP (world Z).
     * Default axle is local X. Rotate 90 degrees about Y so local X -> world Z.
     * Now the cylinder stands vertically on its circular face (like a coin on edge).
     * Lowest point = center.y - half_length = 0.02 above floor.
     */
    world.bodies[cyl].orientation =
        vector4_from_axis_with_angle((vector3){0.0f, 1.0f, 0.0f}, math_pi * 0.5f);

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
    float final_vy = world.bodies[cyl].velocity.y;

    printf("[info] tipped cylinder final y=%.4f vy=%.4f (rest 0.02)\n", final_y, final_vy);

    /* TRUTH: axle vertical (90° about Y). Cylinder stands on circular face.
     * Rest height = half_length = 0.02. Floor collision uses exact SDF
     * which computes lowest point as center.y - half_length. */
    if (final_y < -0.05f) {
        printf("[FAIL] tipped cylinder fell through the floor\n");
        physics_world_cleanup(&world);
        return 1;
    }
    /* Rest height = half_length = 0.02. Tolerance ±0.01 for discrete stepping. */
    if (final_y < 0.01f || final_y > 0.03f) {
        printf("[FAIL] tipped cylinder not at rest height (y=%.4f)\n", final_y);
        physics_world_cleanup(&world);
        return 1;
    }
    if (fabsf(final_vy) > 0.1f) {
        printf("[FAIL] tipped cylinder still moving (vy=%.4f)\n", final_vy);
        physics_world_cleanup(&world);
        return 1;
    }

    printf("[PASS] LIST4 cylinder floor contact holds\n");

    physics_world_cleanup(&world);
    return 0;
}

#endif /* list4_cylinder_floor_test */