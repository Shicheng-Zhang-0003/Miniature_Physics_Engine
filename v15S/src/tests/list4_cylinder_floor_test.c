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

    /* The virtual backstop has no shape/material friction. This regression
     * uses an explicit slab with its top surface at y=0. */
    int floor = physics_world_add_cube(&world, (vector3){0.0f, -0.5f, 0.0f},
                                       (vector3){10.0f, 0.5f, 10.0f}, 0.0f);
    if (floor < 0) {
        printf("[FAIL] could not create floor\n");
        physics_world_cleanup(&world);
        return 1;
    }
    world.bodies[floor].restitution = 0.0f;
    world.bodies[floor].friction_static = 0.8f;
    world.bodies[floor].friction_kinetic = 0.6f;

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
     * The default axle is local X. Rotate 90 degrees about Z so local X ->
     * world Y (the vertical axis). The cylinder stands on its circular face;
     * lowest point = center.y - half_length = 0.02 above the floor.
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

    rigidbody *body = &world.bodies[cyl];
    float final_y = body->position.y;
    float final_vy = body->velocity.y;
    float axis_y = fabsf(body->cached_axes[0].y);
    float support_y = body->cylinder_half_length * axis_y +
                      body->radius * sqrtf(fmaxf(0.0f, 1.0f - axis_y * axis_y));

    printf("[info] tipped cylinder final y=%.4f vy=%.4f support=%.4f\n", final_y, final_vy, support_y);

    /* For axle unit vector a, vertical support radius is
     * h*|a.y| + r*sqrt(1-a.y^2). The solver may tip this short, squat cylinder,
     * so assert against its final geometry rather than its initial pose. */
    if (!isfinite(final_y) || !isfinite(final_vy) || !isfinite(support_y) ||
        (final_y < support_y - 0.01f)) {
        printf("[FAIL] tipped cylinder penetrated the floor\n");
        physics_world_cleanup(&world);
        return 1;
    }
    if (fabsf(final_y - support_y) > 0.02f) {
        printf("[FAIL] cylinder not settled at support height (y=%.4f support=%.4f)\n", final_y, support_y);
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
