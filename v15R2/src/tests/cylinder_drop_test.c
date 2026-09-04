/* MPE_FTC_092a: Cylinder drop test (gravity fixed).
 *
 * Drops a cylinder (a wheel) and a control sphere onto a static floor.
 * Calls mpe_config_init() so g_cfg.world.gravity is real — without it,
 * gravity is 0 and nothing falls (the bug 092 hit).
 *
 * Diagnostic order:
 *   1. If the control SPHERE does not fall, gravity/integration is broken.
 *   2. If the sphere rests but the CYLINDER falls through, cylinder
 *      contact is missing (the real keystone gap -> 093). */
#ifdef MPE_CYLINDER_DROP_TEST

#include <stdio.h>
#include <math.h>
#include "core/physics_world.h"
#include "config/mpe_config.h"

int main(void) {
    mpe_config_init(); /* MPE_FTC_092a: required for real gravity */

    printf("[info] gravity = %.4f\n", g_cfg.world.gravity);

    physics_world world;
    physics_world_init(&world);
    world.next_object_id = 1;

    /* Static floor: large flat cube, top surface at y = 0. */
    int floor_idx = physics_world_add_cube(&world,
        (vector3){0.0f, -0.5f, 0.0f},
        (vector3){10.0f, 0.5f, 10.0f},
        0.0f);

    /* Cylinder wheel: radius 0.05, half-length 0.02, mass 0.5. */
    int cyl_idx = physics_world_add_cylinder(&world,
        0.05f, 0.02f, 0.5f,
        (vector3){0.0f, 0.25f, 0.0f});

    /* Control sphere: same radius and spawn height. */
    int sph_idx = physics_world_add_sphere(&world,
        0.05f, 0.5f,
        (vector3){1.0f, 0.25f, 0.0f});

    if ((floor_idx < 0) || (cyl_idx < 0) || (sph_idx < 0)) {
        printf("[FAIL] could not create bodies\n");
        return 1;
    }

    const float dt = 1.0f / 60.0f;
    float cyl_y = 0.25f, cyl_vy = 0.0f, sph_y = 0.25f;
    int nan_seen = 0;

    for (int t = 0; t < 300; t++) {   /* 5 simulated seconds */
        physics_world_step(&world, dt);
        cyl_y  = world.bodies[cyl_idx].position.y;
        cyl_vy = world.bodies[cyl_idx].velocity.y;
        sph_y  = world.bodies[sph_idx].position.y;
        if ((!isfinite(cyl_y)) || (!isfinite(cyl_vy)) || (!isfinite(sph_y))) {
            nan_seen = 1;
            break;
        }
    }

    printf("[info] sphere   final y=%.4f\n", sph_y);
    printf("[info] cylinder final y=%.4f vy=%.4f\n", cyl_y, cyl_vy);

    if (nan_seen) {
        printf("[FAIL] NaN during drop\n");
        return 1;
    }

    /* 1. Sphere sanity: with gravity it must fall and rest near y=radius. */
    if (sph_y > 0.20f) {
        printf("[GAP] control sphere did not fall (y=%.4f) — gravity or integration broken\n", sph_y);
        return 1;
    }

    /* 2. Cylinder: did it fall through the floor? */
    if (cyl_y < -5.0f) {
        printf("[GAP] cylinder fell through the floor (y=%.4f) — cylinder contact missing\n", cyl_y);
        return 1;
    }
    if (cyl_y > 0.20f) {
        printf("[FAIL] cylinder did not settle (y=%.4f)\n", cyl_y);
        return 1;
    }

    printf("[PASS] cylinder rested on the floor (y=%.4f)\n", cyl_y);
    return 0;
}

#endif /* MPE_CYLINDER_DROP_TEST */
