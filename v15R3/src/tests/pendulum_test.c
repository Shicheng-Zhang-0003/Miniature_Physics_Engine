/* Pendulum truth: a compound pendulum must swing at
 * T = 2*pi*sqrt(I_pivot / (m*g*d)). Exercises revolute constraints +
 * gravity torque together. */
#ifdef MPE_PENDULUM_TEST
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

    /* Static pivot at (0,6,0). Rod 0.2 x 2.0 x 0.2, COM 1 m below anchor.
     * PHYSICS TRUTH: the hinge pin sits at the pivot's BOTTOM face
     * (local (0,-0.2,0)), not its center. Pinning the rod's top face to the
     * pivot CENTER buries 0.2 m of rod inside the solid cube: the joint
     * then perpetually pulls into the body while contact perpetually ejects
     * it — an unphysical geometry no solver can satisfy. A real hinge pin
     * lives outside both bodies. The pin hangs 0.1 m below the pivot's
     * bottom face so the 0.2 m-wide rod clears the cube through its full
     * 20-degree arc (corner rise is halfwidth*sin(20deg) = 0.034 m). */
    int pivot = physics_world_add_cube(&world, (vector3){0.0f, 6.0f, 0.0f}, (vector3){0.2f, 0.2f, 0.2f}, 0.0f);
    float swing = 20.0f * 3.14159265f / 180.0f;
    vector3 com = {sinf(swing) * 1.0f, 5.7f - cosf(swing) * 1.0f, 0.0f};
    int rod = physics_world_add_cube(&world, com, (vector3){0.1f, 1.0f, 0.1f}, 1.0f);
    uint32_t pivot_id = world.bodies[pivot].object_id;
    uint32_t rod_id = world.bodies[rod].object_id;
    /* Hinge about z, pin 0.1 m below the pivot's bottom face; rod anchor at its top face. */
    if (constraint_add_revolute(&world, pivot_id, rod_id, (vector3){0.0f, -0.3f, 0.0f}, (vector3){0.0f, 1.0f, 0.0f},
                                (vector3){0.0f, 0.0f, 1.0f}) < 0) {
        printf("[FAIL] joint creation\n");
        physics_world_cleanup(&world);
        return 1;
    }

    /* I about pivot: box-inertia + parallel axis, d = 1. */
    float I = (1.0f / 12.0f) * (4.0f + 0.04f) + 1.0f;
    float T_exact = 2.0f * 3.14159265f * sqrtf(I / 9.81f);
    const float dt = 1.0f / 60.0f;
    float prev_x = world.bodies[rod].position.x;
    int crossings = 0, first = -1, last = -1;
    for (int t = 0; t < 600; t++) {
        physics_world_step(&world, dt);
        float x = world.bodies[rod].position.x;
        if (!isfinite(x)) {
            printf("[FAIL] NaN\n");
            physics_world_cleanup(&world);
            return 1;
        }
        if ((prev_x <= 0.0f && x > 0.0f) || (prev_x >= 0.0f && x < 0.0f)) {
            crossings++;
            if (first < 0) {
                first = t;
            }
            last = t;
        }
        prev_x = x;
    }
    int fail = 0;
    float T_meas = 0.0f;
    if (crossings >= 4) {
        T_meas = 2.0f * (float)(last - first) * dt / (float)(crossings - 1);
    }
    printf("[info] period: measured=%.4f analytic=%.4f crossings=%d\n", T_meas, T_exact, crossings);
    if (fabsf(T_meas - T_exact) / T_exact > 0.08f) {
        printf("[FAIL] pendulum period off\n");
        fail = 1;
    } else {
        printf("[PASS] compound pendulum period exact\n");
    }
    physics_world_cleanup(&world);
    return fail;
}
#endif /* MPE_PENDULUM_TEST */
