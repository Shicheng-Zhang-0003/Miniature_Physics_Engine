/* Incline-acceleration truth: frictionless slide must accelerate at exactly
 * g*sin(theta) down the slope. */
#ifdef MPE_INCLINE_ACCEL_TEST
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

    float ang = -30.0f * 3.14159265f / 180.0f;
    vector3 n = {-sinf(ang), cosf(ang), 0.0f};
    vector3 surf = {0.0f, 6.0f, 0.0f};
    vector3 rc = {surf.x - n.x * 0.5f, surf.y - n.y * 0.5f, 0.0f};
    int ramp = physics_world_add_cube(&world, rc, (vector3){10.0f, 0.5f, 5.0f}, 0.0f);
    rigidbody *rb = &world.bodies[ramp];
    rb->orientation = vector4_from_axis_with_angle((vector3){0, 0, 1}, ang);
    rigidbody_update_axes(rb);
    rigidbody_sanitize(rb);
    float h = 0.25f;
    float drop = (fabsf(n.x) + fabsf(n.y) + fabsf(n.z)) * h;
    vector3 p0 = {surf.x + n.x * (drop + 0.01f), surf.y + n.y * (drop + 0.01f), 0.0f};
    int box = physics_world_add_cube(&world, p0, (vector3){h, h, h}, 1.0f);
    world.bodies[box].friction_static = 0.0f;
    world.bodies[box].friction_kinetic = 0.0f;
    rb->friction_static = 0.0f;
    rb->friction_kinetic = 0.0f;

    const float dt = 1.0f / 60.0f;
    vector3 d = {cosf(ang), sinf(ang), 0.0f};
    /* Measure acceleration over the middle second (settled sliding). */
    for (int t = 0; t < 30; t++) {
        physics_world_step(&world, dt);
    }
    vector3 p_a = world.bodies[box].position;
    float v_a = vector3_dot(world.bodies[box].velocity, d);
    for (int t = 0; t < 60; t++) {
        physics_world_step(&world, dt);
        if (!isfinite(world.bodies[box].position.x)) {
            printf("[FAIL] NaN\n");
            physics_world_cleanup(&world);
            return 1;
        }
    }
    vector3 p_b = world.bodies[box].position;
    float v_b = vector3_dot(world.bodies[box].velocity, d);
    float a_meas = (v_b - v_a) / 1.0f;
    float a_exact = 9.81f * sinf(-ang); /* downhill magnitude */
    printf("[info] slide accel=%.4f (expect %.4f)\n", a_meas, a_exact);
    int fail = 0;
    if (fabsf(a_meas - a_exact) / a_exact > 0.04f) {
        printf("[FAIL] incline acceleration off\n");
        fail = 1;
    } else {
        printf("[PASS] frictionless slide accelerates at g*sin(theta)\n");
    }
    (void) p_a;
    (void) p_b;
    physics_world_cleanup(&world);
    return fail;
}
#endif /* MPE_INCLINE_ACCEL_TEST */
