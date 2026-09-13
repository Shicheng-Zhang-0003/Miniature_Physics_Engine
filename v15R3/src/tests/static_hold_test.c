/* MPE static-hold truth test: Coulomb stick must hold below the friction
 * angle and yield above it (no creep, no mid-slope freeze). */
#ifdef MPE_STATIC_HOLD_TEST
#include <stdio.h>
#include <math.h>
#include "core/physics_world.h"
#include "physics/constraint.h"
#include "config/mpe_config.h"

static float slope_drift(float slope_deg, float mus, float muk, int *asleep_out) {
    physics_world world;
    physics_world_init(&world);
    constraint_pool_init(&world);
    float ang = slope_deg * 3.14159265f / 180.0f;
    vector3 n = {-sinf(ang), cosf(ang), 0.0f};
    vector3 surf = {0.0f, 4.0f, 0.0f};
    vector3 rc = {surf.x - n.x * 0.5f, surf.y - n.y * 0.5f, 0.0f};
    int ramp = physics_world_add_cube(&world, rc, (vector3){8.0f, 0.5f, 5.0f}, 0.0f);
    rigidbody *rb = &world.bodies[ramp];
    rb->orientation = vector4_from_axis_with_angle((vector3){0, 0, 1}, ang);
    rigidbody_update_axes(rb);
    rigidbody_sanitize(rb);
    rb->friction_static = mus;
    rb->friction_kinetic = muk;
    float h = 0.25f;
    float drop = (fabsf(n.x) + fabsf(n.y) + fabsf(n.z)) * h;
    vector3 p0 = {surf.x + n.x * (drop + 0.005f), surf.y + n.y * (drop + 0.005f), 0.0f};
    int box = physics_world_add_cube(&world, p0, (vector3){h, h, h}, 1.0f);
    world.bodies[box].friction_static = mus;
    world.bodies[box].friction_kinetic = muk;
    const float dt = 1.0f / 60.0f;
    for (int t = 0; t < 120; t++) {
        physics_world_step(&world, dt);
    }
    vector3 s0 = world.bodies[box].position;
    for (int t = 0; t < 300; t++) {
        physics_world_step(&world, dt);
        if (!isfinite(world.bodies[box].position.x)) {
            break;
        }
    }
    vector3 s1 = world.bodies[box].position;
    vector3 d = {cosf(ang), sinf(ang), 0.0f};
    *asleep_out = world.bodies[box].is_sleeping;
    float drift = (s1.x - s0.x) * d.x + (s1.y - s0.y) * d.y;
    physics_world_cleanup(&world);
    return drift;
}

int main(void) {
    mpe_config_init();
    int fail = 0;
    int asleep = 0;

    /* 20 deg, mu_s 0.9 (friction angle 42 deg): must hold essentially still. */
    float hold_drift = slope_drift(-20.0f, 0.9f, 0.7f, &asleep);
    printf("[info] hold case: drift=%.4f m\n", hold_drift);
    if (fabsf(hold_drift) > 0.05f) {
        printf("[FAIL] static hold: box crept %.4f m on a slope it must hold\n", hold_drift);
        fail = 1;
    } else {
        printf("[PASS] static hold: box stands on 20deg slope at mu_s=0.9\n");
    }

    /* 10 deg, mu 0.1 (friction angle 5.7 deg): must slide freely, stay awake. */
    float slide_drift = slope_drift(-10.0f, 0.1f, 0.08f, &asleep);
    printf("[info] slide case: drift=%.4f m awake=%d\n", slide_drift, !asleep);
    if (slide_drift < 2.0f) {
        printf("[FAIL] slide: box barely moved (%.4f m) past the friction angle\n", slide_drift);
        fail = 1;
    } else if (asleep) {
        printf("[FAIL] slide: box fell asleep mid-slope\n");
        fail = 1;
    } else {
        printf("[PASS] past-angle slide stays free (%.4f m, awake)\n", slide_drift);
    }

    if (fail == 0) {
        printf("[PASS] static-hold truth complete\n");
    }
    return fail;
}
#endif /* MPE_STATIC_HOLD_TEST */
