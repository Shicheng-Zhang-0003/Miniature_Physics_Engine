/* MPE Suite v2 — file B: joints + shapes (8 tests).
 * spring impl lives here (needs spring_joint TU + scene stubs, defined in
 * mpe_suite_main.c). list4 is FIXED (Z-rotation + frictional floor).
 * NOTE: compound-pendulum inertia ii = 1/12*m*(L^2+w^2) + m*d^2 lives in
 * suite_a (mpe_t_pendulum) with the m=1, d=1 substitution shown there. */
#include <math.h>
#include <stdio.h>
#include "mpe_test.h"
#include "physics/spring_joint.h"
#include "core/rigidbody.h"

int mpe_t_spring(void) {
    mpe_test_t t;
    mpe_test_begin(&t, "spring");
    mpe_config_init();
    g_cfg.world.drag = 1.0f;
    g_cfg.world.angular_damping_scale = 1.0f;
    g_cfg.world.gravity = 0.0f;
    physics_world w;
    mpe_world_begin(&w);
    joint_init_pool(&w);
    const float k = 20.0f;
    int anchor = physics_world_add_cube(&w, (vector3){0.0f, 50.0f, 0.0f},
                                        (vector3){0.5f, 0.5f, 0.5f}, 0.0f);
    int mass = physics_world_add_sphere(&w, 0.2f, 1.0f, (vector3){2.5f, 50.0f, 0.0f});
    MPE_CHECK(&t, anchor >= 0 && mass >= 0);
    uint32_t ida = w.bodies[anchor].object_id;
    uint32_t idm = w.bodies[mass].object_id;
    MPE_CHECK(&t, add_joint_by_ids(&w, ida, idm, 2.0f, k, 0.0f) >= 0);
    const float dt = 1.0f / 60.0f;
    float prev_x = w.bodies[mass].position.x - 2.0f;
    int crossings = 0, first_cross = -1, last_cross = -1;
    float e0 = 0.5f * k * 0.25f;
    float emax_dev = 0.0f;
    for (int kk = 0; kk < 600; kk++) {
        physics_world_step(&w, dt);
        rigidbody *mb = &w.bodies[mass];
        if (!isfinite(mb->position.x)) {
            printf("[FAIL] NaN\n");
            t.failures++;
            break;
        }
        float x = mb->position.x - 2.0f;
        if ((prev_x <= 0.0f && x > 0.0f) || (prev_x >= 0.0f && x < 0.0f)) {
            crossings++;
            if (first_cross < 0) {
                first_cross = kk;
            }
            last_cross = kk;
        }
        prev_x = x;
        float e = 0.5f * k * x * x + 0.5f * 1.0f * vector3_length_squared(mb->velocity);
        float dev = fabsf(e - e0) / e0;
        if (dev > emax_dev) {
            emax_dev = dev;
        }
    }
    float measured_t = 0.0f;
    if (crossings >= 4) {
        measured_t = 2.0f * (float)(last_cross - first_cross) * dt / (float)(crossings - 1);
    }
    /* T = 2*pi*sqrt(m/k) with m=1, k=20. */
    float analytic_t = 2.0f * 3.14159265f * sqrtf(1.0f / k);
    MPE_INFO("period: measured=%.4f analytic=%.4f crossings=%d", measured_t, analytic_t, crossings);
    MPE_CHECK(&t, crossings >= 4);
    if (crossings >= 4) {
        MPE_CHECK_REL(&t, measured_t, analytic_t, 0.02f, "spring-period");
    }
    MPE_INFO("max energy deviation: %.3f", emax_dev);
    MPE_CHECK(&t, emax_dev <= 0.05f);
    if (t.failures == 0) {
        printf("[PASS] spring period matches 2*pi*sqrt(m/k)\n");
    }
    physics_world_cleanup(&w);
    mpe_test_end(&t);
    return t.failures;
}

int mpe_t_two_world(void) {
    mpe_test_t t;
    mpe_test_begin(&t, "two_world");
    mpe_config_init();
    physics_world wa, wb;
    physics_world_init(&wa);
    physics_world_init(&wb);
    mpe_config_t cfg_b = g_cfg;
    cfg_b.world.gravity = -1.0f;
    physics_world_set_config(&wb, &cfg_b);
    physics_world_add_sphere(&wa, 0.5f, 1.0f, (vector3){0.0f, 10.0f, 0.0f});
    physics_world_add_sphere(&wb, 0.5f, 1.0f, (vector3){0.0f, 10.0f, 0.0f});
    const float dt = 1.0f / 60.0f;
    float ya_mid = 0.0f, yb_mid = 0.0f;
    for (int k = 0; k < 600; k++) {
        physics_world_step(&wa, dt);
        physics_world_step(&wb, dt);
        if (!mpe_world_finite(&wa) || !mpe_world_finite(&wb)) {
            t.failures++;
            break;
        }
        if (k == 60) {
            ya_mid = wa.bodies[0].position.y;
            yb_mid = wb.bodies[0].position.y;
            MPE_INFO("t=1s: y_a=%.3f (g=-9.81) y_b=%.3f (g=-1.0)", ya_mid, yb_mid);
        }
    }
    MPE_CHECK(&t, (yb_mid - ya_mid) >= 2.0f);
    MPE_CHECK(&t, wa.bodies[0].position.y <= 9.0f);
    if (t.failures == 0) {
        printf("[PASS] two worlds independent: separated %.3f m at t=1s by per-world gravity\n",
               yb_mid - ya_mid);
    }
    physics_world_cleanup(&wa);
    physics_world_cleanup(&wb);
    mpe_test_end(&t);
    return t.failures;
}

int mpe_t_revolute(void) {
    mpe_test_t t;
    mpe_test_begin(&t, "revolute");
    mpe_config_init();
    physics_world w;
    mpe_world_begin(&w);
    int pivot = physics_world_add_cube(&w, (vector3){0.0f, 10.0f, 0.0f},
                                       (vector3){0.2f, 0.2f, 0.2f}, 1.0f);
    MPE_CHECK(&t, pivot >= 0);
    rigidbody_set_static(&w.bodies[pivot], true);
    uint32_t pid = w.bodies[pivot].object_id;
    int bob = physics_world_add_sphere(&w, 0.3f, 2.0f, (vector3){1.0f, 8.0f, 0.0f});
    MPE_CHECK(&t, bob >= 0);
    uint32_t bid = w.bodies[bob].object_id;
    vector3 pivot_point = {0.0f, 10.0f, 0.0f};
    float rod_length = vector3_length(vector3_subtraction(pivot_point, w.bodies[bob].position));
    vector3 start_position = w.bodies[bob].position;
    MPE_CHECK(&t, constraint_add_revolute(&w, pid, bid, (vector3){0.0f, 0.0f, 0.0f},
                                          (vector3){-1.0f, 2.0f, 0.0f},
                                          (vector3){0.0f, 0.0f, 1.0f}) >= 0);
    const float dt = 1.0f / 60.0f;
    float max_drift = 0.0f;
    for (int k = 0; k < 600; k++) {
        physics_world_step(&w, dt);
        rigidbody *bb = &w.bodies[bob];
        if (!isfinite(bb->position.x) || !isfinite(bb->position.y) || !isfinite(bb->position.z)) {
            printf("[FAIL] bob went non-finite at tick %d\n", k);
            t.failures++;
            break;
        }
        float drift = fabsf(vector3_length(vector3_subtraction(pivot_point, bb->position)) - rod_length);
        if (drift > max_drift) {
            max_drift = drift;
        }
    }
    float moved = vector3_length(vector3_subtraction(w.bodies[bob].position, start_position));
    MPE_INFO("rod=%.4f max_drift=%.4f moved=%.4f", rod_length, max_drift, moved);
    MPE_CHECK(&t, max_drift <= 0.02f);
    MPE_CHECK(&t, moved >= 0.05f);
    if (t.failures == 0) {
        printf("[PASS] revolute pendulum holds (max drift %.4f) and swings under gravity\n", max_drift);
    }
    physics_world_cleanup(&w);
    mpe_test_end(&t);
    return t.failures;
}

int mpe_t_cylinder_drop(void) {
    mpe_test_t t;
    mpe_test_begin(&t, "cylinder_drop");
    mpe_config_init();
    MPE_INFO("gravity = %.4f", g_cfg.world.gravity);
    physics_world w;
    mpe_world_begin(&w);
    MPE_CHECK(&t, mpe_floor_slab(&w, 0.4f, 0.3f, 0.0f) >= 0);
    int cyl = physics_world_add_cylinder(&w, 0.05f, 0.02f, 0.5f, (vector3){0.0f, 0.25f, 0.0f});
    int sph = physics_world_add_sphere(&w, 0.05f, 0.5f, (vector3){1.0f, 0.25f, 0.0f});
    MPE_CHECK(&t, cyl >= 0 && sph >= 0);
    const float dt = 1.0f / 60.0f;
    if (!mpe_step(&w, 300, dt)) {
        t.failures++;
    }
    float cyl_y = w.bodies[cyl].position.y;
    float cyl_vy = w.bodies[cyl].velocity.y;
    float sph_y = w.bodies[sph].position.y;
    MPE_INFO("sphere y=%.4f cylinder y=%.4f vy=%.4f", sph_y, cyl_y, cyl_vy);
    MPE_CHECK(&t, sph_y <= 0.20f);
    MPE_CHECK(&t, sph_y >= -0.05f);
    MPE_CHECK_NEAR(&t, sph_y, 0.05f, 0.02f, "sphere-rest");
    MPE_CHECK(&t, cyl_y >= -0.05f);
    MPE_CHECK_NEAR(&t, cyl_y, 0.05f, 0.02f, "cylinder-rest");
    if (t.failures == 0) {
        printf("[PASS] cylinder rested on the floor (y=%.4f)\n", cyl_y);
    }
    physics_world_cleanup(&w);
    mpe_test_end(&t);
    return t.failures;
}

int mpe_t_cylinder_sphere(void) {
    mpe_test_t t;
    mpe_test_begin(&t, "cylinder_sphere");
    mpe_config_init();
    physics_world w;
    mpe_world_begin(&w);
    MPE_CHECK(&t, mpe_floor_slab(&w, 0.4f, 0.3f, 0.0f) >= 0);
    int cyl = physics_world_add_cylinder(&w, 0.05f, 0.02f, 0.0f, (vector3){0.0f, 0.06f, 0.0f});
    int sph = physics_world_add_sphere(&w, 0.08f, 0.3f, (vector3){0.0f, 0.08f, 0.5f});
    MPE_CHECK(&t, cyl >= 0 && sph >= 0);
    w.bodies[sph].velocity = (vector3){0.0f, 0.0f, -2.0f};
    const float dt = 1.0f / 60.0f;
    if (!mpe_step(&w, 120, dt)) {
        t.failures++;
    }
    float sph_z = w.bodies[sph].position.z;
    MPE_INFO("sphere final z=%.4f (started at 0.5)", sph_z);
    MPE_CHECK(&t, sph_z >= -0.05f);
    if (t.failures == 0) {
        printf("[PASS] cylinder-sphere collision works\n");
    }
    physics_world_cleanup(&w);
    mpe_test_end(&t);
    return t.failures;
}

int mpe_t_cylinder_cube(void) {
    mpe_test_t t;
    mpe_test_begin(&t, "cylinder_cube");
    mpe_config_init();
    physics_world w;
    mpe_world_begin(&w);
    MPE_CHECK(&t, mpe_floor_slab(&w, 0.4f, 0.3f, 0.0f) >= 0);
    MPE_CHECK(&t, physics_world_add_cube(&w, (vector3){0.0f, 0.25f, 0.5f},
                                         (vector3){0.5f, 0.25f, 0.1f}, 0.0f) >= 0);
    int cyl = physics_world_add_cylinder(&w, 0.05f, 0.02f, 0.5f, (vector3){0.0f, 0.06f, -0.5f});
    MPE_CHECK(&t, cyl >= 0);
    w.bodies[cyl].velocity = (vector3){0.0f, 0.0f, 3.0f};
    const float dt = 1.0f / 60.0f;
    if (!mpe_step(&w, 180, dt)) {
        t.failures++;
    }
    float cyl_z = w.bodies[cyl].position.z;
    float cyl_vz = w.bodies[cyl].velocity.z;
    MPE_INFO("cylinder final z=%.4f vz=%.4f (wall face at z=0.4)", cyl_z, cyl_vz);
    MPE_CHECK(&t, cyl_z <= 0.40f);
    MPE_CHECK(&t, fabsf(cyl_vz) <= 1.0f);
    if (t.failures == 0) {
        printf("[PASS] cylinder-cube wall holds\n");
    }
    physics_world_cleanup(&w);
    mpe_test_end(&t);
    return t.failures;
}

int mpe_t_cylinder_cylinder(void) {
    mpe_test_t t;
    mpe_test_begin(&t, "cylinder_cylinder");
    mpe_config_init();
    physics_world w;
    mpe_world_begin(&w);
    MPE_CHECK(&t, mpe_floor_slab(&w, 0.4f, 0.3f, 0.0f) >= 0);
    int c1 = physics_world_add_cylinder(&w, 0.05f, 0.02f, 0.0f, (vector3){0.0f, 0.06f, 0.0f});
    int c2 = physics_world_add_cylinder(&w, 0.05f, 0.02f, 0.5f, (vector3){0.0f, 0.06f, 0.5f});
    MPE_CHECK(&t, c1 >= 0 && c2 >= 0);
    w.bodies[c2].velocity = (vector3){0.0f, 0.0f, -2.0f};
    const float dt = 1.0f / 60.0f;
    if (!mpe_step(&w, 180, dt)) {
        t.failures++;
    }
    float z2 = w.bodies[c2].position.z;
    MPE_INFO("moving cylinder final z=%.4f (started 0.5, other at 0)", z2);
    MPE_CHECK(&t, isfinite(z2) && z2 >= -0.05f);
    if (t.failures == 0) {
        printf("[PASS] cylinder-cylinder collision works\n");
    }
    physics_world_cleanup(&w);
    mpe_test_end(&t);
    return t.failures;
}

/* list4 FIXED: v1 rotated about Y (axle X->Z, still horizontal) yet asserted
 * the vertical rest height h=0.02. v2 tests BOTH poses with the frictional
 * floor enabled (v1 measured free-fall through the frictionless clamp):
 *   face  (Z-rot, axle X->Y vertical): rest = half_length = 0.02
 *   barrel(Y-rot, axle X->Z horizontal): rest = radius = 0.05 */
int mpe_t_list4_cylinder_floor(void) {
    mpe_test_t t;
    mpe_test_begin(&t, "list4_cylinder_floor");
    mpe_config_init();
    const float dt = 1.0f / 60.0f;
    /* Case FACE: stand on the circular face. */
    {
        physics_world w;
        mpe_world_begin(&w);
        mpe_floor_plane(&w, 0.4f, 0.3f);
        int cyl = physics_world_add_cylinder(&w, 0.05f, 0.02f, 0.5f, (vector3){0.0f, 0.25f, 0.0f});
        MPE_CHECK(&t, cyl >= 0);
        w.bodies[cyl].orientation =
            vector4_from_axis_with_angle((vector3){0.0f, 0.0f, 1.0f}, math_pi * 0.5f);
        rigidbody_update_axes(&w.bodies[cyl]);
        if (!mpe_step(&w, 600, dt)) {
            t.failures++;
        }
        float y = w.bodies[cyl].position.y;
        float vy = w.bodies[cyl].velocity.y;
        MPE_INFO("face: y=%.4f vy=%.4f (rest 0.02)", y, vy);
        MPE_CHECK(&t, y >= -0.05f);
        MPE_CHECK_NEAR(&t, y, 0.02f, 0.01f, "face-rest");
        MPE_CHECK(&t, fabsf(vy) <= 0.1f);
        physics_world_cleanup(&w);
    }
    /* Case BARREL: lie on the side. */
    {
        physics_world w;
        mpe_world_begin(&w);
        mpe_floor_plane(&w, 0.4f, 0.3f);
        int cyl = physics_world_add_cylinder(&w, 0.05f, 0.02f, 0.5f, (vector3){0.0f, 0.25f, 0.0f});
        MPE_CHECK(&t, cyl >= 0);
        w.bodies[cyl].orientation =
            vector4_from_axis_with_angle((vector3){0.0f, 1.0f, 0.0f}, math_pi * 0.5f);
        rigidbody_update_axes(&w.bodies[cyl]);
        if (!mpe_step(&w, 600, dt)) {
            t.failures++;
        }
        float y = w.bodies[cyl].position.y;
        float vy = w.bodies[cyl].velocity.y;
        MPE_INFO("barrel: y=%.4f vy=%.4f (rest 0.05)", y, vy);
        MPE_CHECK(&t, y >= -0.05f);
        MPE_CHECK_NEAR(&t, y, 0.05f, 0.01f, "barrel-rest");
        MPE_CHECK(&t, fabsf(vy) <= 0.1f);
        physics_world_cleanup(&w);
    }
    if (t.failures == 0) {
        printf("[PASS] LIST4 cylinder floor contact holds (face + barrel)\n");
    }
    mpe_test_end(&t);
    return t.failures;
}
