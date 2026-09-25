/* PARANOIA TEST: Floating point edge cases - denormals, Inf, NaN, precision limits */
#ifdef mpe_paranoia_fp_edges
#include <stdio.h>
#include <math.h>
#include <fenv.h>
#include "core/physics_world.h"
#include "physics/constraint.h"
#include "core/det_math.h"
#include "config/mpe_config.h"

int main(void) {
    mpe_config_init();
    int fail = 0;

    /* Test 1: Denormal handling - FTZ/DAZ should be disabled */
    {
        det_pin_fp_state();
        uint32_t mxcsr = 0;
        __asm__ volatile("stmxcsr %0" : "=m"(mxcsr));
        int ftz = (mxcsr >> 15) & 1;
        int daz = (mxcsr >> 24) & 1;
        printf("[INFO] MXCSR FTZ=%d DAZ=%d (should be 0)\n", ftz, daz);
        if (ftz || daz) { printf("[FAIL] FTZ/DAZ enabled - denormals flushed\n"); fail = 1; }
        else { printf("[PASS] denormals preserved\n"); }
    }

    /* Test 2: Inf propagation - should never occur */
    {
        physics_world world;
        physics_world_init(&world);
        constraint_pool_init(&world);

        g_cfg.world.gravity = -9.81f;
        g_cfg.world.drag = 1.0f;

        int a = physics_world_add_sphere(&world, 0.5f, 1.0f, (vector3){0.0f, 2.0f, 0.0f});
        world.bodies[a].velocity = (vector3){INFINITY, 0.0f, 0.0f}; /* should be sanitized */
        rigidbody_wake(&world.bodies[a]);

        const float dt = 1.0f / 60.0f;
        physics_world_step(&world, dt);

        int inf_count = 0;
        for (int i = 0; i < world.body_count; i++) {
            if (isinf(world.bodies[i].position.x) || isinf(world.bodies[i].velocity.x)) inf_count++;
        }

        printf("[INFO] inf_sanitization inf_count=%d\n", inf_count);
        if (inf_count > 0) { printf("[FAIL] Inf propagated\n"); fail = 1; }
        else { printf("[PASS] Inf sanitized\n"); }
        physics_world_cleanup(&world);
    }

    /* Test 3: NaN propagation - should never occur */
    {
        physics_world world;
        physics_world_init(&world);
        constraint_pool_init(&world);

        g_cfg.world.gravity = 0.0f;
        g_cfg.world.drag = 1.0f;

        int a = physics_world_add_sphere(&world, 0.5f, 1.0f, (vector3){0.0f, 0.0f, 0.0f});
        world.bodies[a].position.x = NAN; /* should be sanitized */
        rigidbody_wake(&world.bodies[a]);

        physics_world_step(&world, 1.0f / 60.0f);

        int nan_count = 0;
        for (int i = 0; i < world.body_count; i++) {
            if (isnan(world.bodies[i].position.x)) nan_count++;
        }

        printf("[INFO] nan_sanitization nan_count=%d\n", nan_count);
        if (nan_count > 0) { printf("[FAIL] NaN propagated\n"); fail = 1; }
        else { printf("[PASS] NaN sanitized\n"); }
        physics_world_cleanup(&world);
    }

    /* Test 4: det_math fallback counters - should be zero in-contract */
    {
        det_fallback_reset();
        physics_world world;
        physics_world_init(&world);
        constraint_pool_init(&world);

        g_cfg.world.gravity = -9.81f;
        g_cfg.world.drag = 0.99f;

        int s = physics_world_add_sphere(&world, 0.5f, 1.0f, (vector3){0.0f, 5.0f, 0.0f});
        world.bodies[s].velocity = (vector3){5.0f, 10.0f, 3.0f};
        world.bodies[s].angular_velocity = (vector3){2.0f, 1.0f, -1.0f};
        rigidbody_wake(&world.bodies[s]);

        const float dt = 1.0f / 60.0f;
        for (int t = 0; t < 3600; t++) physics_world_step(&world, dt);

        unsigned long pow_fallback = det_fallback_pow_total();
        unsigned long trig_fallback = det_fallback_trig_total();

        printf("[INFO] det_fallback pow=%lu trig=%lu\n", pow_fallback, trig_fallback);
        if (pow_fallback > 0 || trig_fallback > 0) {
            printf("[FAIL] det_math fallbacks triggered (non-deterministic)\n"); fail = 1;
        } else { printf("[PASS] det_math zero fallbacks in-contract\n"); }
        physics_world_cleanup(&world);
    }

    /* Test 5: Verify real subnormal arithmetic, independent of sleep gates. */
    {
        volatile float smallest_subnormal = 0x1p-149f;
        volatile float doubled_subnormal = smallest_subnormal + smallest_subnormal;
        int preserved = smallest_subnormal > 0.0f && !isnormal(smallest_subnormal) &&
                        doubled_subnormal == 0x1p-148f;
        printf("[INFO] subnormal smallest=%a doubled=%a\n",
               (double)smallest_subnormal, (double)doubled_subnormal);
        if (!preserved) { printf("[FAIL] subnormal addition flushed or changed\n"); fail = 1; }
        else { printf("[PASS] subnormal addition preserved\n"); }
    }

    /* Test 6: Matrix inversion near-singular - should not crash */
    {
        math3 m = {{{0}}};
        m.matrix[0][0] = 1e-20f;
        m.matrix[1][1] = 1.0f;
        m.matrix[2][2] = 1.0f;
        math3 inv = math3_inverse(m);

        int ok = isfinite(inv.matrix[0][0]) && isfinite(inv.matrix[1][1]) && isfinite(inv.matrix[2][2]);
        if (!ok) { printf("[FAIL] near-singular matrix inverse crashed\n"); fail = 1; }
        else { printf("[PASS] near-singular matrix handled\n"); }
    }

    /* Test 7: Quaternion normalization - exactly zero quat */
    {
        vector4 q = {0.0f, 0.0f, 0.0f, 0.0f};
        vector4 qn = vector4_normalisation(q);
        if (qn.w != 1.0f || qn.x != 0.0f || qn.y != 0.0f || qn.z != 0.0f) {
            printf("[FAIL] zero quat normalization failed\n"); fail = 1;
        } else { printf("[PASS] zero quat normalized to identity\n"); }
    }

    /* Test 8: Vector normalization - zero vector */
    {
        vector3 v = {0.0f, 0.0f, 0.0f};
        vector3 vn = vector3_normalisation(v);
        if (vn.x != 0.0f || vn.y != 0.0f || vn.z != 0.0f) {
            printf("[FAIL] zero vector normalization failed\n"); fail = 1;
        } else { printf("[PASS] zero vector normalized to zero\n"); }
    }

    /* Test 9: Cross product - parallel vectors */
    {
        vector3 a = {1.0f, 2.0f, 3.0f};
        vector3 b = {2.0f, 4.0f, 6.0f}; /* parallel */
        vector3 c = vector3_cross(a, b);
        float len2 = vector3_length_squared(c);
        if (len2 > 1e-10f) { printf("[FAIL] parallel cross not zero: %.2e\n", len2); fail = 1; }
        else { printf("[PASS] parallel cross is zero\n"); }
    }

    /* Test 10: Matrix multiplication - identity */
    {
        math3 m = math3_identity();
        math3 m2 = math3_multiplication(m, m);
        for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) {
            float expected = (i == j) ? 1.0f : 0.0f;
            if (fabsf(m2.matrix[i][j] - expected) > 1e-6f) {
                printf("[FAIL] identity multiply failed\n"); fail = 1;
            }
        }
        printf("[PASS] identity matrix multiply correct\n");
    }

    return fail;
}
#endif
