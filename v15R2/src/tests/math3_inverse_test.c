/* MPE_FTC_093e: math3_inverse diagnostic.
 * Test A: invert a known diagonal matrix, check the result.
 * Test B: re-run the air-wheel probe to see if fixing sanitize
 *         alone resolved the zero-inverse problem. */
#ifdef MPE_MATH3_INVERSE_TEST

#include <math.h>
#include <stdio.h>

#include "config/mpe_config.h"
#include "core/physics_world.h"
#include "core/rigidbody.h"
#include "core/math3D.h"

int main(void) {
    mpe_config_init();

    /* --- Test A: direct matrix inverse --- */
    printf("--- Test A: direct math3_inverse ---\n");
    math3 diag = {{{0.000625f, 0, 0}, {0, 0.000379f, 0}, {0, 0, 0.000379f}}};
    math3 inv = math3_inverse(diag);
    printf("[A] input  diag = %.6f %.6f %.6f\n",
           diag.matrix[0][0], diag.matrix[1][1], diag.matrix[2][2]);
    printf("[A] output inv  = %.4f %.4f %.4f\n",
           inv.matrix[0][0], inv.matrix[1][1], inv.matrix[2][2]);

    float expected_x = 1.0f / 0.000625f;   /* 1600 */
    float expected_y = 1.0f / 0.000379f;   /* ~2638 */

    int test_a_pass = (fabsf(inv.matrix[0][0] - expected_x) < 1.0f) &&
                      (fabsf(inv.matrix[1][1] - expected_y) < 1.0f) &&
                      (fabsf(inv.matrix[2][2] - expected_y) < 1.0f);

    if (test_a_pass) {
        printf("[A] PASS: math3_inverse works on diagonal matrix\n");
    } else {
        printf("[A] FAIL: math3_inverse returned wrong/zero result\n");
    }

    /* --- Test B: air-wheel probe (should spin after 093d) --- */
    printf("\n--- Test B: air-wheel probe after 093d ---\n");
    physics_world world;
    physics_world_init(&world);
    world.next_object_id = 1;

    int w = physics_world_add_cylinder(&world, 0.05f, 0.02f, 0.5f,
                                       (vector3){0.0f, 2.0f, 0.0f});
    if (w < 0) { printf("[B] FAIL: could not create cylinder\n"); return 1; }

    rigidbody *rb = &world.bodies[w];
    printf("[B] I_local[0][0]=%.6f I^-1_local[0][0]=%.4f I^-1_sys[0][0]=%.4f\n",
           rb->inertia_tensor_local.matrix[0][0],
           rb->inverse_inertia_tensor_local.matrix[0][0],
           rb->inverse_inertia_system.matrix[0][0]);

    const float dt = 1.0f / 60.0f;
    for (int t = 0; t < 10; t++) {
        rigidbody_wake(rb);
        rb->torque_accumulator.x += 0.25f;
        physics_world_step(&world, dt);
    }

    printf("[B] after 10 steps: wx=%.4f y=%.4f\n",
           rb->angular_velocity.x, rb->position.y);

    int test_b_pass = fabsf(rb->angular_velocity.x) > 0.5f;
    if (test_b_pass) {
        printf("[B] PASS: air wheel spun up — sanitize fix resolved inertia\n");
    } else {
        printf("[B] FAIL: air wheel still not spinning — math3_inverse is broken\n");
    }

    /* Overall */
    if (test_a_pass && test_b_pass) {
        printf("[PASS] 093e: inertia pipeline works\n");
        return 0;
    } else if (test_a_pass && !test_b_pass) {
        printf("[DIAG] math3_inverse is fine but something else zeroes I^-1\n");
        return 0; /* non-gating */
    } else {
        printf("[DIAG] math3_inverse is broken — needs fix in math3D.c\n");
        return 0; /* non-gating */
    }
}

#endif /* MPE_MATH3_INVERSE_TEST */
