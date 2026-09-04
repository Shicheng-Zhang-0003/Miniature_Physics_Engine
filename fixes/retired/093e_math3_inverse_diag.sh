#!/usr/bin/env bash
# ============================================================
# FIX 093e — Diagnose + repair: math3_inverse returns zero
#   The cylinder inertia inverse is zero even though the inertia
#   tensor is a valid diagonal matrix. This test:
#     A) Inverts a diagonal matrix directly (should give 1/diag).
#     B) Re-runs the air-wheel probe after 093d to see if fixing
#        sanitize alone resolves the issue.
#   If the direct inverse is still zero, math3_inverse is broken
#        and needs a fix in math3D.c.
#
# Phase:   phase3_sensors (cylinder keystone)
# Files:   v15R2/src/tests/math3_inverse_test.c (new)
#          v15R2/src/makefile (new target)
# Depends: 093d
# Risk:    low (additive test)
# ============================================================
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"

SRC="v15R2/src"
TEST="$SRC/tests/math3_inverse_test.c"
MAKEFILE="$SRC/makefile"

[[ -d "$SRC/tests" ]] || { echo "[SKIP] $SRC/tests not found"; exit 0; }
grep -q 'MPE_FTC_093e' "$TEST" 2>/dev/null && { echo "[SKIP] math3 inverse test already present"; exit 0; }

cat > "$TEST" << 'TEST_EOF'
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
TEST_EOF

grep -q 'MPE_FTC_093e' "$TEST" || { echo "[FAIL] test not written"; exit 1; }

if ! grep -q 'test_math3_inverse:' "$MAKEFILE"; then
cat >> "$MAKEFILE" << 'MK_EOF'

# MPE_FTC_093e: math3_inverse diagnostic
MATH3_INV_TEST_SOURCES = tests/math3_inverse_test.c core/physics_world.c core/rigidbody.c physics/collision_mechanics.c physics/broadphase.c physics/constraint.c physics/revolute_joint.c config/mpe_config.c config/mpe_config_schema.c core/math3D.c core/math4_special.c

test_math3_inverse: $(MATH3_INV_TEST_SOURCES)
	$(CC) $(CFLAGS) -DMPE_MATH3_INVERSE_TEST $(MATH3_INV_TEST_SOURCES) -lm -o test_math3_inverse
	./test_math3_inverse
MK_EOF
fi

cd "$SRC"
if make test_math3_inverse > /tmp/math3_inv_093e.log 2>&1; then
    echo "----- math3_inverse diagnostic -----"
    grep -E '\[A\]|\[B\]|\[PASS\]|\[DIAG\]|\[FAIL\]' /tmp/math3_inv_093e.log || true
    echo "------------------------------------"
    echo "[PASS] 093e: math3_inverse diagnostic completed"
else
    tail -20 /tmp/math3_inv_093e.log
    echo "[FAIL] 093e: diagnostic failed to build or crashed"
    exit 1
fi
