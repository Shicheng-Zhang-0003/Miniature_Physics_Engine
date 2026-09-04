#!/usr/bin/env bash
# ============================================================
# FIX 093c — Diagnose: air-wheel torque probe
#   Applies torque to a cylinder suspended in the air (no floor
#   contact for the first ~10 steps). Isolates whether the torque
#   integration itself works (suspects: sleeping, zeroed inertia)
#   separately from floor-contact effects.
#   READ-ONLY diagnostic: rewrites the test, non-gating.
#
# Phase:   phase3_sensors (cylinder keystone)
# Files:   v15R2/src/tests/driven_wheel_test.c (rewrite)
# Depends: 093b
# Risk:    low (test-only)
# ============================================================
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"

SRC="v15R2/src"
TEST="$SRC/tests/driven_wheel_test.c"
MAKEFILE="$SRC/makefile"

[[ -d "$SRC/tests" ]] || { echo "[SKIP] $SRC/tests not found"; exit 0; }
cp "$TEST" "${TEST}.pre_093c" 2>/dev/null || true

cat > "$TEST" << 'TEST_EOF'
/* MPE_FTC_093c: Air-wheel torque probe.
 * A cylinder is suspended at y=2.0 (no floor contact for ~10 steps).
 * Torque is applied about its axle each step. We print, per step:
 *   - torque_accumulator.x before the step (is the torque present?)
 *   - is_sleeping before the step (is integration being skipped?)
 *   - angular_velocity.x before/after (did torque produce spin?)
 *   - inverse_inertia_system[0][0] (is the axle inertia zeroed?)
 * If the wheel spins in the air, torque integration is fine and the
 * floor contact is the culprit. If it does not spin, integration or
 * inertia is broken. */
#ifdef MPE_DRIVEN_WHEEL_TEST

#include <math.h>
#include <stdio.h>

#include "config/mpe_config.h"
#include "core/physics_world.h"
#include "core/rigidbody.h"

int main(void) {
    mpe_config_init();
    printf("[info] gravity = %.4f\n", g_cfg.world.gravity);

    physics_world world;
    physics_world_init(&world);
    world.next_object_id = 1;

    /* Cylinder high in the air — no floor contact for the probe window. */
    int w = physics_world_add_cylinder(&world, 0.05f, 0.02f, 0.5f,
                                       (vector3){0.0f, 2.0f, 0.0f});
    if (w < 0) { printf("[FAIL] could not create air wheel\n"); return 1; }

    rigidbody *rb = &world.bodies[w];
    printf("[init] mass=%.4f static=%d sleeping=%d\n",
           rb->mass, rb->static_state, rb->is_sleeping);
    printf("[init] I_local[0][0]=%.6f I^-1_local[0][0]=%.4f I^-1_sys[0][0]=%.4f\n",
           rb->inertia_tensor_local.matrix[0][0],
           rb->inverse_inertia_tensor_local.matrix[0][0],
           rb->inverse_inertia_system.matrix[0][0]);

    const float dt = 1.0f / 60.0f;
    for (int t = 0; t < 10; t++) {
        rigidbody_wake(rb);
        rb->torque_accumulator.x += 0.25f;

        float pre_torque = rb->torque_accumulator.x;
        float pre_wx     = rb->angular_velocity.x;
        int   pre_sleep  = rb->is_sleeping;
        float pre_iinv   = rb->inverse_inertia_system.matrix[0][0];

        physics_world_step(&world, dt);

        printf("[step %d] y=%.4f torque=%.3f sleep=%d I^-1=%.4f | wx %.4f -> %.4f\n",
               t, rb->position.y, pre_torque, pre_sleep, pre_iinv,
               pre_wx, rb->angular_velocity.x);

        if (!isfinite(rb->angular_velocity.x) || !isfinite(rb->position.y)) {
            printf("[FAIL] NaN/Inf\n");
            return 1;
        }
    }

    float final_wx = world.bodies[w].angular_velocity.x;
    if (fabsf(final_wx) > 0.5f) {
        printf("[PASS] air wheel spun up to wx=%.4f — torque integration works; floor contact is the culprit\n", final_wx);
    } else {
        printf("[GAP] air wheel did NOT spin (wx=%.4f) — torque integration or cylinder inertia is broken\n", final_wx);
    }
    return 0; /* non-gating */
}

#endif /* MPE_DRIVEN_WHEEL_TEST */
TEST_EOF

grep -q 'MPE_FTC_093c' "$TEST" || { echo "[FAIL] test not rewritten"; exit 1; }

if ! grep -q 'test_driven_wheel:' "$MAKEFILE"; then
    echo "[FAIL] test_driven_wheel target missing (run 093a first)"; exit 1
fi

cd "$SRC"
if make test_driven_wheel > /tmp/airwheel_093c.log 2>&1; then
    echo "----- air-wheel torque probe -----"
    grep -E '\[info\]|\[init\]|\[step|\[PASS\]|\[GAP\]|\[FAIL\]' /tmp/airwheel_093c.log || true
    echo "----------------------------------"
    echo "[PASS] 093c: air-wheel probe completed"
else
    tail -20 /tmp/airwheel_093c.log
    echo "[FAIL] 093c: probe failed to build or crashed"
    exit 1
fi
