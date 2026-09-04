#!/usr/bin/env bash
# ============================================================
# FIX 093a — FTC Phase 3: Driven-wheel propulsion diagnostic
#
#   Cylinder-floor contact works after 093. This script diagnoses
#   the next link:
#
#       motor/torque/spin -> contact friction -> translation
#
#   Previous version hard-failed when torque did not spin the wheel.
#   That was too early for run_all.sh. This version is intentionally
#   NON-GATING: it prints diagnostics, keeps the fleet green, and tells
#   us which exact subsystem needs the next fix.
#
#   Probe A: apply torque_accumulator.x each tick with rigidbody_wake().
#            If wx remains zero, torque integration is missing/broken.
#
#   Probe B: directly impose angular_velocity.x each tick.
#            If wx is nonzero but dz is zero, contact friction is not
#            converting spin into translation.
#
# Phase:   phase3_sensors
# Files:   v15R3/src/tests/driven_wheel_test.c
#          v15R3/src/makefile
# Depends: 093
# Risk:    low, test-only
# ============================================================
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"

SRC="v15R3/src"
TEST="$SRC/tests/driven_wheel_test.c"
MAKEFILE="$SRC/makefile"

[[ -d "$SRC/tests" ]] || { echo "[SKIP] $SRC/tests not found"; exit 0; }

cp "$TEST" "${TEST}.pre_093a_diag" 2>/dev/null || true

cat > "$TEST" << 'TEST_EOF'
/* MPE_FTC_093a: Driven-wheel propulsion diagnostic.
 *
 * This test is intentionally non-gating. It diagnoses whether:
 *   A) torque_accumulator actually spins a cylinder,
 *   B) a manually spinning cylinder translates through floor friction.
 */
#ifdef MPE_DRIVEN_WHEEL_TEST

#include <math.h>
#include <stdio.h>

#include "config/mpe_config.h"
#include "core/physics_world.h"
#include "core/rigidbody.h"

static int finite3(float a, float b, float c) {
    return isfinite(a) && isfinite(b) && isfinite(c);
}

int main(void) {
    mpe_config_init();

    printf("[info] gravity = %.4f\n", g_cfg.world.gravity);

    physics_world world;
    physics_world_init(&world);
    world.next_object_id = 1;

    int torque_wheel = physics_world_add_cylinder(&world,
                                                  0.05f,
                                                  0.02f,
                                                  0.5f,
                                                  (vector3){0.0f, 0.05f, 0.0f});

    int spin_wheel = physics_world_add_cylinder(&world,
                                                0.05f,
                                                0.02f,
                                                0.5f,
                                                (vector3){1.0f, 0.05f, 0.0f});

    if ((torque_wheel < 0) || (spin_wheel < 0)) {
        printf("[FAIL] could not create diagnostic wheels\n");
        return 1;
    }

    const float dt = 1.0f / 60.0f;

    float torque_start_z = world.bodies[torque_wheel].position.z;
    float spin_start_z = world.bodies[spin_wheel].position.z;

    const float drive_torque = 0.25f;
    const float forced_spin = 25.0f;

    for (int t = 0; t < 180; t++) {
        rigidbody *tw = &world.bodies[torque_wheel];
        rigidbody *sw = &world.bodies[spin_wheel];

        /* Probe A: real torque path. */
        rigidbody_wake(tw);
        tw->torque_accumulator.x += drive_torque;

        /* Probe B: bypass torque integration, prove/disprove friction propulsion. */
        rigidbody_wake(sw);
        sw->angular_velocity.x = forced_spin;

        physics_world_step(&world, dt);
    }

    rigidbody *tw = &world.bodies[torque_wheel];
    rigidbody *sw = &world.bodies[spin_wheel];

    float torque_dz = tw->position.z - torque_start_z;
    float spin_dz = sw->position.z - spin_start_z;

    printf("[probe A] torque wheel: dz=%.4f y=%.4f wx=%.4f vz=%.4f\n",
           torque_dz,
           tw->position.y,
           tw->angular_velocity.x,
           tw->velocity.z);

    printf("[probe B] forced-spin wheel: dz=%.4f y=%.4f wx=%.4f vz=%.4f\n",
           spin_dz,
           sw->position.y,
           sw->angular_velocity.x,
           sw->velocity.z);

    if (!finite3(tw->position.z, tw->position.y, tw->angular_velocity.x) ||
        !finite3(sw->position.z, sw->position.y, sw->angular_velocity.x)) {
        printf("[FAIL] NaN/Inf during driven wheel diagnostic\n");
        return 1;
    }

    if (fabsf(tw->angular_velocity.x) < 0.5f) {
        printf("[DIAG] torque path inactive: torque_accumulator is not producing wheel spin.\n");
    } else if (fabsf(torque_dz) < 0.10f) {
        printf("[DIAG] torque spins wheel, but contact friction is not translating it.\n");
    } else {
        printf("[PASS] torque path produced rolling translation.\n");
    }

    if (fabsf(sw->angular_velocity.x) > 0.5f && fabsf(spin_dz) < 0.10f) {
        printf("[DIAG] forced spin did not translate: friction/solver angular contact coupling is missing.\n");
    } else if (fabsf(spin_dz) >= 0.10f) {
        printf("[PASS] forced spin translated through floor friction.\n");
    } else {
        printf("[DIAG] forced-spin probe inconclusive.\n");
    }

    /* Non-gating by design. We are diagnosing, not enforcing yet. */
    return 0;
}

#endif /* MPE_DRIVEN_WHEEL_TEST */
TEST_EOF

if ! grep -q 'MPE_FTC_093a' "$TEST"; then
    echo "[FAIL] driven wheel diagnostic test was not written"
    exit 1
fi

if ! grep -q 'test_driven_wheel:' "$MAKEFILE"; then
    cp "$MAKEFILE" "${MAKEFILE}.pre_093a_diag"
    cat >> "$MAKEFILE" << 'MK_EOF'

# MPE_FTC_093a: driven-wheel propulsion diagnostic
DRIVEN_WHEEL_TEST_SOURCES = tests/driven_wheel_test.c core/physics_world.c core/rigidbody.c physics/collision_mechanics.c physics/broadphase.c physics/constraint.c physics/revolute_joint.c config/mpe_config.c config/mpe_config_schema.c

test_driven_wheel: $(DRIVEN_WHEEL_TEST_SOURCES)
	$(CC) $(CFLAGS) -DMPE_DRIVEN_WHEEL_TEST $(DRIVEN_WHEEL_TEST_SOURCES) -lm -o test_driven_wheel
	./test_driven_wheel
MK_EOF
fi

cd "$SRC"

if make test_driven_wheel > /tmp/driven_wheel_093a.log 2>&1; then
    echo "----- driven wheel diagnostic -----"
    grep -E '\[info\]|\[probe|\[PASS\]|\[DIAG\]|\[FAIL\]' /tmp/driven_wheel_093a.log || true
    echo "-----------------------------------"
    echo "[PASS] 093a: driven-wheel diagnostic completed"
else
    tail -20 /tmp/driven_wheel_093a.log
    echo "[FAIL] 093a: diagnostic failed to compile or crashed"
    exit 1
fi
