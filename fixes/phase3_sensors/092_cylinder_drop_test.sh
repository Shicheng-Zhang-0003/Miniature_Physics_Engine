#!/usr/bin/env bash
# ============================================================
# FIX 092 — FTC Phase 3: Cylinder drop test (test-first diagnostic)
#   Creates a headless test that drops a cylinder (wheel) and a
#   control sphere onto a static floor. Establishes the success
#   criterion for cylinder contact (093).
#
#   EXPECTED RESULT NOW: the sphere rests on the floor, but the
#   cylinder falls through (no cylinder narrowphase yet). This
#   script reports that gap WITHOUT failing the fleet. Script 093
#   implements cylinder contact and will require this test to PASS.
#
# Phase:   phase3_sensors (cylinder keystone)
# Files:   v15R2/src/tests/cylinder_drop_test.c (new)
#          v15R2/src/makefile (new target)
# Depends: 091
# Risk:    low (additive test)
# ============================================================
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"

SRC="v15R2/src"
TEST="$SRC/tests/cylinder_drop_test.c"
MAKEFILE="$SRC/makefile"

[[ -d "$SRC/tests" ]] || { echo "[SKIP] $SRC/tests not found"; exit 0; }
grep -q 'MPE_FTC_092' "$TEST" 2>/dev/null && { echo "[SKIP] cylinder drop test already present"; exit 0; }

# ---- Write the test ----
cat > "$TEST" << 'TEST_EOF'
/* MPE_FTC_092: Cylinder drop test.
 *
 * Drops a cylinder (a wheel) and a control sphere onto a static floor.
 * Establishes the success criterion for cylinder contact (093).
 *
 * Expected until 093 lands: the SPHERE rests on the floor while the
 * CYLINDER falls straight through it, because the narrowphase has no
 * cylinder case. */
#ifdef MPE_CYLINDER_DROP_TEST

#include <stdio.h>
#include <math.h>
#include "../core/physics_world.h"

int main(void) {
    physics_world world;
    physics_world_init(&world);
    world.next_object_id = 1;

    /* Static floor: large flat cube, top surface at y = 0 (mass 0 = static). */
    int floor_idx = physics_world_add_cube(&world,
        (vector3){0.0f, -0.5f, 0.0f},     /* center */
        (vector3){10.0f, 0.5f, 10.0f},    /* half extents -> top at y=0 */
        0.0f);

    /* Cylinder wheel: radius 0.05, half-length 0.02, mass 0.5.
     * Center spawned 0.25 above the floor top. */
    int cyl_idx = physics_world_add_cylinder(&world,
        0.05f, 0.02f, 0.5f,
        (vector3){0.0f, 0.25f, 0.0f});

    /* Control sphere: same radius and spawn height as the cylinder. */
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

    /* Sphere sanity check: it should rest near y = radius (0.05). */
    if (sph_y < -1.0f) {
        printf("[WARN] control sphere fell too — no floor contact in this world?\n");
    }

    /* A resting cylinder sits with center near (floor_top + radius) = 0.05,
     * allowing penetration slop. A falling one is far below after 5s. */
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
TEST_EOF

grep -q 'MPE_FTC_092' "$TEST" || { echo "[FAIL] test file not written"; exit 1; }

# ---- Add make target (guarded) ----
if ! grep -q 'test_cylinder_drop:' "$MAKEFILE"; then
cat >> "$MAKEFILE" << 'MK_EOF'

# MPE_FTC_092: cylinder drop test
CYLINDER_DROP_TEST_SOURCES = tests/cylinder_drop_test.c core/physics_world.c core/rigidbody.c physics/collision_mechanics.c physics/broadphase.c physics/constraint.c physics/revolute_joint.c config/mpe_config.c config/mpe_config_schema.c

test_cylinder_drop: $(CYLINDER_DROP_TEST_SOURCES)
	$(CC) $(CFLAGS) -DMPE_CYLINDER_DROP_TEST $(CYLINDER_DROP_TEST_SOURCES) -lm -o test_cylinder_drop
	./test_cylinder_drop
MK_EOF
fi

grep -q 'test_cylinder_drop:' "$MAKEFILE" || { echo "[FAIL] make target not added"; exit 1; }

# ---- Build (must succeed) ----
cd "$SRC"
if ! make test_cylinder_drop > /tmp/cyl_drop_build.log 2>&1; then
    # the target both builds AND runs; a failing run is expected, so
    # distinguish a compile error from a failing test
    if grep -qE 'error:|Error [0-9]' /tmp/cyl_drop_build.log && ! grep -q '\[info\] cylinder' /tmp/cyl_drop_build.log; then
        tail -15 /tmp/cyl_drop_build.log
        echo "[FAIL] 092: cylinder drop test failed to compile"
        exit 1
    fi
fi

# ---- Run and report (failure here is the EXPECTED gap) ----
if ./test_cylinder_drop; then
    echo "[PASS] 092: cylinder already rests on floor (contact present)"
else
    echo "[INFO] 092: gap confirmed — cylinder falls through floor."
    echo "       This is expected. Script 093 will implement cylinder contact"
    echo "       and require this test to pass."
fi
