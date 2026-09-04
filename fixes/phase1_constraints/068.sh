#!/usr/bin/env bash
# ============================================================
# FIX 068 — TEST: headless revolute pendulum
#   A static pivot cube + a hanging bob joined by a revolute joint.
#   Asserts the anchor holds (rod length stays constant) and the bob
#   swings under gravity. This is the behavioural gate for 062/063/067.
# Phase:   phase1_constraints
# Files:   v15R3/src/tests/revolute_test.c (new), makefile (target)
# Depends: 062, 063, 067
# Risk:    low (additive file + appended make target)
# ============================================================
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"
SRC="v15R3/src"
TEST="$SRC/tests/revolute_test.c"
MAKEFILE="$SRC/makefile"
grep -q 'MPE_FTC_067' "$SRC/core/physics_world.c" 2>/dev/null || { echo "[SKIP] 067 not applied yet"; exit 0; }

mkdir -p "$SRC/tests"
if [[ ! -f "$TEST" ]] || ! grep -q 'MPE_FTC_068' "$TEST"; then
cat > "$TEST" <<'EOF'
/* MPE_FTC_068: revolute pendulum test. Built via `make test_revolute`. */
#ifdef MPE_REVOLUTE_TEST
#include <stdio.h>
#include <math.h>
#include "core/physics_world.h"
#include "physics/constraint.h"
#include "config/mpe_config.h"

int main (void) {
    mpe_config_init ();
    physics_world world;
    physics_world_init (&world);

    int pivot_index = physics_world_add_cube (&world, (vector3) {0.0f, 10.0f, 0.0f}, (vector3) {0.2f, 0.2f, 0.2f}, 1.0f);
    rigidbody_set_static (&world.bodies [pivot_index], true);
    uint32_t pivot_id = world.bodies [pivot_index].object_id;

    int bob_index = physics_world_add_sphere (&world, 0.3f, 2.0f, (vector3) {1.0f, 8.0f, 0.0f});
    uint32_t bob_id = world.bodies [bob_index].object_id;

    vector3 pivot_point = {0.0f, 10.0f, 0.0f};
    float rod_length = vector3_length (vector3_subtraction (pivot_point, world.bodies [bob_index].position));
    vector3 start_position = world.bodies [bob_index].position;

    constraint_pool_init ();
    vector3 anchor_a = {0.0f, 0.0f, 0.0f};        /* pivot centre -> world (0,10,0) */
    vector3 anchor_b = {-1.0f, 2.0f, 0.0f};       /* bob-local -> world (0,10,0)   */
    vector3 axis = {0.0f, 0.0f, 1.0f};            /* swing in the x-y plane        */
    int joint_index = constraint_add_revolute (pivot_id, bob_id, anchor_a, anchor_b, axis);
    if (joint_index < 0) { printf ("[FAIL] could not add revolute joint\n"); return 1; }

    const float dt = 1.0f / 60.0f;
    int fail = 0;
    float max_drift = 0.0f;
    for (int t = 0; t < 600; t++) {
        physics_world_step (&world, dt);
        rigidbody *bob = &world.bodies [bob_index];
        if ((!isfinite (bob->position.x)) || (!isfinite (bob->position.y)) || (!isfinite (bob->position.z))) {
            printf ("[FAIL] bob went non-finite at tick %d\n", t);
            fail = 1;
            break;
        }
        float dist = vector3_length (vector3_subtraction (pivot_point, bob->position));
        float drift = fabsf (dist - rod_length);
        if (drift > max_drift) { max_drift = drift; }
    }
    if (!fail) {
        rigidbody *bob = &world.bodies [bob_index];
        float moved = vector3_length (vector3_subtraction (bob->position, start_position));
        printf ("[info] rod=%.4f max_drift=%.4f moved=%.4f bob=(%.3f,%.3f,%.3f)\n",
                rod_length, max_drift, moved, bob->position.x, bob->position.y, bob->position.z);
        if (max_drift > 0.15f) {
            printf ("[FAIL] anchor drift %.4f too large — revolute not holding\n", max_drift);
            fail = 1;
        } else if (moved < 0.05f) {
            printf ("[FAIL] bob did not move — gravity or joint not acting\n");
            fail = 1;
        } else {
            printf ("[PASS] revolute pendulum holds (max drift %.4f) and swings under gravity\n", max_drift);
        }
    }
    physics_world_cleanup (&world);
    return fail;
}
#endif /* MPE_REVOLUTE_TEST */
EOF
fi

if ! grep -q 'MPE_FTC_068' "$MAKEFILE"; then
cp "$MAKEFILE" "${MAKEFILE}.pre_068"
cat >> "$MAKEFILE" <<'EOF'

# MPE_FTC_068: headless revolute pendulum test
REVOLUTE_TEST_SOURCES := tests/revolute_test.c core/physics_world.c core/rigidbody.c \
	physics/collision_mechanics.c physics/broadphase.c physics/constraint.c physics/revolute_joint.c \
	config/mpe_config.c config/mpe_config_schema.c
test_revolute:
	$(CC) $(CFLAGS) -DMPE_REVOLUTE_TEST $(REVOLUTE_TEST_SOURCES) -lm -o test_revolute
	./test_revolute
EOF
fi

grep -q 'MPE_FTC_068' "$TEST" || { echo "[FAIL] test file not written"; exit 1; }
grep -q 'test_revolute:' "$MAKEFILE" || { echo "[FAIL] make target not added"; exit 1; }
cd "$SRC"
if make test_revolute 2>&1 | tail -3; then
  echo "[PASS] 068: revolute pendulum test built and passed"
else
  echo "[FAIL] revolute pendulum test build or run failed"
  exit 1
fi
