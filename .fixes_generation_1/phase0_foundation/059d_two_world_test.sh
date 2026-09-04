#!/usr/bin/env bash
# ============================================================
# FIX 059d — TEST-002 (first): two-world independence test
#   World A is stepped 600 ticks; world B is never stepped and its
#   body must not move by even 0.1mm. Any drift proves shared-state
#   leakage between worlds. This is the behavioural gate that the
#   retired 050 file-existence check pretended to be.
# Phase:   phase0_foundation
# Files:   v15R3/src/tests/two_world_test.c (new)
#          v15R3/src/makefile (new target)
# Depends: 055, 056, 059, 059c
# Risk:    low (additive file + appended make target)
# ============================================================
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"
SRC="v15R3/src"
TEST="$SRC/tests/two_world_test.c"
MAKEFILE="$SRC/makefile"
grep -q 'MPE_FTC_059C' "$SRC/core/physics_world.c" 2>/dev/null || { echo "[SKIP] 059c not applied yet"; exit 0; }

mkdir -p "$SRC/tests"
if [[ ! -f "$TEST" ]] || ! grep -q 'MPE_FTC_059D' "$TEST"; then
cat > "$TEST" <<'EOF'
/* MPE_FTC_059D: two-world independence test. Built via `make test_two_world`. */
#ifdef MPE_TWO_WORLD_TEST
#include <stdio.h>
#include <math.h>
#include "core/physics_world.h"
#include "config/mpe_config.h"

int main (void) {
    mpe_config_init ();
    physics_world world_a;
    physics_world world_b;
    physics_world_init (&world_a);
    physics_world_init (&world_b);
    world_b.next_object_id = 1000; /* keep contact-cache IDs disjoint */
    physics_world_add_sphere (&world_a, 0.5f, 1.0f, (vector3) {0.0f, 10.0f, 0.0f});
    physics_world_add_cube (&world_b, (vector3) {50.0f, 10.0f, 50.0f}, (vector3) {0.5f, 0.5f, 0.5f}, 2.0f);
    vector3 b_initial_position = world_b.bodies [0].position;
    const float dt = 1.0f / 60.0f;
    int fail = 0;
    for (int t = 0; t < 600; t++) {
        physics_world_step (&world_a, dt);
        rigidbody *rb_b = &world_b.bodies [0];
        float drift = vector3_length (vector3_subtraction (rb_b->position, b_initial_position));
        if (drift > 0.0001f) {
            printf ("[FAIL] world B drifted on tick %d (drift=%.6f)\n", t, drift);
            fail = 1;
            break;
        }
        if ((!isfinite (rb_b->position.x)) || (!isfinite (rb_b->position.y)) || (!isfinite (rb_b->position.z))) {
            printf ("[FAIL] world B went non-finite on tick %d\n", t);
            fail = 1;
            break;
        }
    }
    if (!fail) {
        if (world_a.bodies [0].position.y < 9.0f) {
            printf ("[PASS] two worlds independent: A stepped (sphere y=%.3f), B untouched\n",
                    world_a.bodies [0].position.y);
        } else {
            printf ("[FAIL] world A sphere did not fall (y=%.3f)\n", world_a.bodies [0].position.y);
            fail = 1;
        }
    }
    physics_world_cleanup (&world_a);
    physics_world_cleanup (&world_b);
    return fail;
}
#endif /* MPE_TWO_WORLD_TEST */
EOF
fi

if ! grep -q 'MPE_FTC_059D' "$MAKEFILE"; then
cp "$MAKEFILE" "${MAKEFILE}.pre_059d"
cat >> "$MAKEFILE" <<'EOF'

# MPE_FTC_059D: headless two-world independence test
TWO_WORLD_SOURCES := tests/two_world_test.c core/physics_world.c core/rigidbody.c \
	physics/collision_mechanics.c physics/broadphase.c \
	config/mpe_config.c config/mpe_config_schema.c
test_two_world:
	$(CC) $(CFLAGS) -DMPE_TWO_WORLD_TEST $(TWO_WORLD_SOURCES) -lm -o test_two_world
	./test_two_world
EOF
fi

# Postflight: actually run it
grep -q 'MPE_FTC_059D' "$TEST" || { echo "[FAIL] test file not written"; exit 1; }
grep -q 'test_two_world:' "$MAKEFILE" || { echo "[FAIL] make target not added"; exit 1; }
cd "$SRC"
if make test_two_world 2>&1 | tail -3; then
  echo "[PASS] 059d: two-world independence test built and passed"
else
  echo "[FAIL] two-world test build or run failed"
  exit 1
fi
