#!/usr/bin/env bash
# ============================================================
# FIX 080 — FTC Phase 2: mecanum drive validation test
#   Creates a robot, applies full strafe for 180 ticks (3 seconds),
#   asserts the robot moved sideways (x-axis displacement).
# Phase:   phase2_robotics
# Files:   v15R2/src/tests/mecanum_drive_test.c (new), makefile target
# Depends: 075, 076
# Risk:    low (new test file + make target)
# ============================================================
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"
SRC="v15R2/src"
TEST="$SRC/tests/mecanum_drive_test.c"
MAKEFILE="$SRC/makefile"
grep -q 'MPE_FTC_080' "$TEST" 2>/dev/null && { echo "[SKIP] mecanum test already present"; exit 0; }
[[ -f "$SRC/robotics/drivetrain.h" ]] || { echo "[SKIP] drivetrain.h missing (run 074 first)"; exit 0; }
mkdir -p "$SRC/tests"

cat > "$TEST" <<'EOF'
/* MPE_FTC_080: Mecanum drive validation test */
#ifdef MPE_MECANUM_DRIVE_TEST
#include <stdio.h>
#include <math.h>
#include "core/physics_world.h"
#include "physics/constraint.h"
#include "config/mpe_config.h"
#include "robotics/robot.h"
#include "robotics/drivetrain.h"

int main (void) {
    mpe_config_init ();
    physics_world world;
    physics_world_init (&world);
    constraint_pool_init ();

    ftc_robot robot;
    int rc = ftc_robot_create (&world, &robot, 0.0f, 0.5f, 0.0f, MOTOR_GB_5203_30);
    if (rc != 0) {
        printf ("[FAIL] could not create robot\n");
        return 1;
    }

    float start_x, start_y, start_z;
    ftc_robot_get_position (&world, &robot, &start_x, &start_y, &start_z);

    const float dt = 1.0f / 60.0f;
    int fail = 0;
    int total_ticks = 180;

    for (int t = 0; t < total_ticks; t++) {
        /* Full strafe right (forward=0, strafe=1, rotate=0) */
        drivetrain_mecanum (&robot, 0.0f, 1.0f, 0.0f);
        drivetrain_update (&world, &robot, dt);
        physics_world_step (&world, dt);

        for (int i = 0; i < world.body_count; i++) {
            rigidbody *rb = &world.bodies [i];
            if ((!isfinite (rb->position.x)) || (!isfinite (rb->position.y)) ||
                (!isfinite (rb->position.z))) {
                printf ("[FAIL] NaN detected in body %d at tick %d\n", i, t);
                fail = 1;
                break;
            }
        }
        if (fail) {break;}
    }

    if (!fail) {
        float end_x, end_y, end_z;
        ftc_robot_get_position (&world, &robot, &end_x, &end_y, &end_z);
        float dx = end_x - start_x;
        float dz = end_z - start_z;

        printf ("[info] start=(%.3f,%.3f,%.3f) end=(%.3f,%.3f,%.3f)\n",
                start_x, start_y, start_z, end_x, end_y, end_z);
        printf ("[info] displacement x=%.4f  z=%.4f\n", dx, dz);

        /* Robot should have moved sideways (x-axis) */
        float lateral_displacement = fabsf (dx);
        if (lateral_displacement < 0.3f) {
            printf ("[FAIL] robot did not strafe enough (dx=%.4f, expected >0.3)\n", dx);
            fail = 1;
        } else {
            printf ("[PASS] robot strafed under mecanum drive (dx=%.4f)\n", dx);
        }
    }

    physics_world_cleanup (&world);
    return fail;
}
#endif /* MPE_MECANUM_DRIVE_TEST */
EOF

if ! grep -q 'MPE_FTC_080' "$MAKEFILE"; then
cp "$MAKEFILE" "${MAKEFILE}.pre_080"
cat >> "$MAKEFILE" <<'EOF'

# MPE_FTC_080: headless mecanum drive test
MECANUM_TEST_SOURCES := tests/mecanum_drive_test.c core/physics_world.c core/rigidbody.c \
	physics/collision_mechanics.c physics/broadphase.c physics/constraint.c physics/revolute_joint.c \
	robotics/motor.c robotics/motor_presets.c robotics/battery.c robotics/robot.c robotics/drivetrain.c \
	robotics/wheel_traction.c config/mpe_config.c config/mpe_config_schema.c
test_mecanum_drive:
	$(CC) $(CFLAGS) -DMPE_MECANUM_DRIVE_TEST $(MECANUM_TEST_SOURCES) -lm -o test_mecanum_drive
	./test_mecanum_drive
EOF
fi

grep -q 'MPE_FTC_080' "$TEST" || { echo "[FAIL] test file not written"; exit 1; }
grep -q 'test_mecanum_drive:' "$MAKEFILE" || { echo "[FAIL] make target not added"; exit 1; }
cd "$SRC"
if make test_mecanum_drive 2>&1 | tail -3; then
  echo "[PASS] 080: mecanum drive test built and passed"
else
  echo "[FAIL] mecanum drive test build or run failed"
  exit 1
fi
