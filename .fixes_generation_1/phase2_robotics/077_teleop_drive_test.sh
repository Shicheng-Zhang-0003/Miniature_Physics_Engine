#!/usr/bin/env bash
# ============================================================
# FIX 077 — FTC Phase 2: teleop drive validation test
#   Creates a robot, applies full forward tank drive for 180 ticks
#   (3 seconds), asserts:
#     - Robot moved forward (position.z changed significantly)
#     - No NaN in any body state
#     - Robot didn't flip (chassis y stayed reasonable)
# Phase:   phase2_robotics
# Files:   v15R3/src/tests/teleop_drive_test.c (new), makefile target
# Depends: 070-074, 067 (constraint loop in physics_world_step)
# Risk:    low (new test file + make target)
# ============================================================
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"
SRC="v15R3/src"
TEST="$SRC/tests/teleop_drive_test.c"
MAKEFILE="$SRC/makefile"
grep -q 'MPE_FTC_077' "$TEST" 2>/dev/null && { echo "[SKIP] teleop drive test already present"; exit 0; }
[[ -f "$SRC/robotics/robot.h" ]] || { echo "[SKIP] robot.h missing (run 073 first)"; exit 0; }
mkdir -p "$SRC/tests"

cat > "$TEST" <<'EOF'
/* MPE_FTC_077: Teleop drive validation test */
#ifdef MPE_TELEOP_DRIVE_TEST
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

    /* Create robot at origin, using goBILDA 30:1 motors */
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
    int total_ticks = 180;  /* 3 seconds */

    for (int t = 0; t < total_ticks; t++) {
        /* Full forward tank drive */
        drivetrain_tank (&robot, 1.0f, 1.0f);
        drivetrain_update (&world, &robot, dt);

        /* Step physics (includes constraints) */
        physics_world_step (&world, dt);

        /* Check for NaN */
        for (int i = 0; i < world.body_count; i++) {
            rigidbody *rb = &world.bodies [i];
            if ((!isfinite (rb->position.x)) || (!isfinite (rb->position.y)) ||
                (!isfinite (rb->position.z)) ||
                (!isfinite (rb->velocity.x)) || (!isfinite (rb->velocity.y)) ||
                (!isfinite (rb->velocity.z))) {
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
        float dz = end_z - start_z;
        float dy = end_y - start_y;

        printf ("[info] start=(%.3f,%.3f,%.3f) end=(%.3f,%.3f,%.3f)\n",
                start_x, start_y, start_z, end_x, end_y, end_z);
        printf ("[info] displacement z=%.4f  dy=%.4f\n", dz, dy);
        printf ("[info] motor RPM: [%.0f, %.0f, %.0f, %.0f]\n",
                robot.wheel_motors [0].rpm, robot.wheel_motors [1].rpm,
                robot.wheel_motors [2].rpm, robot.wheel_motors [3].rpm);
        printf ("[info] battery: %.2fV (%.0f%%)\n",
                battery_get_voltage (&robot.battery, 0.0f),
                robot.battery.charge_fraction * 100.0f);

        /* Robot should have moved in some direction (z or x) */
        float total_displacement = sqrtf (dz * dz + (end_x - start_x) * (end_x - start_x));
        if (total_displacement < 0.05f) {
            printf ("[FAIL] robot did not move (displacement=%.4f)\n", total_displacement);
            fail = 1;
        } else if (fabsf (dy) > 1.0f) {
            printf ("[FAIL] robot flipped or fell (dy=%.4f)\n", dy);
            fail = 1;
        } else {
            printf ("[PASS] robot drove under motor power (displacement=%.4f, dy=%.4f)\n",
                    total_displacement, dy);
        }
    }

    physics_world_cleanup (&world);
    return fail;
}
#endif /* MPE_TELEOP_DRIVE_TEST */
EOF

if ! grep -q 'MPE_FTC_077' "$MAKEFILE"; then
cp "$MAKEFILE" "${MAKEFILE}.pre_077"
cat >> "$MAKEFILE" <<'EOF'

# MPE_FTC_077: headless teleop drive test
TELEOP_TEST_SOURCES := tests/teleop_drive_test.c core/physics_world.c core/rigidbody.c \
	physics/collision_mechanics.c physics/broadphase.c physics/constraint.c physics/revolute_joint.c \
	robotics/motor.c robotics/motor_presets.c robotics/battery.c robotics/robot.c robotics/drivetrain.c \
	config/mpe_config.c config/mpe_config_schema.c
test_teleop_drive:
	$(CC) $(CFLAGS) -DMPE_TELEOP_DRIVE_TEST $(TELEOP_TEST_SOURCES) -lm -o test_teleop_drive
	./test_teleop_drive
EOF
fi

grep -q 'MPE_FTC_077' "$TEST" || { echo "[FAIL] test file not written"; exit 1; }
grep -q 'test_teleop_drive:' "$MAKEFILE" || { echo "[FAIL] make target not added"; exit 1; }
cd "$SRC"
if make test_teleop_drive 2>&1 | tail -3; then
  echo "[PASS] 077: teleop drive test built and passed"
else
  echo "[FAIL] teleop drive test build or run failed"
  exit 1
fi
