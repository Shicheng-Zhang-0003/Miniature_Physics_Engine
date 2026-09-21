#!/usr/bin/env bash
# MFS FTC test build + run (parked code, ported to the v15S core).
# Compiles entirely OUT-OF-TREE: no engine file is touched. Run from v15S/src:
#   ../robotics_backup/build_tests.sh [--build-only]
set -euo pipefail
SRC="$(cd "$(dirname "$0")/../src" && pwd)"
RB="$SRC/../robotics_backup"
OUT="${OUTDIR:-/tmp/ftc_tests}"
mkdir -p "$OUT"

CFLAGS="-I$SRC -I$(dirname "$SRC") -O2 -Wall -Wextra -ffp-contract=off"
CORE="core/physics_world.c core/rigidbody.c core/mpe_registry.c core/mpe_loader.c core/det_math.c core/mpe_primary.c physics/collision_narrowphase.c physics/collision_cache.c physics/collision_solver.c physics/collision_ccd.c physics/collision_cylinder.c physics/broadphase.c physics/constraint.c physics/revolute_joint.c physics/depenetration.c physics/islands.c config/mpe_config.c config/mpe_config_schema.c scene/boundary.c"
FTC="$RB/robotics/robot.c $RB/robotics/drivetrain.c $RB/robotics/motor.c $RB/robotics/motor_presets.c $RB/robotics/battery.c $RB/robotics/ftc_fleet.c $RB/robotics/ftc_module.c"

cd "$SRC"
pass=0; fail=0
run_test() { # name, -Dflag, file
    local name="$1" flag="$2" file="$3"
    if gcc $CFLAGS "-D$flag" "$file" $FTC $CORE -lm -ldl -rdynamic -o "$OUT/$name" 2>"$OUT/$name.build.log"; then
        if [ "${1:-}" != "--build-only" ] && [ "${BUILD_ONLY:-0}" != "1" ]; then
            if "$OUT/$name" >"$OUT/$name.run.log" 2>&1; then
                echo "[PASS] $name"; pass=$((pass+1));
            else
                echo "[FAIL] $name (exit $?)"; fail=$((fail+1)); tail -n 5 "$OUT/$name.run.log";
            fi
        else
            echo "[BUILD-OK] $name"; pass=$((pass+1));
        fi
    else
        echo "[BUILD-FAIL] $name"; fail=$((fail+1)); head -n 10 "$OUT/$name.build.log";
    fi
}

if [ "${1:-}" = "--build-only" ]; then BUILD_ONLY=1; fi

echo "--- FTC module plugin (hot-plug .so; must precede ftc_hotload) ---"
mkdir -p "$RB/plugins"
FTC_MOD="$RB/robotics/robot.c $RB/robotics/drivetrain.c $RB/robotics/motor.c $RB/robotics/motor_presets.c $RB/robotics/battery.c $RB/robotics/ftc_fleet.c $RB/robotics/ftc_module.c"
if gcc $CFLAGS -fPIC -shared $FTC_MOD -lm -o "$RB/plugins/mpe_ftc.so" 2>"$OUT/mpe_ftc.build.log"; then
    echo "[BUILD-OK] plugins/mpe_ftc.so"; pass=$((pass+1));
else
    echo "[BUILD-FAIL] plugins/mpe_ftc.so"; fail=$((fail+1)); head -n 10 "$OUT/mpe_ftc.build.log";
fi

RB_T="$RB/tests_robotics"
export FTC_SO="$RB/plugins/mpe_ftc.so"
run_test teleop MPE_TELEOP_DRIVE_TEST $RB_T/teleop_drive_test.c
run_test mecanum MPE_MECANUM_DRIVE_TEST $RB_T/mecanum_drive_test.c
run_test tank MFS_TANK_TURN_TEST $RB_T/tank_turn_test.c
run_test odometry MFS_ODOMETRY_ACCURACY_TEST $RB_T/odometry_accuracy_test.c
run_test ftc_integration MPE_FTC_INTEGRATION_TEST $RB_T/ftc_integration_test.c
run_test ftc_debug MPE_FTC_DEBUG_TEST $RB_T/ftc_debug_test.c
run_test physics_truth MPE_PHYSICS_TRUTH_TEST $RB_T/physics_truth_test.c
run_test physics_truth_diag MPE_PHYSICS_TRUTH_DIAG $RB_T/physics_truth_diag.c
run_test idle_spin MFS_IDLE_DIAG $RB_T/idle_spin_diag.c
run_test idle_spin_deep MFS_IDLE_DEEP_DIAG $RB_T/idle_spin_deep_diag.c
run_test idle_rootcause MFS_IDLE_ROOTCAUSE_DIAG $RB_T/idle_rootcause_diag.c
run_test odometry_diag MFS_ODOM_DIAG $RB_T/odometry_diag.c
run_test ftc_hotload MPE_FTC_HOTLOAD_TEST $RB_T/ftc_hotload_test.c

echo "--- compile-only units ---"
for u in "$RB/robotics/gui_robot_registry.c" "$RB/gamepad/gamepad.c"; do
    n=$(basename $u .c)
    if gcc $CFLAGS -DMPE_GTK4=1 $(pkg-config --cflags gtk4 epoxy 2>/dev/null) -c "$u" -o "$OUT/$n.o" 2>"$OUT/$n.build.log"; then
        echo "[BUILD-OK] $n"; pass=$((pass+1));
    else
        echo "[BUILD-FAIL] $n"; fail=$((fail+1)); head -n 10 "$OUT/$n.build.log";
    fi
done
echo "FTC RESULT: pass=$pass fail=$fail"
[ "$fail" -eq 0 ]
