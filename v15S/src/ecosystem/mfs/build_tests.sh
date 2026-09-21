#!/usr/bin/env bash
# MFS overarching ecosystem: FTC/robotics test build + run.
# Everything MFS-wise lives under v15S/src/ecosystem/mfs/ (modules,
# submodules, tests, docs). Run from v15S/src:
#   ecosystem/mfs/build_tests.sh [--build-only]
set -euo pipefail
MFS="$(cd "$(dirname "$0")" && pwd)"
SRC="$(cd "$MFS/../.." && pwd)"
OUT="${OUTDIR:-/tmp/ftc_tests}"
mkdir -p "$OUT"

CFLAGS="-I$SRC -I$MFS -O2 -Wall -Wextra -ffp-contract=off"
CORE="core/physics_world.c core/rigidbody.c core/mpe_registry.c core/mpe_loader.c core/det_math.c core/mpe_primary.c physics/collision_narrowphase.c physics/collision_cache.c physics/collision_solver.c physics/collision_ccd.c physics/collision_cylinder.c physics/broadphase.c physics/constraint.c physics/revolute_joint.c physics/depenetration.c physics/islands.c config/mpe_config.c config/mpe_config_schema.c scene/boundary.c"
FTC="ecosystem/mfs/modules/ftc/ftc_module.c ecosystem/mfs/modules/ftc/ftc_fleet.c ecosystem/mfs/modules/ftc/submodules/robot.c ecosystem/mfs/modules/ftc/submodules/drivetrain.c ecosystem/mfs/modules/ftc/submodules/motor.c ecosystem/mfs/modules/ftc/submodules/motor_presets.c ecosystem/mfs/modules/ftc/submodules/battery.c"

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
mkdir -p "$MFS/plugins" "$SRC/plugins"
FTC_MOD="$FTC"
if gcc $CFLAGS -fPIC -shared $FTC_MOD -lm -o "$MFS/plugins/mpe_ftc.so" 2>"$OUT/mpe_ftc.build.log"; then
    echo "[BUILD-OK] mfs/plugins/mpe_ftc.so"; pass=$((pass+1));
else
    echo "[BUILD-FAIL] mfs/plugins/mpe_ftc.so"; fail=$((fail+1)); head -n 10 "$OUT/mpe_ftc.build.log";
fi
# The kernel loader only accepts plugins/<name>.so under its CWD, so stage
# a copy at the jailed location too (same binary; gitignored build output).
cp "$MFS/plugins/mpe_ftc.so" "$SRC/plugins/mpe_ftc.so"

RB_T="$MFS/tests"
export FTC_SO="plugins/mpe_ftc.so"
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
for u in "$MFS/modules/ftc/gui_robot_registry.c" "$MFS/modules/module_1/submodules/gamepad/gamepad.c"; do
    n=$(basename $u .c)
    if gcc $CFLAGS -DMPE_GTK4=1 $(pkg-config --cflags gtk4 epoxy 2>/dev/null) -c "$u" -o "$OUT/$n.o" 2>"$OUT/$n.build.log"; then
        echo "[BUILD-OK] $n"; pass=$((pass+1));
    else
        echo "[BUILD-FAIL] $n"; fail=$((fail+1)); head -n 10 "$OUT/$n.build.log";
    fi
done
echo "FTC RESULT: pass=$pass fail=$fail"
[ "$fail" -eq 0 ]
