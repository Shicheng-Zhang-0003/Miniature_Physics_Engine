#!/usr/bin/env bash
# MFS overarching ecosystem: FTC/robotics test build + run (v2).
# Everything MFS-wise lives under v15S/src/ecosystem/mfs/ (modules,
# submodules, tests, docs). Run from v15S/src:
#   ecosystem/mfs/build_tests.sh [--build-only]
set -euo pipefail
MFS="$(cd "$(dirname "$0")" && pwd)"
SRC="$(cd "$MFS/../.." && pwd)"
ROOT="$(cd "$SRC/../.." && pwd)"
TMPDIR="$ROOT/temp"
export TMPDIR
export MPE_GAMEPAD_DEVICE=disabled
OUT="${OUTDIR:-$TMPDIR/ftc_tests}"
mkdir -p "$OUT"

TEST_CC="${MFS_TEST_CC:-gcc}"
CFLAGS="-I$SRC -I$MFS -O2 -Wall -Wextra -ffp-contract=off ${MFS_TEST_CFLAGS:-}"
CORE="core/physics_world.c core/rigidbody.c core/mpe_registry.c core/mpe_loader.c core/det_math.c core/mpe_primary.c physics/collision_narrowphase.c physics/collision_cache.c physics/collision_solver.c physics/collision_ccd.c physics/collision_cylinder.c physics/broadphase.c physics/constraint.c physics/revolute_joint.c physics/depenetration.c physics/islands.c config/mpe_config.c config/mpe_config_schema.c scene/boundary.c ecosystem/mpe_ecosystem.c"
FTC="ecosystem/mfs/modules/ftc/ftc_module.c ecosystem/mfs/modules/ftc/ftc_fleet.c ecosystem/mfs/modules/ftc/submodules/robot.c ecosystem/mfs/modules/ftc/submodules/drivetrain.c ecosystem/mfs/modules/ftc/submodules/motor.c ecosystem/mfs/modules/ftc/submodules/motor_presets.c ecosystem/mfs/modules/ftc/submodules/battery.c"
MOD1="ecosystem/mfs/modules/module_1/mfs_module_1.c ecosystem/mfs/modules/module_1/submodules/gamepad/gamepad.c"

cd "$SRC"
pass=0; fail=0; info_pass=0

run_binary() {
    local name="$1"
    if [ "$name" = "ftc_hotload" ] && [ "${MFS_ASAN_HOTLOAD_ODR_SUPPRESS:-0}" = "1" ]; then
        ASAN_OPTIONS="${ASAN_OPTIONS:-detect_leaks=1:halt_on_error=1}:detect_odr_violation=0" "$OUT/$name"
    else
        "$OUT/$name"
    fi
}

if [ "${1:-}" = "--build-only" ]; then BUILD_ONLY=1; fi

echo "--- FTC module plugin (hot-plug .so; must precede ftc_hotload) ---"
mkdir -p "$MFS/plugins" "$SRC/plugins"
FTC_MOD="$FTC"
if "$TEST_CC" $CFLAGS -fPIC -shared $FTC_MOD -lm -o "$MFS/plugins/mpe_ftc.so" 2>"$OUT/mpe_ftc.build.log"; then
    echo "[BUILD-OK] mfs/plugins/mpe_ftc.so"; pass=$((pass+1));
else
    echo "[BUILD-FAIL] mfs/plugins/mpe_ftc.so"; fail=$((fail+1)); head -n 10 "$OUT/mpe_ftc.build.log";
fi
cp "$MFS/plugins/mpe_ftc.so" "$SRC/plugins/mpe_ftc.so"

# Build unified suite binary (include module_1 and gamepad objects for MPI functions)
RB_T="$MFS/tests"
MOD1_OBJ="$OUT/mfs_module_1.o"
GAMEPAD_OBJ="$OUT/gamepad.o"
"$TEST_CC" $CFLAGS -c "$MFS/modules/module_1/mfs_module_1.c" -o "$MOD1_OBJ" 2>"$OUT/mfs_module_1.build.log" || { echo "[BUILD-FAIL] mfs_module_1.o"; fail=$((fail+1)); head -n 10 "$OUT/mfs_module_1.build.log"; }
"$TEST_CC" $CFLAGS -c "$MFS/modules/module_1/submodules/gamepad/gamepad.c" -o "$GAMEPAD_OBJ" 2>"$OUT/gamepad.build.log" || { echo "[BUILD-FAIL] gamepad.o"; fail=$((fail+1)); head -n 10 "$OUT/gamepad.build.log"; }
"$TEST_CC" $CFLAGS "$RB_T/mfs_suite_main.c" "$RB_T/mfs_suite_a.c" "$RB_T/mfs_suite_b.c" "$RB_T/mfs_suite_c.c" $FTC $CORE "$MOD1_OBJ" "$GAMEPAD_OBJ" -lm -ldl -rdynamic -o "$OUT/mfs_suite" 2>"$OUT/mfs_suite.build.log" && {
    echo "[BUILD-OK] mfs_suite (unified)"; pass=$((pass+1));
} || { echo "[BUILD-FAIL] mfs_suite"; fail=$((fail+1)); head -n 20 "$OUT/mfs_suite.build.log"; }

if [ "${BUILD_ONLY:-0}" != "1" ] && [ "${1:-}" != "--build-only" ]; then
    # Run unified suite with --all
    echo "--- Running unified MFS suite ---"
    "$OUT/mfs_suite" --all
    suite_rc=$?
    if [ $suite_rc -eq 0 ]; then
        echo "[PASS] mfs_suite --all"
        pass=$((pass+1));
    else
        echo "[FAIL] mfs_suite --all (exit $suite_rc)"; fail=$((fail+1));
    fi
else
    # Build-only mode: just verify binary runs --list
    "$OUT/mfs_suite" --list >"$OUT/mfs_suite_list.log" 2>&1
    echo "[BUILD-OK] mfs_suite (unified)"; pass=$((pass+1));
fi

# Diagnostic (ungated) - compile only
for u in "$MFS/modules/ftc/gui_robot_registry.c" "$MFS/modules/module_1/submodules/gamepad/gamepad.c"; do
    n=$(basename $u .c)
    if [ "$n" = "gui_robot_registry" ] && ! pkg-config --exists gtk4 epoxy 2>/dev/null; then
        echo "[SKIP] $n (gtk4 dev headers absent; not part of any loadable module)";
        continue;
    fi
    GTK_CFLAGS="$(pkg-config --cflags gtk4 epoxy 2>"$OUT/pkg-config.log")"
    if "$TEST_CC" $CFLAGS -DMPE_GTK4=1 $GTK_CFLAGS -c "$u" -o "$OUT/$n.o" 2>"$OUT/$n.build.log"; then
        echo "[BUILD-OK] $n"; pass=$((pass+1));
    else
        echo "[BUILD-FAIL] $n"; fail=$((fail+1)); head -n 10 "$OUT/$n.build.log";
    fi
done

echo "FTC RESULT: gated pass=$pass fail=$fail info-diags=$info_pass (ungated)"
[ "$fail" -eq 0 ]
