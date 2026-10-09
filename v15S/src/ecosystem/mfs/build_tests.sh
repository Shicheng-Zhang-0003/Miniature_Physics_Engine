#!/usr/bin/env bash
# MFS overarching ecosystem: FTC/robotics test build + run (v2).
# Everything MFS-wise lives under v15S/src/ecosystem/mfs/ (modules,
# submodules, tests, docs). Run from v15S/src:
#   ecosystem/mfs/build_tests.sh [--build-only]
# Windows: runs on MSYS2 bash (native .dll/.exe produced, runnable without
# MSYS2). Linux behaviour unchanged.
#
# DUAL-MODE (DESPOT-2026-09-28): this file is SOURCE-IDENTICAL in both trees
# (standalone 461-MFS root and embedded 475-MPE/v15S/src/ecosystem/mfs/).
# It auto-detects which tree it lives in and where the engine sources are:
#
#   embedded:   $MFS/../../core/physics_world.c exists -> SRC=$MFS/../..
#   standalone: $MFS_ENGINE_SRC env, else sibling ../475-MPE/v15S/src,
#               else fail fast with the fix (not a gcc wall).
#
# Canonical invocations:
#   cd <475-MPE>/v15S/src && ecosystem/mfs/build_tests.sh [--build-only]
#   461-MFS/build_tests.sh [--build-only]   (needs the 475 engine alongside,
#       or MFS_ENGINE_SRC=/path/to/v15S/src)
#
# All MFS file references below are ABSOLUTE ($MFS/...) so the script works
# from any CWD; engine CORE files are absolute ($SRC/...) for the same
# reason. The script still `cd`s to $SRC for legacy tooling that expects it.
set -euo pipefail
MFS="$(cd "$(dirname "$0")" && pwd)"

# ---- engine-tree discovery (dual-mode) ----
if [ -f "$MFS/../../core/physics_world.c" ] && [ -d "$MFS/modules/ftc" ]; then
    MODE="embedded"
    SRC="$(cd "$MFS/../.." && pwd)"
    ROOT="$(cd "$SRC/../.." && pwd)"
    OUT="${OUTDIR:-$ROOT/temp/ftc_tests}"
else
    MODE="standalone"
    CAND="${MFS_ENGINE_SRC:-}"
    if [ -z "$CAND" ]; then
        CAND="$(cd "$MFS/../475-MPE/v15S/src" 2>/dev/null && pwd || true)"
    fi
    if [ -n "$CAND" ] && [ -f "$CAND/core/physics_world.c" ]; then
        SRC="$CAND"
    else
        echo "[BUILD-FAIL] engine tree not found (standalone MFS needs the 475 engine)" >&2
        echo "  looked at: \$MFS_ENGINE_SRC, $MFS/../475-MPE/v15S/src" >&2
        echo "  fix: MFS_ENGINE_SRC=/path/to/475-MPE/v15S/src $0 $*" >&2
        echo "  or run the embedded copy: cd <475-MPE>/v15S/src && ecosystem/mfs/build_tests.sh" >&2
        exit 2
    fi
    OUT="${OUTDIR:-$MFS/temp/ftc_tests}"
fi
TMPDIR="$(dirname "$OUT")"
export TMPDIR
export MPE_GAMEPAD_DEVICE=disabled
mkdir -p "$OUT"
OUT="$(cd "$OUT" && pwd)"

TEST_CC="${MFS_TEST_CC:-gcc}"
# ---- Windows detection (MSYS2/MinGW/native; Linux unchanged) ----
MPE_WINDOWS="${MPE_WINDOWS:-0}"
case "$(uname -s 2>/dev/null || echo unknown)" in
  MINGW*|MSYS*|CYGWIN*|Windows*) MPE_WINDOWS=1 ;;
esac
case "${TEST_CC}" in
  *mingw*) MPE_WINDOWS=1 ;;
esac
if [ "${OS:-}" = "Windows_NT" ]; then MPE_WINDOWS=1; fi
if [ "$MPE_WINDOWS" = "1" ]; then
  PLUGIN_EXT=".dll"
  EXE_EXT=".exe"
  DL_LIBS=""
  RDYNAMIC=""
  FPIC=""
  # Static libgcc/winpthread for self-contained .exe/.dll (runs without
  # MSYS2 DLLs in PATH; system libs winmm/xinput/ws2_32 stay dynamic).
  # Falls back gracefully if the toolchain lacks static winpthread.
  WIN_LIBS="-lwinmm -lxinput -lws2_32 -static-libgcc -Wl,-Bstatic -lwinpthread -Wl,-Bdynamic"
  PTHREAD="-pthread"
  # Cross from Linux: run Windows binaries via wine when available.
  WINE_RUN=""
  if command -v wine >/dev/null 2>&1 && [ "$(uname -s 2>/dev/null)" = "Linux" ]; then
    case "${TEST_CC}" in *mingw*) WINE_RUN="wine" ;; esac
  fi
else
  PLUGIN_EXT=".so"
  EXE_EXT=""
  DL_LIBS="-ldl"
  RDYNAMIC="-rdynamic"
  FPIC="-fPIC"
  WIN_LIBS=""
  PTHREAD="-pthread"
fi
CFLAGS="-I$SRC -I$MFS -O2 -Wall -Wextra -ffp-contract=off ${MFS_TEST_CFLAGS:-}"
# Engine CORE: absolute engine paths (was engine-relative + cd; absolute
# survives any CWD and both modes).
CORE="$SRC/core/physics_world.c $SRC/core/rigidbody.c $SRC/core/mpe_registry.c $SRC/core/mpe_loader.c $SRC/core/mpe_diag.c $SRC/core/event_log.c $SRC/core/det_math.c $SRC/core/mpe_primary.c $SRC/physics/collision_narrowphase.c $SRC/physics/collision_cache.c $SRC/physics/collision_solver.c $SRC/physics/collision_ccd.c $SRC/physics/collision_cylinder.c $SRC/physics/broadphase.c $SRC/physics/constraint.c $SRC/physics/revolute_joint.c $SRC/physics/depenetration.c $SRC/physics/islands.c $SRC/config/mpe_config.c $SRC/config/mpe_config_schema.c $SRC/scene/boundary.c $SRC/ecosystem/mpe_ecosystem.c"
# MFS sources: absolute under $MFS (identical layout in both trees).
# DESPOT-2026-10-06: mfs_internal.c is now linked into the SUITE as well as
# the bundle -- tests/mfs_suite_c.c gained mfs_t_registry, which gates the
# registry's own concurrency contract (a module self-detaching from inside its
# own tick used to hang the dispatcher forever; the suite had no coverage of
# that file at all, so the bug was only findable with a standalone probe).
FTC="$MFS/modules/ftc/ftc_module.c $MFS/modules/ftc/ftc_fleet.c $MFS/modules/ftc/submodules/robot.c $MFS/modules/ftc/submodules/drivetrain.c $MFS/modules/ftc/submodules/motor.c $MFS/modules/ftc/submodules/motor_presets.c $MFS/modules/ftc/submodules/battery.c"
MOD1_SRCS="$MFS/modules/module_1/mfs_module_1.c $MFS/modules/module_1/submodules/gamepad/gamepad.c"

cd "$SRC"
# Engine-tree guard (embedded already proven above; standalone proven by
# discovery): fail fast with the fix, not a gcc wall.
if [ ! -f "$SRC/core/physics_world.c" ] || [ ! -d "$MFS/modules/ftc" ]; then
    echo "[BUILD-FAIL] not an engine tree (expected v15S/src layout at $SRC)" >&2
    echo "  run from v15S/src: ecosystem/mfs/build_tests.sh" >&2
    exit 2
fi
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

if [ "$MPE_WINDOWS" = "1" ]; then
  # ---- Windows: thin DLL must link against host import lib ----
  # Build suite first (exports engine symbols), then plugin against it.
  echo "--- MFS suite (Windows: host exports first) ---"
  mkdir -p "$MFS/plugins"
  RB_T="$MFS/tests"
  MOD1_OBJ="$OUT/mfs_module_1.o"
  GAMEPAD_OBJ="$OUT/gamepad.o"
  "$TEST_CC" $CFLAGS -c "$MFS/modules/module_1/mfs_module_1.c" -o "$MOD1_OBJ" 2>"$OUT/mfs_module_1.build.log" || { echo "[BUILD-FAIL] mfs_module_1.o"; fail=$((fail+1)); head -n 10 "$OUT/mfs_module_1.build.log"; }
  "$TEST_CC" $CFLAGS -c "$MFS/modules/module_1/submodules/gamepad/gamepad.c" -o "$GAMEPAD_OBJ" 2>"$OUT/gamepad.build.log" || { echo "[BUILD-FAIL] gamepad.o"; fail=$((fail+1)); head -n 10 "$OUT/gamepad.build.log"; }
  if "$TEST_CC" $CFLAGS "$RB_T/mfs_suite_main.c" "$RB_T/mfs_suite_a.c" "$RB_T/mfs_suite_b.c" "$RB_T/mfs_suite_c.c" $FTC "$MFS/mfs_internal.c" $CORE "$MOD1_OBJ" "$GAMEPAD_OBJ" -lm $PTHREAD $WIN_LIBS -Wl,--export-all-symbols -Wl,--out-implib,"$OUT/libmfs_suite.a" -o "$OUT/mfs_suite$EXE_EXT" 2>"$OUT/mfs_suite.build.log"; then
      echo "[BUILD-OK] mfs_suite (unified)"; pass=$((pass+1));
  else
      echo "[BUILD-FAIL] mfs_suite"; fail=$((fail+1)); head -n 20 "$OUT/mfs_suite.build.log";
  fi
  if [ -n "$EXE_EXT" ] && [ -f "$OUT/mfs_suite$EXE_EXT" ]; then
    cp "$OUT/mfs_suite$EXE_EXT" "$OUT/mfs_suite" 2>/dev/null || true
  fi
  echo "--- FTC module plugin (hot-plug $PLUGIN_EXT; linked against host) ---"
  FTC_MOD="$FTC"
  if "$TEST_CC" $CFLAGS $FPIC -shared $FTC_MOD "$OUT/libmfs_suite.a" -lm $WIN_LIBS -o "$MFS/plugins/mpe_ftc$PLUGIN_EXT" 2>"$OUT/mpe_ftc.build.log"; then
      echo "[BUILD-OK] mfs/plugins/mpe_ftc$PLUGIN_EXT"; pass=$((pass+1));
  else
      echo "[BUILD-FAIL] mfs/plugins/mpe_ftc$PLUGIN_EXT"; fail=$((fail+1)); head -n 20 "$OUT/mpe_ftc.build.log";
  fi
  # Consolidated 2026-10-09: no $SRC/plugins copy (top-level plugins/ deleted).
  if [ "$PLUGIN_EXT" != ".so" ]; then
    cp "$MFS/plugins/mpe_ftc$PLUGIN_EXT" "$MFS/plugins/mpe_ftc.so" 2>/dev/null || true
  fi
else
  echo "--- FTC module plugin (hot-plug $PLUGIN_EXT; must precede ftc_hotload) [$MODE] ---"
  mkdir -p "$MFS/plugins"
  FTC_MOD="$FTC"
  if "$TEST_CC" $CFLAGS $FPIC -shared $FTC_MOD -lm $WIN_LIBS -o "$MFS/plugins/mpe_ftc$PLUGIN_EXT" 2>"$OUT/mpe_ftc.build.log"; then
      echo "[BUILD-OK] mfs/plugins/mpe_ftc$PLUGIN_EXT"; pass=$((pass+1));
  else
      echo "[BUILD-FAIL] mfs/plugins/mpe_ftc$PLUGIN_EXT"; fail=$((fail+1)); head -n 10 "$OUT/mpe_ftc.build.log";
  fi
  # Keep .so alias on Windows so the loader path resolves (loader accepts both).
  # Consolidated 2026-10-09: no $SRC/plugins copy.
  if [ "$PLUGIN_EXT" != ".so" ]; then
    cp "$MFS/plugins/mpe_ftc$PLUGIN_EXT" "$MFS/plugins/mpe_ftc.so" 2>/dev/null || true
  fi
fi

# Build unified suite binary (include module_1 and gamepad objects for MPI functions)
# (Windows already built above; Linux builds here.)
if [ "$MPE_WINDOWS" != "1" ]; then
RB_T="$MFS/tests"
MOD1_OBJ="$OUT/mfs_module_1.o"
GAMEPAD_OBJ="$OUT/gamepad.o"
"$TEST_CC" $CFLAGS -c "$MFS/modules/module_1/mfs_module_1.c" -o "$MOD1_OBJ" 2>"$OUT/mfs_module_1.build.log" || { echo "[BUILD-FAIL] mfs_module_1.o"; fail=$((fail+1)); head -n 10 "$OUT/mfs_module_1.build.log"; }
"$TEST_CC" $CFLAGS -c "$MFS/modules/module_1/submodules/gamepad/gamepad.c" -o "$GAMEPAD_OBJ" 2>"$OUT/gamepad.build.log" || { echo "[BUILD-FAIL] gamepad.o"; fail=$((fail+1)); head -n 10 "$OUT/gamepad.build.log"; }
"$TEST_CC" $CFLAGS "$RB_T/mfs_suite_main.c" "$RB_T/mfs_suite_a.c" "$RB_T/mfs_suite_b.c" "$RB_T/mfs_suite_c.c" $FTC "$MFS/mfs_internal.c" $CORE "$MOD1_OBJ" "$GAMEPAD_OBJ" -lm $DL_LIBS $RDYNAMIC $PTHREAD $WIN_LIBS -o "$OUT/mfs_suite$EXE_EXT" 2>"$OUT/mfs_suite.build.log" && {
    echo "[BUILD-OK] mfs_suite (unified)"; pass=$((pass+1));
} || { echo "[BUILD-FAIL] mfs_suite"; fail=$((fail+1)); head -n 20 "$OUT/mfs_suite.build.log"; }
# Alias for runner expecting extensionless name on Windows
if [ -n "$EXE_EXT" ] && [ -f "$OUT/mfs_suite$EXE_EXT" ]; then
  cp "$OUT/mfs_suite$EXE_EXT" "$OUT/mfs_suite" 2>/dev/null || true
fi
fi # end Linux-only suite build (Windows built earlier)

if [ "${BUILD_ONLY:-0}" != "1" ] && [ "${1:-}" != "--build-only" ]; then
    # Run unified suite with --all.
    # DESPOT-2026-09-26: under ASan the hotload case dlopens a plugin image
    # containing a second mpe_module_desc (intentional duplicate-global, the
    # very thing hotload proves loadable). Suppress only the ODR heuristic —
    # leaks/UB remain armed — when the runner asks for it (it always does in
    # the sanitizer profile via MFS_ASAN_HOTLOAD_ODR_SUPPRESS=1).
    echo "--- Running unified MFS suite ---"
    # DESPOT-2026-09-26: suite stdout goes to mfs_suite.run.log (kept as an
    # artifact AND scanned by the runner for sanitizer errors via *.run.log).
    # It must not inline here: the runner's mfs-count-contract counts
    # script-level [PASS]/[BUILD-OK] lines against the script summary (5),
    # and 8 inlined suite [PASS] lines break that count (13 != 5).
    SUITE_BIN="$OUT/mfs_suite$EXE_EXT"
    # DESPOT-2026-10-03: hand the suite the plugin by ABSOLUTE path.
    # mfs_t_ftc_hotload resolves the FTC plugin relative to the WORKING
    # DIRECTORY, so the same source, same build and same .so passed or failed
    # purely on where build_tests.sh happened to be invoked from. It passed
    # when run as `cd ecosystem/mfs && ./build_tests.sh` and failed when the
    # runner invoked it from the repository root -- which is why CI never saw
    # it. Do not rely on CWD: name the artifact explicitly.
    if [ -f "$MFS/plugins/mpe_ftc$PLUGIN_EXT" ]; then
        MPE_FTC_PLUGIN="$MFS/plugins/mpe_ftc$PLUGIN_EXT"
        export MPE_FTC_PLUGIN
    fi
    # Cross from Linux: prefix with wine (empty on native Windows/MSYS2).
    # shellcheck disable=SC2086
    # DESPOT-2026-10-09: errexit must be OFF around the suite run. Under
    # `set -e` a single failing test aborted the WHOLE script one line above
    # `suite_rc=$?`, so every line after it was unreachable: the per-run log
    # pointer, the Total/Pass/Fail echo, the sanitizer scan, the
    # "[FAIL] mfs_suite --all (exit N)" branch, and finally the
    # "FTC RESULT: gated pass=.. fail=.." summary itself. A red suite
    # therefore printed NO counts and exited silently -- the harness
    # destroyed the exact evidence it exists to produce, and the runner
    # surfaced the missing summary as a confusing contract violation instead
    # of the real test failures. Capture the code, then restore strict mode.
    set +e
    if [ "${MFS_ASAN_HOTLOAD_ODR_SUPPRESS:-0}" = "1" ]; then
        ASAN_OPTIONS="${ASAN_OPTIONS:-detect_leaks=1:halt_on_error=1}:detect_odr_violation=0" ${WINE_RUN:-} "$SUITE_BIN" --all >"$OUT/mfs_suite.run.log" 2>&1
    else
        ${WINE_RUN:-} "$SUITE_BIN" --all >"$OUT/mfs_suite.run.log" 2>&1
    fi
    suite_rc=$?
    set -e
    # Pointer only (never inline suite [PASS] lines: the runner counts
    # script-level bracket lines against the script summary).
    echo "(full suite output: $OUT/mfs_suite.run.log)"
    grep -E "^(=== SUMMARY ===|Total:|Pass:|Fail:)" "$OUT/mfs_suite.run.log" || true
    if grep -q "ERROR: AddressSanitizer\|runtime error:" "$OUT/mfs_suite.run.log"; then
        echo "[FAIL] mfs_suite sanitizer error (see mfs_suite.run.log)"; fail=$((fail+1)); suite_rc=1
    fi
    if [ $suite_rc -eq 0 ]; then
        echo "[PASS] mfs_suite --all"
        pass=$((pass+1));
    else
        echo "[FAIL] mfs_suite --all (exit $suite_rc)"; fail=$((fail+1));
    fi
else
    # Build-only mode: just verify binary runs --list
    ${WINE_RUN:-} "$OUT/mfs_suite$EXE_EXT" --list >"$OUT/mfs_suite_list.log" 2>&1
    echo "[BUILD-OK] mfs_suite (unified)"; pass=$((pass+1));
fi

# Diagnostic (ungated) - compile only
for u in "$MFS/modules/ftc/gui_robot_registry.c" "$MFS/modules/module_1/submodules/gamepad/gamepad.c"; do
    n=$(basename $u .c)
    if [ "$n" = "gui_robot_registry" ] && ! pkg-config --exists gtk4 epoxy 2>/dev/null; then
        echo "[SKIP] $n (gtk4 dev headers absent; not part of any loadable module)";
        continue;
    fi
    if [ "$n" = "gamepad" ]; then
        # gamepad is portable C (Linux js + Windows XInput); never needs GTK.
        if "$TEST_CC" $CFLAGS -c "$u" -o "$OUT/$n.o" 2>"$OUT/$n.build.log"; then
            echo "[BUILD-OK] $n"; pass=$((pass+1));
        else
            echo "[BUILD-FAIL] $n"; fail=$((fail+1)); head -n 10 "$OUT/$n.build.log";
        fi
        continue;
    fi
    GTK_CFLAGS="$(pkg-config --cflags gtk4 epoxy 2>"$OUT/pkg-config.log" || true)"
    if "$TEST_CC" $CFLAGS -DMPE_GTK4=1 $GTK_CFLAGS -c "$u" -o "$OUT/$n.o" 2>"$OUT/$n.build.log"; then
        echo "[BUILD-OK] $n"; pass=$((pass+1));
    else
        echo "[BUILD-FAIL] $n"; fail=$((fail+1)); head -n 10 "$OUT/$n.build.log";
    fi
done

# ---------------------------------------------------------------------
# Legacy per-test mains + ungated diags (DESPOT-2026-10-06).
#
# These files were compiled by NOTHING: not build_tests.sh, not the makefile,
# not mfs_sources.mk. They were also broken in a way nothing would have
# reported -- each includes "ecosystem/mfs/tests/mfs_test_common.h", an
# ENGINE-RELATIVE path. Built from the standalone tree that resolves through
# -I$SRC against the 475 TWIN's copy of the header, not this tree's (confirmed
# with gcc -H: it picked
# ../475-MPE/v15S/src/ecosystem/mfs/tests/mfs_test_common.h). Benign only
# because the two files are byte-identical today -- exactly the cross-tree
# coupling SYNC_CONTRACT.md exists to prevent, waiting for the trees to
# diverge. Includes are now same-directory.
#
# They are compiled (not deleted) because TESTING.md calls them the reference
# implementations the unified ports were checked against, and that claim is
# only meaningful while they still build. Each is behind its own -D guard.
# mfs_suite_*.c are excluded (they are the unified suite itself).
# ---------------------------------------------------------------------
if [ "${1:-}" != "--build-only" ]; then
  LEGACY_PAIRS="teleop_drive_test:MPE_TELEOP_DRIVE_TEST
mecanum_drive_test:MPE_MECANUM_DRIVE_TEST
tank_turn_test:MFS_TANK_TURN_TEST
odometry_accuracy_test:MFS_ODOMETRY_ACCURACY_TEST
ftc_integration_test:MPE_FTC_INTEGRATION_TEST
ftc_hotload_test:MPE_FTC_HOTLOAD_TEST
physics_truth_test:MPE_PHYSICS_TRUTH_TEST
ftc_debug_test:MPE_FTC_DEBUG_TEST
idle_spin_diag:MFS_IDLE_DIAG
idle_rootcause_diag:MFS_IDLE_ROOTCAUSE_DIAG
idle_spin_deep_diag:MFS_IDLE_DEEP_DIAG
odometry_diag:MFS_ODOM_DIAG"
  OLDIFS="$IFS"
  IFS='
'
  for pair in $LEGACY_PAIRS; do
    IFS="$OLDIFS"
    base="${pair%%:*}"; def="${pair##*:}"
    src="$MFS/tests/$base.c"
    if [ ! -f "$src" ]; then echo "[SKIP] $base (missing)"; IFS='
'; continue; fi
    if "$TEST_CC" $CFLAGS -I"$MFS/tests" -D"$def" "$src" $FTC "$MFS/mfs_internal.c" $CORE "$MOD1_OBJ" "$GAMEPAD_OBJ" \
         -lm $DL_LIBS $RDYNAMIC $PTHREAD $WIN_LIBS -o "$OUT/$base$EXE_EXT" 2>"$OUT/$base.build.log"; then
      echo "[BUILD-OK] $base"; pass=$((pass+1))
    else
      echo "[BUILD-FAIL] $base"; fail=$((fail+1)); head -n 10 "$OUT/$base.build.log"
    fi
    IFS='
'
  done
  IFS="$OLDIFS"
  info_pass=$((info_pass + 12))
fi

echo "FTC RESULT: gated pass=$pass fail=$fail info-diags=$info_pass (ungated)"
[ "$fail" -eq 0 ]
