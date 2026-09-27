#!/usr/bin/env bash
# MFS overarching ecosystem: FTC/robotics test build + run (v2).
# Everything MFS-wise lives under v15S/src/ecosystem/mfs/ (modules,
# submodules, tests, docs). Run from v15S/src:
#   ecosystem/mfs/build_tests.sh [--build-only]
# Windows: runs on MSYS2 bash (native .dll/.exe produced, runnable without
# MSYS2). Linux behaviour unchanged.
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

if [ "$MPE_WINDOWS" = "1" ]; then
  # ---- Windows: thin DLL must link against host import lib ----
  # Build suite first (exports engine symbols), then plugin against it.
  echo "--- MFS suite (Windows: host exports first) ---"
  mkdir -p "$MFS/plugins" "$SRC/plugins"
  RB_T="$MFS/tests"
  MOD1_OBJ="$OUT/mfs_module_1.o"
  GAMEPAD_OBJ="$OUT/gamepad.o"
  "$TEST_CC" $CFLAGS -c "$MFS/modules/module_1/mfs_module_1.c" -o "$MOD1_OBJ" 2>"$OUT/mfs_module_1.build.log" || { echo "[BUILD-FAIL] mfs_module_1.o"; fail=$((fail+1)); head -n 10 "$OUT/mfs_module_1.build.log"; }
  "$TEST_CC" $CFLAGS -c "$MFS/modules/module_1/submodules/gamepad/gamepad.c" -o "$GAMEPAD_OBJ" 2>"$OUT/gamepad.build.log" || { echo "[BUILD-FAIL] gamepad.o"; fail=$((fail+1)); head -n 10 "$OUT/gamepad.build.log"; }
  if "$TEST_CC" $CFLAGS "$RB_T/mfs_suite_main.c" "$RB_T/mfs_suite_a.c" "$RB_T/mfs_suite_b.c" "$RB_T/mfs_suite_c.c" $FTC $CORE "$MOD1_OBJ" "$GAMEPAD_OBJ" -lm $PTHREAD $WIN_LIBS -Wl,--export-all-symbols -Wl,--out-implib,"$OUT/libmfs_suite.a" -o "$OUT/mfs_suite$EXE_EXT" 2>"$OUT/mfs_suite.build.log"; then
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
  cp "$MFS/plugins/mpe_ftc$PLUGIN_EXT" "$SRC/plugins/mpe_ftc$PLUGIN_EXT" 2>/dev/null || true
  if [ "$PLUGIN_EXT" != ".so" ]; then
    cp "$MFS/plugins/mpe_ftc$PLUGIN_EXT" "$MFS/plugins/mpe_ftc.so" 2>/dev/null || true
    cp "$SRC/plugins/mpe_ftc$PLUGIN_EXT" "$SRC/plugins/mpe_ftc.so" 2>/dev/null || true
  fi
else
  echo "--- FTC module plugin (hot-plug $PLUGIN_EXT; must precede ftc_hotload) ---"
  mkdir -p "$MFS/plugins" "$SRC/plugins"
  FTC_MOD="$FTC"
  if "$TEST_CC" $CFLAGS $FPIC -shared $FTC_MOD -lm $WIN_LIBS -o "$MFS/plugins/mpe_ftc$PLUGIN_EXT" 2>"$OUT/mpe_ftc.build.log"; then
      echo "[BUILD-OK] mfs/plugins/mpe_ftc$PLUGIN_EXT"; pass=$((pass+1));
  else
      echo "[BUILD-FAIL] mfs/plugins/mpe_ftc$PLUGIN_EXT"; fail=$((fail+1)); head -n 10 "$OUT/mpe_ftc.build.log";
  fi
  cp "$MFS/plugins/mpe_ftc$PLUGIN_EXT" "$SRC/plugins/mpe_ftc$PLUGIN_EXT"
  # Keep .so alias on Windows so both loader paths resolve (loader accepts both).
  if [ "$PLUGIN_EXT" != ".so" ]; then
    cp "$MFS/plugins/mpe_ftc$PLUGIN_EXT" "$MFS/plugins/mpe_ftc.so" 2>/dev/null || true
    cp "$SRC/plugins/mpe_ftc$PLUGIN_EXT" "$SRC/plugins/mpe_ftc.so" 2>/dev/null || true
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
"$TEST_CC" $CFLAGS "$RB_T/mfs_suite_main.c" "$RB_T/mfs_suite_a.c" "$RB_T/mfs_suite_b.c" "$RB_T/mfs_suite_c.c" $FTC $CORE "$MOD1_OBJ" "$GAMEPAD_OBJ" -lm $DL_LIBS $RDYNAMIC $PTHREAD $WIN_LIBS -o "$OUT/mfs_suite$EXE_EXT" 2>"$OUT/mfs_suite.build.log" && {
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
    # Cross from Linux: prefix with wine (empty on native Windows/MSYS2).
    # shellcheck disable=SC2086
    if [ "${MFS_ASAN_HOTLOAD_ODR_SUPPRESS:-0}" = "1" ]; then
        ASAN_OPTIONS="${ASAN_OPTIONS:-detect_leaks=1:halt_on_error=1}:detect_odr_violation=0" ${WINE_RUN:-} "$SUITE_BIN" --all >"$OUT/mfs_suite.run.log" 2>&1
    else
        ${WINE_RUN:-} "$SUITE_BIN" --all >"$OUT/mfs_suite.run.log" 2>&1
    fi
    suite_rc=$?
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

echo "FTC RESULT: gated pass=$pass fail=$fail info-diags=$info_pass (ungated)"
[ "$fail" -eq 0 ]
