# MFS Testing

What the suite contains, what each gate proves, and how to run it.

## Suite layout

- **Unified suite** (`tests/mfs_suite_main.c` + `mfs_suite_a/b/c.c`, 12 gated
  tests, registry with exact-name dispatch, config save/restore, NaN
  watchdog). This is what `build_tests.sh` runs and what the release gates
  count.
- **Legacy per-test mains** (`teleop_drive_test.c`, `mecanum_drive_test.c`,
  `tank_turn_test.c`, `odometry_accuracy_test.c`, `ftc_integration_test.c`,
  `ftc_hotload_test.c`, `physics_truth_test.c`, …): kept for archaeology and
  as the reference implementations the unified ports were checked against
  (notably the hotload attach/spawn/bitwise/detach flow and the module_1
  drive/intake/shooter/fire schedule — both were broken in early unified
  ports and re-ported faithfully).
- **Ungated diags** (`idle_*_diag.c`, `odometry_diag.c`,
  `test_bounce_debug.c`): compile/run, informational only.
- **Shared rigs:** `tests/mfs_test.h` (framework: registry asserts with
  file:line, config isolation, Coulomb floor helpers, finite watchdog) and
  `tests/mfs_test_common.h` (tile slab top y=0, μ 1.0/0.8, e=0). Robot tests
  pin **128 solver iterations** (40:1 chassis/wheel mass ratio).

## Gate table (unified suite, `--all`)

| Test | Drives | Gates |
|---|---|---|
| `teleop` | tank full-forward, 180 ticks | disp ≥ 0.5 m, \|dy\| ≤ 1.0, yaw ≤ 0.3 rad |
| `mecanum` | strafe 180 ticks | strafe ≥ 0.30 m (MFS-STRAFE-F1 FIXED; was XFAIL) |
| `tank` | differential turn 120 ticks | disp ≤ 0.3 m, yaw ≥ 0.1 rad |
| `odometry` | fwd 180 + strafe 60 ticks | move > 0.2 m, odom error < 30% (fwd); strafe transmit ≥ 0.10 m + tracking ≤ 30% (MFS-STRAFE-F2 FIXED; was XFAIL, tripwire kept) |
| `ftc_integration` | fwd + turn + strafe smoke | disp > 0.5 m, dy < 0.5 m, finite |
| `ftc_hotload` | static vs `dlopen` fleet | spawn-refused-unattached, OOB NULL, **bitwise pose+odometry**, detach coast < 2.0 m, re-attach |
| `module_1` | drive@30, shooter@50, stage+fire@80 | drive ≥ 0.5 m, shooter ≥ 3000 rpm, fired ≥ 1 |
| `physics_truth` | 15 subtests (T1–T15) | freefall, inertia, bounce `e²(h-r)+r`, rolling, rolling-resistance decay band, **isolated** motor free-speed/stall/back-EMF, static hold, stopping distance `v²/2μg`, 3000-tick stability, coast-down, energy, cylinder rest, revolute anchor |

Physics-truth notes: T6/T8 test the **motor model isolated** (no
joints/world) because the jointed air-spin plant is a documented limit
cycle; T10 uses the settled-start stopping-distance method (instant decel
windows on tip/chatter transients are fragile); T3 tracks the post-bounce
apex, not the drop height.

## How to run

From the engine tree (canonical — this is what CI and the release ritual
use), with an MFS-rooted `OUTDIR` if desired:

```
cd <475-MPE>/v15S/src
ecosystem/mfs/build_tests.sh            # full suite
ecosystem/mfs/build_tests.sh --build-only
OUTDIR=/tmp/mfs_out ecosystem/mfs/build_tests.sh
```

Script contract (do not break it): stdout carries exactly the 5
script-level `[BUILD-OK]`/`[PASS]` lines matching the `FTC RESULT: gated
pass=5 fail=0` summary; full suite output goes to `$OUT/mfs_suite.run.log`
(kept as artifact and scanned for sanitizer errors). `MPE_GAMEPAD_DEVICE`
is forced to `disabled` (no `/dev/input/js0` probing on headless boxes).

**Standalone (FIXED 2026-09-28, was "does not run from a bare checkout"):** 
`build_tests.sh` and the `Makefile` are dual-mode — source-identical in both
trees, auto-detecting the engine (`$MFS_ENGINE_SRC`, else sibling
`../475-MPE/v15S/src`, else fail-fast with the fix). From a bare 461-MFS
checkout: `./build_tests.sh [--build-only]` and `make && make test` both
work (suite runs 8/8, outputs under `461-MFS/temp/`). The engine-tree
invocation above remains canonical for CI/release.

## Sanitizers

```
MFS_TEST_CFLAGS="-O1 -fno-omit-frame-pointer -fsanitize=address,undefined" \
MFS_ASAN_HOTLOAD_ODR_SUPPRESS=1 build_tests.sh
```

- Hotload intentionally loads a plugin image containing a second
  `mpe_module_desc`, so only the ODR heuristic is suppressed
  (`detect_odr_violation=0`); leak detection, address checks, and UBSan
  stay armed.
- Every `mfs_test_world` world is released on **every** exit path (missing
  cleanups once polluted the live-world registry and crashed re-init under
  ASan via stack-slot reuse — fixed class, keep it fixed).
- `-O3` (.so/Makefile) vs `-O2` (tests) is a disclosed FP divergence;
  bit-identity holds only within one build config + libm.
