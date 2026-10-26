# MFS Testing

What the suite contains, what each gate proves, and how to run it.

## Suite layout

- **Unified suite** (`tests/mfs_suite_main.c` + `mfs_suite_a/b/c.c`, 25 gated
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

## Current status (2026-10-14): 29/30

One gate is **red**: `tank` (pivot heading/translation baselines — still
red after the observer Q-filter moved it 2.37→2.78 / 0.043→0.118 with
cause chain in [OBSERVER-Q]; the yaw governor/damper left its numbers
bit-identical — peaks never reach the 3.0 band; no baseline touched).
`odometry_yaw` went red→GREEN on the Q-filter. `partial_throttle` and
`bucket_game` are green after re-derivation to saturation-tolerant laws.
The new `yaw_stability` gate is green (proven red pre-fix). The earlier
`drive_directions` red resolved; the 2026-10-09 "14/17" line and the
CCD-margin narrative below it are historical. Full measurements and the
root-cause write-ups are in
[`KNOWN_FAILURES.md`](KNOWN_FAILURES.md) → `[TANK-PIVOT-*]` /
`[ODOMETRY-YAW]` (search the archives for `[CCD-OBSTACLE-MARGIN]` for the older
set).

## Gate table (unified suite, `--all`)

| Test | Drives | Gates |
|---|---|---|
| `teleop` | tank full-forward, 180 ticks | disp ≥ 0.5 m, \|dy\| ≤ 1.0, yaw ≤ 0.3 rad |
| `mecanum` | strafe 180 ticks | strafe ≥ 0.30 m (MFS-STRAFE-F1 FIXED; was XFAIL) |
| `release_settle` | fwd 180 + release 240 ticks | drive ≥ 0.5 m, then chassis < 0.1 m/s, wheels < 2 rad/s, axles < 3° (DESPOT-2026-10-04: locks the idle-observer storm + glide-equilibrium fixes) |
| `tank` | differential turn 120 ticks | disp ≤ 0.3 m, yaw ≥ 0.1 rad |
| `odometry` | fwd 180 + strafe 60 ticks | move > 0.2 m, odom error < 30% (fwd); strafe transmit ≥ 0.10 m + tracking ≤ 30% (MFS-STRAFE-F2 FIXED; was XFAIL, tripwire kept) |
| `ftc_integration` | fwd + turn + strafe smoke | disp > 0.5 m, dy < 0.5 m, finite |
| `ftc_hotload` | static vs `dlopen` fleet | spawn-refused-unattached, OOB NULL, **bitwise pose+odometry**, detach coast < 2.0 m, re-attach |
| `module_1` | drive@30, shooter@50, stage+fire@80 | drive ≥ 0.5 m, shooter ≥ 3000 rpm, fired ≥ 1 |
| `external_truth` | free fall vs `g_n`=9.80665, viscous-drag ODE, `α=τ/I`, cylinder/sphere inertia, restitution `v_out=e·v_in`, rolling no-slip, energy, Coulomb `d=v₀²/2μg`, DC motor `V=IR+K_eω` + stall/free endpoints, battery OCV/sag/PTC `I²t`/capacity (DESPOT-2026-10-02) |
| `odometry_yaw` | rotate ± on mecanum, L/R pivot ± on tank | **odom_theta sign must match the chassis** on both drivetrains (hard; convention- and battery-independent — this is the gate whose absence let a sign-inverted heading ship inside 15/15 green), tank yaw magnitude within 60%, mecanum magnitude as `[XFAIL][MECANUM-PIVOT]`, `odom_slip`/`clamp_events` well-formed (DESPOT-2026-10-06) |
| `registry` | internal module registry, two worlds | self-detach from inside own `pre_step` defers and never hangs, module detach callback fires exactly once, state released for the detached world only, per-world alias slots distinct, attach/detach idempotence, unregister (DESPOT-2026-10-06) |
| `drive_directions` | pure fwd / rev / strafe / rotate ±, steady-state window | axis dominance (fwd→+Z, strafe→-X/robot-right, rotate→yaw), planar cross-talk ceilings (fwd 25%, strafe 35%), **rotate vs rotate⁻ must have opposite yaw sign**, yaw/planar dominance ratio, fwd/rev antisymmetry (DESPOT-2026-10-02) |
| `ball_spin` | ball spin/wake corner | finite, ball never goes through the floor |
| `bucket_game` | event-gated drive (to carried ≥ 2 or 400 ticks), strafe clear 80 ticks, brake 210 ticks, shoot a calibration lob, read its rim crossing (bounds: 1–12 m out, in −6..6 × 1..14), **move** the bucket onto it, shoot the second ball | capture ≥ 2 then scored ≥ 1 (DESPOT-2026-10-14: fixed 150 ticks encoded chatter-era drive speed; capability gates must be event-gated, timeout still fails) |
| `brake_mode` | drive@1.0 120 ticks, then `ftc_robot_set_brake(1)` — what the gamepad maps every no-stick-input frame to — and hold 90 ticks, then release + redrive 90 | commands zero on brake, `sum|rpm|` < 5 and chassis `|v|` < 0.2 after brake, `sum|rpm|` > 100 on release (no latching) — pins the "motors keep running when sticks are released" fix (BRAKE-2026-10-12) |
| `partial_throttle` | straight drive at cmd 0.25 / 0.5 / 0.75, 480 ticks each (single-driven via the fleet pre_step) | finite; yaw-rate std < 0.02 rad/s at every command; every command drives > 0.5 m (forward-not-backward); response non-decreasing with ordered endpoints (DESPOT-2026-10-14: strict pairwise monotone asserted a false law for a traction-saturating plant — 0.25/0.50 trajectories are bit-identical cone-limited crawl; old curve inflated by chatter rectification. See gate comment + [OBSERVER-Q]/[CUT-LATCH]) |
| `physics_truth` | 15 subtests (T1–T15) | freefall, inertia, bounce `e²(h-r)+r`, rolling, rolling-resistance decay band, **isolated** motor free-speed/stall/back-EMF, static hold, stopping distance `v²/2μg`, 3000-tick stability, coast-down, energy, cylinder rest, revolute anchor |
| `biobuzz_field` | build regulation field twice (idempotency) + 60 settle ticks | floor half 1.8288/top 0; 4 wall inner faces ±1.8288, height 0.3048; pivot bar 1.1165; 2×11 hive bodies live; beam midpoints mirror-symmetric; 4 flower top rings at 0.5461 near nominal stations; ≥6 tape proxies; finite |
| `biobuzz_stage` | build + stage + preload (no robots) + 120 ticks | 40 staged (16 flower + 8 garden + 6 cell + 10 tray) + 16 preload = 40 pollen / 8+8 nectar at regulation radii/colours; stations split 16/8/6/10/16; finite |
| `biobuzz_tip` | teleport 8 pollen into red up-cell, 240 ticks | exactly 1 red tip (edge-triggered), blue quiet (3 nectar = 5.1 < 8); recorded wall moved > 0.10 (mirrored re-pose); unlock +1; score carries 20 |
| `biobuzz_flower` | constructed stack (bottom red nectar, 2 pollen, top blue nectar) + red-garden pieces, 90 ticks | red: owned 0 + bottom 5 + garden 3 = 8; blue: owned 4×2=8 + bottom 5 = 13 (Table 10-2 ownership/colours) |
| `biobuzz_table` | pure predicates, no world | Table 10-2 values (3/5/20/20/2/5/2/1), RP thresholds (swarm 16, P1 4, P2 7), piece spec (40/16, radii), start poses (walls/yaw/zones) |
| `select` | two worlds, two robots in the first | fresh attach reads −1; default 0 after first spawn; OOB selects fail −1 leaving stored untouched; second world independent; drive routed through the selection moves robot 1 (> 0.30 m) while robot 0 holds (< 0.20 m) — pins the terminal/gamepad controlled-robot routing |
| `yaw_stability` | rotate=1.0 for 600 ticks, peak + end yaw | peak < 6.0 rad/s and end < 5.5 (pre-fix: 11.07/10.81 — unbounded spin with ~zero passive decay + lift feedback; post-fix cap ≈ 4.4 rock-steady via governor + active damper). See [YAW-HANDLING] |
| `fleet_clear` | spawn robot + balls + bucket + BIOBUZZ pieces, clear, respawn; then wipe pool behind the fleet, step, respawn | direct API forgets count/selection/balls/scores/tips and respawn works (the "9 3 then one-robot-already" report); pre_step self-heal fires on a replaced pool and respawn works immediately |
| `shoot_power` | same muzzle, fire at power 1.0 / 0.7 / 0.4, fly 180 ticks each | exit speeds scale exactly (9.0 / 6.3 / 3.6 m/s); ranges order 14.2 > 9.1 > 4.7 m; full-power path bit-identical (bucket calibration depends on it) |
| `hive_shot` | robot at a legal wall-adjacent spot, aimed lob (0.65 power, 70 deg), 300 ticks | ball rests INSIDE the red up-cell (cell census 1/0) + last-shot params recorded (the "only undersides" report: stock 50-degree muzzle provably cannot reach the 1.5 m mouths from legal spots at any power) |

Physics-truth notes: T6/T8 test the **motor model isolated** (no
joints/world) because the jointed air-spin plant is a documented limit
cycle; T10 uses the settled-start stopping-distance method (instant decel
windows on tip/chatter transients are fragile); T3 tracks the post-bounce
apex, not the drop height.

## How to run

From the engine tree (canonical — this is what CI and the release ritual
use), with an MFS-rooted `OUTDIR` if desired:

```
cd <475-MPE>/v16R1/src
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
`../475-MPE/v16R1/src`, else fail-fast with the fix). From a bare 461-MFS
checkout: `./build_tests.sh [--build-only]` and `make && make test` both
work (suite runs 17/17, outputs under `461-MFS/temp/`). The engine-tree
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
