# MFS — overarching module ecosystem ("mfs-simulator")

> **Status 2026-10-09: the suite is 14/17, not green.** `tank`,
> `drive_directions` and `odometry_yaw` are red from one root cause — the
> global CCD obstacle margin added in commit `41bcc0b`, which perturbs
> motion on a pivoting chassis. Zeroing it restores 17/17. Measurements and
> the harness fix that made the failure visible at all are in
> `docs/KNOWN_FAILURES.md` → `[CCD-OBSTACLE-MARGIN]`.
> Any "15/15 green" wording below describes an earlier tree.

> **Audit state (2026-10-06).** The yaw axis was audited by writing probes
> that test the claims rather than reading them. The suite was 15/15 green
> with ASan+UBSan clean while **the heading odometry had the opposite sign to
> the chassis on both drivetrains**, and **no test in the tree read
> `odom_theta`, `odom_slip` or `clamp_events` at all** — that coverage hole
> is the real defect. Both bugs are fixed and gated; the coverage hole is
> closed with a new `odometry_yaw` case that produces 4 failures when the
> original defects are reintroduced and 0 with the fix. The only yaw
> measurement in the suite was also **aliasing**: it differenced two
> `atan2(quaternion)` values and wrapped into `[-pi,pi]`, which folds any
> real rate above `pi` rad/s to a small plausible number *and inverts its
> sign* — so the gate written to catch rotate-directionality was passing on an
> artefact. Now integrates `angular_velocity.y*dt`, which cannot alias.
> A registry self-detach deadlock was also fixed and gated (`registry`);
> `mfs_internal.c` had never been linked into the suite at all.
> One frontier is **open, not fixed**: a full-power mecanum pivot runs ~3.5x
> past the kinematic ceiling. Root cause is structural (analytic-mode hubs
> ship zero friction, so there is no longitudinal traction path) and every
> cheap bound tested either failed or destroyed the F1 strafe. It is ticketed
> as `[MECANUM-PIVOT]` with its elimination table. Suite **17 gated, 17/17**.

> **Audit state (2026-10-05).** The 2026-10-02 drift recurred with the
> guard working correctly and nobody reading it. This tree had again fallen
> behind its twin at `475-MPE/v15S/src/ecosystem/mfs`, now by 19 files of
> real content: the idle-tire hold and hysteretic wheel brake, the motor
> explicit-path load fix, the 128-iteration spawn floor and 30 m field, and
> the fifteenth gate. Byte drift was reported on 47 files, but 28 of those
> were the engine tree's tree-wide clang-format pass and nothing else — a
> reformat larger than the change. Normalising comments and whitespace and
> comparing token streams showed every one of the 19 was an insertion on the
> twin's side and none ran the other way, so the twin was a strict superset
> and the direction was unambiguous rather than guessed. Mirrored all 54
> shared files twin → this tree; no file added, none removed. Suite **15
> gated, 15/15**, and 15/15 again under ASan+UBSan with leak detection on.
> `docs/SYNC_CONTRACT.md` records the method and adds a clause for it.

> **Audit state (2026-10-02).** This tree had silently drifted 11 files behind
> its twin at `475-MPE/v15S/src/ecosystem/mfs`, including four gated tests it
> did not contain, because `sync_mfs_check.sh` — advertised in
> `docs/SYNC_CONTRACT.md` as usable from this root — could not locate the twin
> from here and exited with the same status as real drift. The drift guard is
> fixed and now reports three distinct outcomes (in sync / drift / **did not
> run**); the drift was mirrored (the twin was verified to be a strict
> superset, so nothing was lost); and `.gitignore` was added after finding 13
> build artifacts tracked since the initial import. Suite is now **15 gated
> tests, 15/15**, clean under ASan+UBSan. See `docs/KNOWN_FAILURES.md` for the
> full ledger.

Everything MFS-wise lives under this folder: modules and submodules
contained within the overarching MFS module ecosystem. (Previously
`v15S/robotics_backup/` + scattered `mfs_*` dirs; consolidated here with
history preserved via `git mv`.)

```
v15S/src/ecosystem/mfs/                  # MFS root ("mfs-simulator")
  readme.md                              # this file
  Makefile                               # unified standalone build (thin .so, build/ objs)
  mfs_sources.mk                         # canonical engine+FTC file lists (mirrored in build_tests.sh)
  build_tests.sh                         # FTC/robotics test build + run (17 gated (unified) + build checks + ungated diags)
  mfs_ecosystem.c                        # overarching descriptor: registers
                                         #   modules/module_1 + modules/ftc
  mfs_internal.c/.h                      # internal static module registry
  modules/
    module_1/                            # MPI module "mfs-simulator" (BioBuzz
      mfs_module_1.c/.h                  #   field + mecanum robot + intake +
      mfs_module_1_test.c                #   shooter + balls; attach→tick→detach test)
      submodules/
        gamepad/gamepad.c/.h             # F310 joystick input (module_1 only)
    ftc/                                 # MPI module "ftc-fleet" (hot-pluggable
      ftc_module.c                       #   robot fleet; exports mpe_module_desc
      ftc_fleet.c/.h                     #   per-world fleet (spawn/step/get)
      gui_robot_registry.c/.h            #   GUI-side robot + proxy registry
      submodules/                        # ftc support libs (no descriptors)
        robot.c/.h                       # chassis + wheels + joints assembly
        drivetrain.c/.h                  # tank/mecanum mixers, traction, odometry
        motor.c/.h                       # DC electrical model
        motor_presets.c/.h               # 57-preset FTC catalog (see docs/)
        battery.c/.h                     # sag + drain model
  tests/                                 # teleop, mecanum, tank, odometry,
                                         # external-truth, ftc integration,
                                         # physics truth, odometry_yaw,
                                         # hotload, module_1 test, registry,
                                         # mfs_test.h (framework) +
                                         #   mfs_test_common.h (shared setup:
                                         #   128 iters + tile floor),
                                         # (+12 legacy per-test mains and
                                         #  ungated diags, wired by
                                         #  build_tests.sh --all-targets)
  .gitignore                            # build/, temp/, plugins/, *.o/.so
  docs/
    FTC_SPECS.md                         # motor spec-sheet sources + URLs
    ARCHITECTURE.md                      # layout, lifecycle, build invariants
    MODELS.md                            # motor/battery/drivetrain/odometry math
    TESTING.md                           # suite gates, how to run, sanitizers
    KNOWN_FAILURES.md                    # ticketed frontiers (strafe, air-spin)
    SYNC_CONTRACT.md                     # twin-tree sync with the 475 copy
    VALIDATION.md                        # external-truth reference, 76 checks, findings
  plugins/                               # build output only (gitignored):
                                         # mpe_ftc.so lands here
```

Kernel interface (`v15S/src/ecosystem/`, one level up, NOT MFS) holds the
MEI registry (`mpe_ecosystem.h/.c`) every ecosystem plugs into.

## Include convention

Location-independent; never use `../` crosses (they break on every move):

- engine headers → `core/...`, `physics/...`, `config/...` (`-I` engine `src/`)
- MFS-internal  → `modules/...` from the MFS root (`-I` this folder), e.g.
  `modules/ftc/submodules/robot.h`, `modules/ftc/ftc_fleet.h`
- siblings within one dir stay bare (`"robot.h"`, `"gamepad.h"`)

## Modules vs submodules

- A **module** exports an MPI descriptor (`mpe_module_desc_t`) and has a
  tick lifecycle (attach/pre_step/post_step/detach): `module_1`
  (`mfs_module_1_desc`) and `ftc` (`mpe_module_desc`, the symbol the kernel
  `dlopen` loader `dlsym`s). The overarching `mfs_ecosystem.c` descriptor
  registers and attaches **both** through `mfs_internal.*`.
- A **submodule** is a support library with no descriptor, owned by exactly
  one module: the robot stack under `modules/ftc/submodules/`, gamepad
  under `modules/module_1/submodules/`.

Design rules modules follow (and future modules should too):

- **One descriptor per `.so`**; the loader `dlsym`s exactly that symbol.
  Ecosystem bundles are the exception: `mfs_ecosystem.so` links its inner
  modules' objects, so it exports both symbols and the loader takes
  `mpe_ecosystem_desc` first by documented precedence.
- **Per-world state only**: fleets/states allocate in `attach`, free in
  `detach`. Detach never deletes world bodies.
- **Tick position**: `pre_step` lands motor torques/traction in the
  accumulators right before velocity integration — identical to the manual
  drive + `physics_world_step()` flow.
- **Determinism flags are honest**: `false` on both modules (odometry
  integrates the heading frame with libm `cosf/sinf`; module_1 also polls
  a live gamepad). Force/torque paths are IEEE-exact; same machine plus
  same libm is bit-stable (proven by `ftc_hotload_test`).
- **Thin `.so` pattern**: module shared objects carry only MFS objects;
  engine symbols resolve against the `-rdynamic` host (no version skew).

## Build & run

From `v15S/src` (canonical — what CI and the release ritual use):

```
ecosystem/mfs/build_tests.sh            # full FTC suite (17 inner via unified mfs_suite --all, + build checks + ungated diags)
ecosystem/mfs/build_tests.sh --build-only
```

Binaries and logs go to `../../temp/ftc_tests` by default (override with `OUTDIR=...`).
The script also stages `mpe_ftc.so` at `ecosystem/mfs/plugins/`, which is
what `mod load ecosystem/mfs/plugins/mpe_ftc.so` and the hotload test use
(gitignored build output). Test mains select via `-D`.

> **Corrected 2026-10-09.** This text used to say the loader "only accepts
> `plugins/<name>.so`". That is no longer true, and `src/plugins/` no longer
> exists: the loader now jails to **`ecosystem/`** at any nesting depth, and
> the former top-level `plugins/` directory was removed as part of the
> ecosystem consolidation. Load the plugin from its `ecosystem/mfs/plugins/`
> path.

Standalone, from a bare 461-MFS checkout (dual-mode since 2026-09-28 —
same file, auto-discovers the engine via `$MFS_ENGINE_SRC` or sibling
`../475-MPE/v15S/src`):

```
./build_tests.sh [--build-only]   # full suite, outputs under ./temp/ftc_tests
make && make test                 # thin .so files + module_1 test
```

Thin-`.so` builds (either tree): `make` produces `mfs_module_1.so`,
`mfs_ecosystem.so`, `plugins/mpe_ftc.so`; `make test` runs the module_1
test. Out-of-tree engine work needs nothing else; or from `v15S/src`:
`make mfs_ecosystem.so`.

Inside the engine terminal (no rebuild needed — full drive session).
One field, one robot, one controller (streamlined 2026-10-04; the
controller is the physical Logitech F310, mode switch X):

```
mod load ecosystem/mfs/mfs_ecosystem.so   # load the bundle
eco attach mfs-simulator                  # attach ftc-fleet to the primary world
ftc spawn                                 # THE mecanum robot, tile field auto-added
ftc telemetry                             # pose, odometry, battery, wheels
```

Drive with the pad: left stick = forward/strafe, right stick X = rotate,
START toggles control, LB+RB e-stop; sticks-centered is stopped. There is
no terminal drive — `ftc drive`/`stop` were removed when the pad took over
(all motion is `drivetrain_mecanum`).

Lower level (single module instead of the bundle):

```
mod load ecosystem/mfs/plugins/mpe_ftc.so
mod ls                      # -> ftc-fleet-1.0 [generic]
mod attach ftc-fleet        # per-world fleet allocated on primary
```

`spawn` adds a tile floor automatically when the world has none
(robots need frictional contact). `eco command mfs-simulator
<spawn|drive|list|telemetry|help> [...]` drives the full fleet API
(multi-robot, tank + mecanum, presets) that `ftc` streamlines for humans;
`eco attach` no longer pulls in the parked BioBuzz game module
(intake/shooter/balls/gamepad stay available via explicit module attach
and the direct API, and stay green in the MFS suite).

Spawning/commanding from host C (same headers, static or dlsym'd):

```c
int i = ftc_fleet_spawn(world, x, y, z, MOTOR_GB_5203_26_9, FTC_DRIVETRAIN_MECANUM);
ftc_robot *r = ftc_fleet_get(world, i);
drivetrain_tank(r, 1.0f, 1.0f);   // every tick, then:
physics_world_step(world, dt);    // module pre_step drives the fleet
physics_world_detach_module(world, "ftc-fleet");
```

## Port notes (MFS_PORT_V15S, kept from the parked tree)

- `constraint_add_revolute(world, ...)` / `constraint_pool_init(&world)`
  (world-first API); robot tests pin 128 solver iterations (40:1
  chassis/wheel stacked mass ratio; default 64 cannot converge it).
- Parked rigidbody hooks are gone from the core (`is_mecanum`,
  `roller_angle_rad`, `driven_this_tick`, wheel-lock loop). Roller
  geometry is robot-local state (`wheel_roller_angle`,
  `wheel_is_mecanum`); wheel-lock writes were deleted (wheels are woken
  directly instead).
- Joint anchors sit 2 mm below exact touch (`WHEEL_PRELOAD`) inside the
  10 mm slop, so wheels keep persistent floor contact. Ground clearance
  is 5 mm.
- Traction control (`wheel_traction_scale`): cuts torque only on
  overspeed (burnout); under-speed keeps full torque so wheels spin up
  to rolling speed instead of skidding.
- Mecanum strafe is TRANSMITTED by the analytic roller force
  (MFS-STRAFE-A, default ON for mecanum only): each wheel carries its
  roller-axle direction (`wheel_roller_angle`, X-pattern ±45°) and
  `drivetrain_mecanum_analytic` applies the Coulomb-capped (0.7*N),
  dissipative, contact-gated lateral force at the wheel contact plus its
  r×F motor-load torque — bounded by the friction cone for free, no
  chassis force ever (verified: no sin45/torque-proportional term; the
  retired `sin45·Στ/r` chassis injection stays deleted). Analytic-mode
  hubs ship zero isotropic friction (engine contact supplies the normal
  only: one tangential model, no double count), and no roller
  bodies/joints are built (6 bodies / 4 joints). This replaces the
  retired chassis-force era AND the articulated 32-roller build (kept
  behind `ftc_robot_set_mecanum_analytic_default(0)` for forensics):
  strafe +X 2.2250 m in 3 s vs 0.30 m gated; diagonal (0.5,0.5) composes
  (DESPOT-2026-10-06: this figure has now been documented three times and
  drifted twice. It was 3.40 m (a stale source comment), then 2.2872 m
  (measured), and is 2.2250 m as of the v_ref change. `mfs_suite mecanum`
  prints the truth -- read the log, not this file.)
  to (1.19,1.20) m. Odometry is therefore pure encoder kinematics
  and `odom_slip` stays honest (1 during peel, reporting only).
- Motor model: implicit-in-speed solve with disturbance observer
  (stall *and* free speed both exact), free-speed governor backstop,
  copper thermal derating, 20 A PTC fuse with brownout recovery.
- Traction budgets against wheel materials (not the global floor
  default); wheels ship grippy rubber (0.9/0.7). Idle hold is gated
  below 0.25 m/s as documented; chassis wakes on super-threshold motion
  so sleep never swallows drift.
- `physics_truth_test` bounce expectation accounts for ball radius
  (`e^2*(h-r)+r`). Motor free-speed test spins airborne wheels in
  vacuum. T14 gates position + no-runaway (steady pure roll on slop
  contacts is an engine rolling-model edge, out of MFS scope).
- `gui_robot_registry` proxies are same-world static bodies flagged
  `no_collide` (render-only: skipped by broadphase/dispatch/floor/CCD),
  with `gui_robot_despawn/clear` lifecycle and a no-double-step contract
  on `gui_robot_tick`.

## Known observations (not FTC bugs)

- Static settle shows a systematic left-high roll (~15 mm) that follows
  world X under 90° rotation: solver pair-order lock-in, not assembly
  geometry. Harmless to all gates.
- Wheel spin is bounded two ways: the free-speed governor (diode at 1.155×
  the voltage-scaled no-load point on measured speed) and the implicit
  motor solve (no discrete-time overshoot). Unbounded torque can no longer
  pump wheels to span-busting speeds through the FTC paths.
- Odometry is pure encoder forward kinematics (no chassis fusion), so any
  encoder-vs-truth disagreement is real slip and is reported as error
  rather than papered over. `odom_slip` flags >0.25 m/s planar or
  >0.35 rad/s yaw encoder-vs-truth disagreement (reporting only; odometry
  is never corrected). Encoders are PPR-quantized (counts = base_ppr x
  gear), so creep speeds staircase like hardware.
- KNOWN FAILURE [MFS-STRAFE-F1 FIXED 2026-09-28, F2 FIXED 2026-09-28]:
  F1 strafe develops 2.2250 m vs 0.30 m gated (hard-gated in the
  suite). F2 transmit fixed (physics 0.64 m in 1 s vs 0.10 m, hard-gated)
  AND the encoder tracking half is **STILL OPEN, not fixed** — measured
  2026-10-06: `physics dx = 0.6422`, `odometry dx = 0.8463`, which is
  **+31.8%**, outside the 30% band. (The v_ref 0.05->0.005 change moved this
  from +43.7%; it did not close the gate, despite a source comment claiming
  it did. Still prints `[XFAIL][MFS-STRAFE-F2]`.) The suite prints this as
  `[XFAIL][MFS-STRAFE-F2] ... tracking open`, so it is surfaced as a
  known-red marker rather than hidden, but this README described it as
  closed with better numbers than the run produces (it claimed odom 1.08 vs
  physics 0.87, i.e. ~25%, and neither figure matches). Transmit (F1) is
  genuinely fixed and hard-gated; tracking (F2) is not. Original diagnosis:
  the implicit clamp and governor took min(spec, V-line) while the explicit
  observer twin ran unclamped, so at fresh-pack voltage the paths disagreed
  6.7% and the observer carried a phantom load into peel; voltage-scaling
  both bounds to the V-line no-load point narrowed it but did not close it.
  The XFAIL branch stays as a
  fallback tripwire. `odom_slip` still flags real slip elsewhere.
  The articulated 5-link ground->roller->bearing->hub chain does not
  converge in the GS solver (0.03–0.17 m chaotic across 64–512
  iterations; solver mathematics in KNOWN_FAILURES.md); the rail-ellipse
  aggregate also measured dead (0.001 m, static-stick lock). All 8 suite
  tests hard-gate; forward/tank/hotload/module_1/physics_truth unaffected.
