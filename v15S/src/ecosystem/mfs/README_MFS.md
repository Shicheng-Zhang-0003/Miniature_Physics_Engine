# MFS — overarching module ecosystem ("mfs-simulator")

Everything MFS-wise lives under this folder: modules and submodules
contained within the overarching MFS module ecosystem. (Previously
`v15S/robotics_backup/` + scattered `mfs_*` dirs; consolidated here with
history preserved via `git mv`.)

```
v15S/src/ecosystem/mfs/                  # MFS root ("mfs-simulator")
  README_MFS.md                          # this file
  Makefile                               # unified standalone build (thin .so, build/ objs)
  mfs_sources.mk                         # canonical engine+FTC file lists (mirrored in build_tests.sh)
  build_tests.sh                         # FTC/robotics test build + run (11 gated + 5 info)
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
                                         # ftc integration, physics truth,
                                         # hotload, module_1 test,
                                         # mfs_test_common.h (shared setup:
                                         #   128 iters + tile floor),
                                         # (+5 ungated diags)
  docs/
    FTC_SPECS.md                         # motor spec-sheet sources + URLs
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

From `v15S/src`:

```
ecosystem/mfs/build_tests.sh            # full FTC suite (11 gated tests + 5 info diags + build checks)
ecosystem/mfs/build_tests.sh --build-only
```

Binaries and logs go to `../../temp/ftc_tests` by default (override with `OUTDIR=...`).
The script also stages `mpe_ftc.so` at `src/plugins/` — the kernel loader
only accepts `plugins/<name>.so` under its working directory, so that copy
is what `mod load` and the hotload test use (same binary as
`mfs/plugins/`; both gitignored build output). Test mains select via `-D`.

Standalone (out-of-tree, engine untouched): `make -C ecosystem/mfs`
(`mfs_module_1.so`, `mfs_ecosystem.so`, `mpe_ftc.so` into `plugins/`),
`make -C ecosystem/mfs test`, or from `v15S/src`: `make mfs_ecosystem.so`.

Inside the engine terminal (no rebuild needed — full drive session):

```
mod load ecosystem/mfs/mfs_ecosystem.so   # load the bundle
eco attach mfs-simulator                  # attach it to the primary world
ftc spawn                                 # mecanum robot at the origin
ftc drive 0 tank 1 1                      # full forward (persists)
ftc telemetry 0                           # pose, odometry, battery, wheels
ftc drive 0 stop
```

Lower level (single module instead of the bundle):

```
mod load plugins/mpe_ftc.so
mod ls                      # -> ftc-fleet-1.0 [generic]
mod attach ftc-fleet        # per-world fleet allocated on primary
```

`spawn` adds a tile floor automatically when the world has none
(robots need frictional contact). `eco command mfs-simulator
<spawn|drive|list|telemetry|help> [...]` drives the same surface the
`ftc` commands use; `eco config mfs-simulator get shooter_rpm` reads
bundle config.

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
- Mecanum strafe is a torque-derived roller force on the chassis
  (`sin45·Στ/r` from instantaneous motor torques), capped at 1.1× the
  static cone as a documented breakaway margin: the isotropic contact
  model cannot roll sideways, so the stand-in must exceed static where
  real rollers would roll. `odom_slip` flags every fused tick; encoder
  math itself is exact. Anisotropic roller friction in the solver remains
  the principled long-term model.
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
- Wheel spin is bounded two ways: the free-speed governor (hard backstop
  at 1.1× free speed) and the implicit motor solve (no discrete-time
  overshoot). Unbounded torque can no longer pump wheels to span-busting
  speeds through the FTC paths.
- Odometry tracks physics within ~5% on tile floors (was 45% error with
  sign flip on the frictionless backstop). Lateral encoder blindness is
  structural (roller thrust bypasses wheels): `odom_slip` flags fused
  ticks instead of hiding them.
