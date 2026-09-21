# MFS — overarching module ecosystem ("mfs-simulator")

Everything MFS-wise lives under this folder: modules and submodules
contained within the overarching MFS module ecosystem. (Previously
`v15S/robotics_backup/` + scattered `mfs_*` dirs; consolidated here with
history preserved via `git mv`.)

```
v15S/src/ecosystem/mfs/                  # MFS root ("mfs-simulator")
  README_MFS.md                          # this file
  Makefile                               # unified standalone build
  build_tests.sh                         # FTC/robotics test build + run
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
  tests/                                 # teleop, mecanum, tank, odometry
                                         # (+diags), ftc integration/debug,
                                         # physics truth (+diag), hotload
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
ecosystem/mfs/build_tests.sh            # full FTC suite (13 tests + diags)
ecosystem/mfs/build_tests.sh --build-only
```

Binaries and logs go to `/tmp/ftc_tests` (override with `OUTDIR=...`).
The script also stages `mpe_ftc.so` at `src/plugins/` — the kernel loader
only accepts `plugins/<name>.so` under its working directory, so that copy
is what `mod load` and the hotload test use (same binary as
`mfs/plugins/`; both gitignored build output). Test mains select via `-D`.

Standalone (out-of-tree, engine untouched): `make -C ecosystem/mfs`
(`mfs_module_1.so`, `mfs_ecosystem.so`, `mpe_ftc.so` into `plugins/`),
`make -C ecosystem/mfs test`, or from `v15S/src`: `make mfs_ecosystem.so`.

Inside the engine terminal (no rebuild needed):

```
mod load plugins/mpe_ftc.so
mod ls                      # -> ftc-fleet-1.0 [generic]
mod attach ftc-fleet        # per-world fleet allocated on primary
```

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
- Mecanum strafe is a reduced-order roller force on the chassis budgeted
  in the net-vector friction circle; rotate works through real wheel
  differentials. Anisotropic roller friction in the solver remains the
  principled long-term model.
- `physics_truth_test` bounce expectation accounts for ball radius
  (`e^2*(h-r)+r`). Motor free-speed test settles before driving.
- `gui_robot_registry` proxies are same-world static bodies.

## Known observations (not FTC bugs)

- Static settle shows a systematic left-high roll (~15 mm) that follows
  world X under 90° rotation: solver pair-order lock-in, not assembly
  geometry. Harmless to all gates.
- Unbounded raw torque can pump a light wheel past ~2000 rad/s, where
  swept broadphase spans explode. The FTC torque paths (back-EMF plus
  traction control) bound wheels near free speed, so robot code cannot
  reach that regime. A span cap belongs in the broadphase when the
  engine is next touched.
- Odometry over-reads distance ~4x under chronic wheel slip (floor μ 0.2
  in the test worlds vs tile-like grip): the encoder FK math is correct,
  the wheels physically spin. Coherent fix is tile friction in the robot
  test setups plus threshold recalibration — not a model change.
