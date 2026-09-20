# MFS robotics — first citizen of the MPE kernel module ecosystem

This folder was parked because the robot bounced on wheel contact
mid-transition. It is now ported to the current core APIs, all
12 headless tests/diagnostics run green (see `build_tests.sh`), and —
new — the whole stack ships as a **hot-pluggable kernel module**:
`plugins/mpe_ftc.so`, importable at runtime with zero engine changes.

- `robotics/` — battery, motor, motor presets, drivetrain, robot, fleet,
  module descriptor, GUI registry
- `gamepad/` — F310 joystick drive input (standalone support lib, Linux only)
- `tests_robotics/` — teleop, mecanum, ftc integration/debug, tank turn,
  odometry (+diags), idle-spin diags, physics truth (+diag), hotload
- `plugins/` — built `mpe_ftc.so` lands here (gitignored build output)

## Module ecosystem: hot plug and play

The engine already speaks MPI (`mpe_module_desc`, registry, `dlopen`
loader, `mod` terminal command). FTC is its first real ecosystem
member — a `generic` tick module named **`ftc-fleet`**:

```
# inside the engine terminal (./engine, no rebuild needed):
mod load ../robotics_backup/plugins/mpe_ftc.so
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

Design rules the module follows (and future modules should too):

- **One descriptor per `.so`** (`mpe_module_desc` in `ftc_module.c`);
  the loader `dlsym`s exactly that symbol.
- **Per-world state only**: the fleet array is allocated in `attach`,
  found per world by descriptor name, freed in `detach`. Detach never
  deletes world bodies — same ownership as manual creation.
- **Tick position**: `pre_step` calls `drivetrain_update()` per fleet
  member, landing motor torques/traction in the accumulators right
  before velocity integration — identical to the manual
  `drivetrain_update()` + `physics_world_step()` flow.
- **Determinism flag is honest**: `false`, because odometry integrates
  the heading frame with libm `cosf/sinf`. Force/torque paths are
  IEEE-exact; same machine + same libm is bit-stable (proven by
  `ftc_hotload_test`, which asserts bitwise-identical pose+odometry
  across the statically-linked and dlopen'd copies).
- **Hosts must export engine symbols** (`-rdynamic`, as the engine's
  own makefile already does) so the `.so`'s references
  (`rigidbody_wake`, `constraint_add_revolute`, `g_cfg` via the
  `mpe_world_cfg` accessor, …) resolve at load.

`ftc_hotload_test` proves all of it: static attach, `dlopen` +
`dlsym` full import, identical 2.64 m drives, bitwise equivalence,
detach/re-attach lifecycle, unload.

## Build & run (out-of-tree; the engine is never touched)

From `v15S/src`:

```
../robotics_backup/build_tests.sh
```

Binaries and logs go to `/tmp/ftc_tests` (override with `OUTDIR=...`).
`--build-only` compiles without running. Compile flags mirror the engine
(`-I. -I..`, `-ffp-contract=off`); test mains select via `-D`.

## Port notes (MFS_PORT_V15S)

- Includes rewritten for the parked layout: engine headers as
  `core/...`, `physics/...`, `config/...`; sibling headers as
  `robotics_backup/robotics/...`. Compile from `v15S/src` with `-I. -I..`.
- `constraint_add_revolute(world, ...)` / `constraint_pool_init(&world)`
  (world-first API); robot tests pin 128 solver iterations (40:1
  chassis/wheel stacked mass ratio; default 64 cannot converge it).
- Parked rigidbody hooks are gone from the core (`is_mecanum`,
  `roller_angle_rad`, `driven_this_tick`, wheel-lock loop). Roller
  geometry is robot-local state (`wheel_roller_angle`,
  `wheel_is_mecanum`); wheel-lock writes were deleted (wheels are woken
  directly instead).
- Joint anchors sit 2 mm below exact touch (`WHEEL_PRELOAD`) inside the
  10 mm slop, so wheels keep persistent floor contact; the old pose
  tucked wheel tops 10 mm inside the chassis and the solver fought the
  joint-held overlap every tick. Ground clearance is 5 mm.
- Traction control (`wheel_traction_scale`): cuts torque only on
  overspeed (burnout); under-speed keeps full torque so wheels spin up
  to rolling speed instead of skidding.
- Mecanum strafe is a reduced-order roller force on the chassis
  (friction-circle budgeted); rotate works through real wheel
  differentials. Anisotropic roller friction in the solver remains the
  principled long-term model.
- `physics_truth_test` bounce expectation accounts for ball radius
  (`e^2*(h-r)+r`; the old `e^2*h` was 33% off while the engine was 2%
  off). Motor free-speed test settles before driving, like its
  siblings.
- `gui_robot_registry` proxies are same-world static bodies (the retired
  `obj_per_scene`/`object_count` globals are gone).

## Original drive-tuning notes (kept for the MFS return)

Slop-gated zero-depth contacts (`penetration_slop`) restored persistent
wheel friction (teleop 0.92 m, strafe +X 0.49 m in 3 s); per-iteration
revolute axis-drift correction holds wheels upright; commanding must
wake the chassis (velocity integrator drains sleeping-body forces).

## Known engine-side observations (not FTC bugs)

- Static settle shows a systematic left-high roll (~15 mm) that follows
  world X under 90° rotation: solver pair-order lock-in, not assembly
  geometry. Harmless to all gates; do not chase it from here.
- Unbounded raw torque can pump a light wheel past ~2000 rad/s, where
  swept broadphase spans explode (minutes per tick). The FTC torque
  paths (back-EMF + traction control) bound wheels near free speed, so
  robot code cannot reach that regime. A span cap belongs in the
  broadphase when the engine is next touched.
