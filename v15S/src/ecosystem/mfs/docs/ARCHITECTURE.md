# MFS Architecture

How the pieces fit, who owns what, and what rules future modules must follow.
All paths below are relative to this folder (the MFS root).

## Layout

```
mfs_ecosystem.c / mfs_internal.c / mfs_internal.h
  Overarching bundle: registers both modules, owns the internal tick
  registry (mutex + cond, 16 slots, refcount + snapshot dispatch).
modules/ftc/                        # MPI module "ftc-fleet"
  ftc_module.c                      # exports mpe_module_desc (the symbol the
                                    # kernel dlopen loader dlsym's)
  ftc_fleet.c/.h                    # per-world fleet: spawn / step / get
  gui_robot_registry.c/.h           # GUI-side render-only proxies (GTK dep,
                                    # NOT in any .so — see below)
  submodules/                       # support libs, no descriptors, owned by ftc
    robot.c/.h                      # chassis + wheels + rollers + joints assembly
    drivetrain.c/.h                 # tank/mecanum mixers, traction, odometry
    motor.c/.h                      # DC electrical model (quasi-static)
    motor_presets.c/.h              # 57-motor FTC catalog (+ COUNT sentinel)
    battery.c/.h                    # sag + drain + PTC fuse
modules/module_1/                   # MPI module "mfs-simulator" (BioBuzz game)
  mfs_module_1.c/.h                 # field + robot + intake + shooter + balls
  mfs_module_1_test.c               # legacy standalone main (-D mains)
  submodules/gamepad/               # F310 joystick (module_1 only)
tests/                              # unified suite (mfs_suite_*) + legacy
                                    # per-test mains + 5 ungated diags
docs/                               # this folder
plugins/                            # build output only (gitignored)
build/                              # object files (gitignored)
```

## Validation

Two tiers, deliberately separate. Most tests re-derive what the code claims
or compare two paths through the same model — that cannot catch a shared
misconception. `external_truth` checks the model against constants and laws
that are not this project's (`g_n = 9.80665`, CODATA 2022, exact), so a
shared error cannot cancel. Full table and findings in
`docs/VALIDATION.md`.

## Module vs submodule (hard rule)

- A **module** exports an MPI descriptor (`mpe_module_desc_t`: attach /
  pre_step / post_step / detach) and has a tick lifecycle. There are exactly
  two: `ftc` (`mpe_module_desc`) and `module_1` (`mfs_module_1_desc`).
  `mfs_ecosystem.c` is a bundle exception: it links both modules' objects
  and exports both symbols; the loader takes `mpe_ecosystem_desc` first by
  documented precedence.
- A **submodule** is a support library with no descriptor, owned by exactly
  one module. Never give a submodule a descriptor or a second owner.

## Lifecycle and ownership

- **Per-world state only.** Fleets and game states allocate in `attach`,
  free in `detach`. Detach frees the fleet array / state struct but never
  deletes world bodies — bodies belong to the world and persist until the
  world is cleared or cleaned up.
- **Dangling pointers are documented, not bugs:** `ftc_fleet_get()` pointers
  dangle after detach (same rule as bodies after `physics_world_cleanup`).
  Re-fetch after re-attaching; body *indices* stay valid.
- **Tick position:** `pre_step` lands motor torques and traction in the
  force/torque accumulators immediately before velocity integration — the
  same tick position as the manual
  `drivetrain_*()` → `drivetrain_update()` → `physics_world_step()` flow,
  so module-driven and host-driven robots behave identically (proven by
  `ftc_hotload`: static vs dlopen bitwise-identical pose + odometry).
- **Practical capacity:** the fleet array grows 4→32 (hard cap), but each
  mecanum robot adds 4 wheels + up to 32 roller bodies + ~36 joints to one
  solver island at 128 iterations. Treat 2–4 robots as the practical limit.

## Concurrency

- `mfs_internal.*` is the only shared mutable state: pthread mutex + cond,
  16 slots, `in_flight` refcount, snapshot-then-invoke dispatch (hooks may
  attach/detach mid-tick; use-after-free and double-attach races closed).
  Three per-slot fields carry that contract: `in_flight` (detach blocks
  until it drains), `detaching` (new callbacks are refused once a detach
  owns the slot, so the drain always terminates), and `attached` (published
  only after `attach()` returns, so a concurrent attach either waits or
  takes the idempotent path). `mfs_internal_modules_detach_all()` was
  re-checked against `mfs_internal_module_detach()` during the 2026-10-02
  audit specifically for the `detaching` reset on the no-callback path and
  is correct on both.
- Physics stepping itself is single-threaded and deterministic in
  force/torque paths; both modules report `deterministic = false` honestly
  because odometry integrates the heading with libm `cosf/sinf` and
  module_1 polls a live gamepad. Same machine + same libm + same build is
  bit-stable (hotload proves it).

## Build invariants (do not break these)

- **Repository hygiene:** `build/`, `temp/`, `plugins/` and all object/shared
  objects are gitignored. This was documented from the start and **implemented
  on 2026-10-02**: there had been no `.gitignore` at all, and 13 `.o`/`.so`
  files were tracked since the initial import (`02fe0b2`). Binary objects in
  history are unreviewable in diffs and carry dead weight to every clone.
  Tracked file count went 67 -> 54. If you see an artifact in `git status`
  as *tracked*, that is a regression.
- **Thin `.so` pattern:** module shared objects contain ONLY MFS objects;
  engine symbols resolve against the `-rdynamic` host at dlopen. Linking
  engine objects into the `.so` caused version skew and duplicate symbols.
- **Stale-`sizeof` hazard:** MFS objects are compiled against engine headers
  (`core/rigidbody.h` lays out the bodies array the plugin indexes). The
  Makefile emits `.d` sidecars (`-MMD -MP`) so engine-header edits force a
  rebuild — a stale `sizeof(rigidbody)` once made spawns fail with "no fleet
  attached". Never drop dependency tracking.
- **`-O3` (.so) vs `-O2` (tests)** is a disclosed FP divergence: bit-identity
  holds only within one build config + libm.
- **`gui_robot_registry.c` is excluded from every `.so`** (GTK dependency,
  globals outside the module system). Its proxies are same-world static
  bodies flagged `no_collide` (render-only: skipped by broadphase, dispatch,
  floor contact, and CCD), with `gui_robot_despawn/clear` lifecycle and a
  no-double-step contract on `gui_robot_tick`.
- **One descriptor per `.so`** (bundles excepted); the loader `dlsym`s
  exactly that symbol. The hotload test intentionally loads an image with a
  second `mpe_module_desc`, so ASan runs suppress only the ODR heuristic
  (`detect_odr_violation=0`); leaks and UBSan stay armed.

## Include convention (location-independent)

Never use `../` crosses — they break on every move:

- engine headers → `core/...`, `physics/...`, `config/...` (`-I` engine `src/`)
- MFS-internal → `modules/...` from the MFS root (`-I` this folder)
- siblings within one directory stay bare (`"robot.h"`, `"gamepad.h"`)
