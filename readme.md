# 🧊 MINIATURE PHYSICS ENGINE (MPE)

> **Active head:** `v15S` — GTK4 port of the `v15R3` release with a modular kernel, per-world config, and upgraded data structures. Build from `v15S/src`; run the complete verification matrix from the repository root with `python3 tools/test_runner.py --profile full`.

**License:** GPL-3.0 · **Language:** C · **UI:** GTK4 · **Renderer:** OpenGL 3.3 Core

---

## 📋 Overview

MPE is a custom-built **3D rigid-body physics engine and real-time rendering pipeline**, written entirely in **C**. It runs on a **zero-dependency core** — engine needs **GTK4** (windowing/UI) + **OpenGL via libepoxy** + **X11**; `mpe-tui` additionally needs **ncurses**. Headless tests need only libc+libm+libdl.

MPE is built around four priorities:

- **Mathematical transparency** — every integrator, solver, and collision test is hand-written and inspectable.
- **Cache-efficient data layouts** — tightly packed structs and contiguous instance buffers.
- **Deterministic simulation** — fixed-timestep physics decoupled from render framerate.
- **Real-time scaling** — GPU instancing and an O(N) spatial-hash broadphase.

The kernel is fully modular: every pipeline stage (broadphase, narrowphase shapes, solver, joints, force fields) is hot-swappable at runtime via `.so` plugins — see [Module system](#-module-system-hot-plug-physics) below.

---

## ✨ What's New in v15S

`v15S` is the GTK4 + modular-kernel evolution of `v15R3` (whose release record is preserved in [`release_notes_v15R3.md`](release_notes_v15R3.md)). Highlights over `v15R3`:

- **GTK4 port** — event controllers, gestures, overlay, dialogs; Wayland-safe input (no more X11-only mouse lock).
- **Module system (MPI)** — stable C ABI (`core/mpe_module.h`): register/override shapes, broadphase, solver stages, and tick modules; load `.so` plugins live (`mod load`), attach/detach per world.
- **Per-world configuration** — every physics function takes the owning world's config (`NULL` = global default); two worlds can run different gravity/iterations/slop side by side.
- **Upgraded data structures** — growable body/contact pools (512→16384, 4K→64K ceilings), O(1) contact-pair hash probes, per-world id→index cache (islands O(J+B)), small-first broadphase node pool, process-wide determinism counters.
- **No kernel globals** — the last old-series global (`g_physics_world`) moved to an app-ownership TU (`core/mpe_primary.c`); the kernel holds zero simulation state. The type/plugin registry stays process-global by design (documented in `core/mpe_registry.h`).
- **TUI stress suite** — new `stress` (300 bodies + every joint type) and `ccd` (60/144/300 m/s battery) scenes, `--broadphase/--solver` backend flags, per-scene configs, pool visibility in dumps.

Inherited from `v15R3`:

- **Domain-driven architecture** — clean `core`, `physics`, `render`, `scene`, `ui_input` modules.
- **Warm-starting contact solver** with multi-point Sutherland–Hodgman manifolds for stable stacking.
- **Full constraint framework** — revolute, fixed, prismatic, distance, and rope constraints plus spring joints (scene v200 persists springs + all five constraint types; all types live in the headless suite and the TUI demo).
- **3D spatial-hash grid broadphase** with adaptive cell sizing.
- **Interactive spring-joint system** with live magenta rendering.
- **POSIX-style debug terminal** — drive the whole simulation from a shell (now with a `mod` command for hot-plugging physics).
- **Built-in validation suite** (F5–F11), including a 60-second long-run stability test and config torture test.
- **Shader/render failure visibility** — the engine no longer continues silently in a broken render state.
- **Physics-truth pass** — Verlet-exact free flight, post-integration Poisson gate, CCD remainder integration, strict warm-start, true cylinder SDF geometry, no velocity clamps or restitution caps; game-only damping (`nice_value`, angular scale) labeled and defaulted off/vacuum.
- **Sleep truth** — three-gate wake (first-touch pair novelty, fast-other, deep overlap): slow pushers wake sleepers at any speed, resting stacks proceed to sleep and stay settled (F10 root cause, fixed).
- **Terminal debugger + output suite** — `mpe-tui`: live ncurses inspector (bodies, joints, constraints, math, scene graph) plus pipeable `--snapshot`/`--stream` state dumps; see [below](#-terminal-debugger--output-suite-mpe-tui).

---

## 🎨 Rendering System

### Hardware-Instanced Rendering

MPE eliminates per-object draw calls using **GPU instancing**:

- The CPU packs model matrices + colors into contiguous buffers.
- The GPU batches all dynamic bodies into **three instanced draws** (spheres, cubes, cylinders).
- The grid, selection outline, and spring-joint overlay share a utility shader with cached uniform locations.

### Shading

- Custom **GLSL Phong** lighting (ambient + diffuse + specular).
- **Equatorial axis rings** painted on every object (red/green/blue) so rotation is visible at a glance.

---

## ⚙️ Physics Engine

### Broadphase — Spatial Hash Grid

Objects are mapped into hashed grid buckets; collision checks are limited to local neighborhoods for **average O(N)** scaling. Cell size adapts to object radii. Node pools start small (4K) and double to a 1M cap with overflow telemetry. A sleep system removes inactive bodies from the solver. The backend is swappable (`hash` builtin; `--broadphase` / `mod use-broadphase`).

### Narrowphase

| Pair | Method |
|---|---|
| Sphere–Sphere | Analytical distance test |
| Sphere–OBB | Closest-point projection |
| OBB–OBB | Separating Axis Theorem (15 axes) + Sutherland–Hodgman face clipping |
| Cylinder–* | Exact solid-cylinder SDF (caps, rim, inside) / segment-OBB / coaxial |
| Custom | Foreign shapes via the pair registry (`mpe_register_pair_handler`) |

All narrowphase functions take the owning world's config; dispatch is registry-first with builtin fallback (one shared implementation — no duplicated per-path chains).

### Solver

- **Impulse-based sequential solver**, 64 iterations by default (configurable 1–128), with **warm starting** (per-world O(1) hash cache).
- Every stage (resolve, Poisson restitution, rolling resistance, split impulse) is an optional module hook — foreign solvers observe or replace per stage.
- Static + kinetic friction, rolling friction, Catto split-impulse penetration correction (no velocity Baumgarte).
- Positional depenetration pass for pile stability (registry-routed, per-world config).

### Integration

- **Semi-implicit (symplectic) Euler** for linear motion.
- **Quaternion-based angular integration** (no gimbal lock).
- **Fixed 60 Hz timestep** with an accumulator and 5-substep cap (spiral-of-death prevention).

### Data structures

- Bodies and contact caches grow ×2 from small initials to compile-time ceilings; an empty world costs kilobytes.
- Contact-pair novelty probes and id→index lookups are O(1) per-world hash operations (verified + linear fallback inside).
- Determinism fallback counters are process-wide and asserted zero in-contract by `module`.

---

## 🧩 Module system (hot-plug physics)

Stable ABI in `core/mpe_module.h` (`MPE_MODULE_ABI 1`):

```c
mpe_register_pair_handler(3, 0, 100, -1, capsule_vs_sphere, "capsule-sphere");
physics_world_set_solver(world, mpe_find_solver("seq-impulse"));
physics_world_attach_module(world, desc);   // pre_step / post_step hooks
```

- Shapes: `object_custom` bodies (id ≥ 100) dispatch through the registry; see `plugins/mpe_capsule.c` (true segment capsule; bounding invariant `R = √(h²+rc²)`).
- Backends: `mpe_register_broadphase` / `mpe_register_solver`; builtins `hash` + `seq-impulse` (takeover refused; per-stage `mod_state` threaded through both step paths).
- Loading: `mpe_loader_load("plugins/mpe_capsule.so")` (ABI-checked `dlopen`, CWD-jailed), or live in the debug terminal: `mod load|unload|attach|detach|use-broadphase|use-solver|ls`, or in TUI: `--solver NAME --broadphase NAME`. Ecosystem bundles (`mpe_ecosystem_desc`) load from `ecosystem/mfs/*.so`.
- Lifetime rules: unload refuses `-2` (busy) while any live world references the code — detach/reset first, then retry. Unregister detaches every live world pre-`dlclose`; pair handlers self-unregister via destructor with a `dladdr` purge backstop. Tick hook tables are snapshotted, so hooks may attach/detach mid-tick.
- Modules never touch globals: per-world config via `mpe_world_cfg(world)`, per-module per-world state via `attach`.

---

## 🧮 Mathematics Core

A fully custom, dependency-free math library: 3D vectors, 4×4 matrices, quaternions, and inertia tensors — designed for tightly packed, cache-friendly structs. Out-of-contract transcendentals fall back to libm through counted, process-wide diagnostics (`det_fallback_*_total()`).

---

## 🔍 Terminal debugger & output suite (`mpe-tui`)

A terminal-only companion to the GTK engine — a live inspector and a
scriptable state-dump suite in one binary (needs only ncurses):

```bash
cd v15S/src
make mpe-tui
./mpe-tui                         # live ncurses inspector (needs a TTY)
./mpe-tui --snapshot 600          # one full state dump (pipeable, diffable)
./mpe-tui --stream 600 --every 60 # dumps over time
./mpe-tui --snapshot 10 --scene tower|pendulum|springlab|f10|demo|stress|ccd
./mpe-tui --snapshot 60 --scene demo --solver seq-impulse --broadphase hash
```

Live screens: overview table, per-object characteristics + mathematics
(quaternion, euler, inertia tensors, momentum, energy), joint/constraint
detail with live endpoint geometry, pairwise scene graph, help. Snapshot
sections (`[engine]`, `[body i]`, `[springs]`, `[constraints]`, `[pairs]`,
`[islands]`, `[stats]`, `[result]`) are fixed-format and deterministic —
`make tui-smoke` checks every scene dumps finite state. Dumps report pool
usage (`bodies=n/cap`, `cacheCount=n/cap`).

---

## 🖥️ Platform & Rendering Stack

| Layer | Technology |
|---|---|
| Windowing / UI | GTK4 |
| Graphics API | OpenGL 3.3 Core (via libepoxy) |
| Lighting | Custom GLSL Phong |
| Debug visualization | Axis rings, wireframe selection, joint lines, overflow counters |

---

## 🎮 Controls

### Movement & Camera

| Action | Input |
|---|---|
| Move | `W A S D` |
| Look around | Mouse (left-click to lock) |
| Jump / fly up | `Space` |
| Fly down (Debug) | `Shift` |
| Steer camera mouse-free (Debug) | `I J K L` |
| Release mouse | `Escape` |
| Toggle Game / Debug mode | `0` |

### Spawning

| Action | Input |
|---|---|
| Spawn object | Hold `Enter` |
| Spawner settings | `8` |

### Selection & Editing

| Action | Input |
|---|---|
| Select object | Right-click (raycast) **or** `R` (Debug) |
| Open object menu | `E` |
| Apply impulse | `F` |
| Delete object | Middle-click |
| World settings | `7` |
| Save / Load scene | `9` |

### Debug Terminal & Validation

| Action | Input |
|---|---|
| Open debug terminal | `1` (Debug) |
| Stability stack test | `F5` |
| Sleep / wake test | `F6` |
| Editor torture test | `F7` |
| Spawn stress test (300 objects) | `F8` |
| Validation report | `F9` |
| Long-run validation (60 s) | `F10` |
| **Config torture test** | **F11** |

---

## 🐚 Debug Terminal

In Debug Mode, press `1` to open a **POSIX-style shell** over the physics world. The simulation is exposed as a virtual filesystem:

| Path | Contents |
|---|---|
| `/obj` | All rigid bodies |
| `/joint` | All spring joints |
| `/world` | World variables (gravity, drag, friction) |
| `/camera` | Camera state |
| `/spawner` | Spawner settings |

A few examples:

```
touch new.sph            # spawn a sphere
ln 1 2                   # spring-join objects 1 and 2
mv 3 /pos/0/10/0         # teleport object 3
chown 5.0 3              # set object 3's mass to 5 kg
chmod static 3           # make it immovable
kill -STOP 3             # put it to sleep
ps aux                   # list every body with state
export GRAVITY=-2.0      # change world gravity
mod ls                   # list loaded physics modules
mod load ./plugins/mpe_capsule.so   # hot-plug a foreign shape
```

Type `help` for the full command list, `man <command>` for usage. `Ctrl+L` clears, `Esc` closes. Mutating commands require Debug Mode; in Game Mode the terminal is read-only.

---

## 🧪 Validation Tests

MPE ships with built-in stability tests:

| Key | Test |
|---|---|
| `F5` | 10-cube stability stack |
| `F6` | Sleeping cube + moving projectile (sleep/wake) |
| `F7` | Editor torture: select, joint, delete, reset |
| `F8` | Spawn stress: up to 300 mixed objects |
| `F9` | Print validation report |
| `F10` | Long-run validation: 3600 ticks (60 s) of idle stability |
| `F11` | Config torture: 78 tunables randomised to extremes, then 3600 ticks |

`F10` monitors for NaN values, fallen objects, and residual motion, printing `PASS`/`FAIL` at the end. `F11` is a robustness verdict — `PASS` means no NaN and nothing fell through the world (speeds reported, never gated; under extremes, perpetual fall/creep can be the true outcome). Each F11 press uses the next printed seed. Torture pins solver resolution (gravity −17…−1, ≥96 iterations — proven envelope for the 10:1 validation column) while material/world extremes stay fully random.

---

## 🛠️ Build Instructions

### Dependencies (Ubuntu / Debian)

```bash
sudo apt update
sudo apt install build-essential pkg-config libgtk-4-dev libepoxy-dev
# Optional, for the mpe-tui terminal debugger:
sudo apt install libncurses-dev
```

### Build and run

```bash
cd v15S/src
make clean
make
./engine
```

---

## ⚠️ Known Limitations

- **Scene format:** v200 saves bodies (with stable IDs, sleep state, damping) plus springs + revolute/fixed/distance/prismatic/rope joints, with a CRC32 integrity footer. Files v130/v140/v150/v151/v152/v153 load via the legacy reader.
- **Global state:** the kernel holds no simulation state — every step takes an explicit `physics_world` (growable pools, per-world caches/scratch/config). The single GUI process owns its primary world via `core/mpe_primary.c` (app layer, like UE's `GWorld`); the type/plugin registry is process-global by design. App/UI state (camera, input, selection, terminal, diagnostics) remains global by design.
- **Joint UI/persistence scope:** solver supports spring/revolute/fixed/prismatic/distance/rope; menus create springs (+revolutes in TUI scenes); v200 persists springs + revolute/fixed/distance/prismatic/rope.

---

## 📜 Version History

- **v15S (current head)** — GTK4 port, module system (MPI hot-plug), per-world config, data-structure upgrades (growable pools, O(1) caches), kernel global-state removal, TUI stress suite (`stress`/`ccd` scenes, backend flags), 32/32 headless green (29 physics + 3 diag-informational).
- **v15R3 (release)** — configuration system, physics-truth pass, full constraint framework, TUI debugger + snapshot suite, 29/29 headless green. Release notes: [`release_notes_v15R3.md`](release_notes_v15R3.md).
- **v15R2** — config-system hardening + MFS robotics (prior RC, parked (now consolidated in `v15S/src/ecosystem/mfs/`)).
- **v1.4 Alpha RC3** — domain-driven restructure, spatial-hash broadphase, physics-world encapsulation.
- **v1.4 Alpha 2** — warm-starting solver, multi-point contact manifolds.
- **v1.4 Alpha RC1** — spring joints, joint renderer, color painting, OBB raycast selection.
- **v1.3** — established instanced rendering and spatial-hash direction.

See [`v15S/evolution.txt`](v15S/evolution.txt) for the full lineage back to stage 0. Release notes: [`release_notes_v15R3.md`](release_notes_v15R3.md).

---

### Screenshots

<img width="4424" height="1824" alt="Screenshot from 2026-07-18 17-18-52" src="https://github.com/user-attachments/assets/5d1d044d-3926-469e-ab27-9f3719452324" />
<img width="4558" height="1908" alt="Screenshot from 2026-07-18 17-20-09" src="https://github.com/user-attachments/assets/acebe348-707e-485e-835c-08cd1b1dc0fa" />

---

## 🧪 Verification suite

The unified test runner provides quick, physics, and full profiles. The full
profile builds the active engine, checks the canonical 32-case C suite, runs
all 30 isolated legacy cases and 14 paranoia cases, then repeats the physics,
MFS, and TUI suites under AddressSanitizer and UndefinedBehaviorSanitizer. It
also checks generated TUI snapshots and writes JSON and JUnit reports, snapshots,
and per-command logs below each run's directory in `temp/qa_runs/`.

```bash
python3 tools/test_runner.py --profile quick   # runner contracts + canonical C suite
python3 tools/test_runner.py --profile physics # also isolated legacy + paranoia cases
python3 tools/test_runner.py --profile full    # full build, physics, MFS, TUI, sanitizers
python3 tools/test_runner.py --list            # discover registered test cases
```

The canonical C suite runs all cases in one process with shared audited builders
(exact-name dispatch, saved/restored config, real Coulomb floors everywhere).
The full profile also runs each older C case in a separate process to catch
cross-test contamination. The runner verifies that reported case names exactly
match the C registry, distinguishes informational diagnostics from gates, and
fails on missing, duplicate, or malformed test output. Its 11 Python contract
tests cover registry discovery, result parsing, report generation, TUI snapshot
validation, and command-launch failures.

| Test | Proves |
|------|--------|
| `two_world` | Independent `physics_world` instances |
| `revolute` | Hinge joints hold anchor and allow swing |
| `cylinder_drop` | Cylinder settles on the floor |
| `driven_wheel` | Torque → friction → translation (grounded, coupled) |
| `math3_inverse` | Matrix inverse at small inertia tensors |
| `floor_collision_diag` | Floor contact diagnostics |
| `cylinder_sphere/cube/cylinder` | Cylinder narrowphase pairs |
| `list4_cylinder_floor` | Tipped-cylinder floor regression |
| `scene_roundtrip` | Save/load v200 round-trip (springs + all constraint types) |
| `static_hold` | Coulomb stick holds / yields past friction angle |
| `rolling_decay` | Contact-patch rolling resistance decay |
| `ccd_sweep` | Swept TOI: no tunneling at 144 m/s |
| `kinematic` | Velocity-driven platforms carry bodies |
| `determinism` | Twin worlds agree bitwise over 600 ticks |
| `momentum` / `angmom` | Linear / angular momentum conservation |
| `spring` | Hooke period + bounded energy |
| `projectile` | Verlet-exact free-flight parabola |
| `incline_accel` | Slope acceleration matches `g·sinθ` |
| `pendulum` | Revolute pendulum period |
| `bounce_series` | Poisson restitution series |
| `friction_stop` | Coulomb stopping distance `v²/(2μg)` |
| `stack` | 6-cube tower stands |
| `f10_long_run` | F10 settle gates on the validation scene |
| `sleep_contact_wake` | Slow pushers wake sleepers; resting contact doesn't churn |
| `f11_torture` | Deterministic config extremes without corruption |
| `frustum` | Frustum culling math |
| `module` | Per-world config, registry dispatch, custom shapes, solver hooks, id cache, pool growth, det counters |
| `loader_lifecycle` | Plugin load/busy-unload/purge, builtin-hijack refusal, stage reset, `mod_state` threading |
| `ftc_ecosystem` | Bundle load → attach → spawn/drive/telemetry → detach/unload end-to-end |

Run the focused canonical suite with `python3 tools/test_runner.py --profile quick`; use `--profile full` for all registered suites and sanitizers.

### MFS robotics (`v15S/src/ecosystem/mfs/`)
- **FTC stack**: motor presets (spec-sheet derived), back-EMF electrical model with implicit-in-speed solve + disturbance observer (stall *and* free speed exact), traction budgeting against wheel materials, emergent roller-anisotropy mecanum strafe (no chassis-force cheat; lateral grip emerges from the anisotropic contact ellipse), pure-encoder odometry with `odom_slip` flag, tile-friction test floors.
- **Suite**: `build_tests.sh` — 11 gated tests + 5 informational diagnostics, all green (was 13 pass / 3 fail + broken `make`).
- **Modules**: `ftc-fleet` tick module (hot-pluggable, bitwise-identical static vs `.so`), `mfs_module_1` game module, `mfs-simulator` ecosystem bundle (loadable via `mod load ecosystem/mfs/mfs_ecosystem.so`).

### Determinism and precision
- Fixed 1/60 s timestep, fixed solver iteration order, exact IEEE `+ - * / sqrt`.
- Per-tick transcendentals (damping retention, rotation rotors) use fixed-coefficient polynomials (`v15S/src/core/det_math.h`), bit-identical on all IEEE-754 targets at `ENABLE_NATIVE=0`; `ENABLE_NATIVE=1` (-march=native) breaks bitwise portability. The build disables FP contraction (`-ffp-contract=off`). Fallback counters are process-wide (`det_fallback_*_total()`, asserted zero in-contract).
- Proven by `determinism`: twin worlds agree bitwise over 600 ticks.
- float32 world: the playable volume is bounded (±250 m), where float resolution (~0.03 mm at the corners) sits 300× below contact slop. No origin rebasing required inside the boundary box.
