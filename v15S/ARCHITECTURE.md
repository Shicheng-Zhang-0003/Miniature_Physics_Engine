# MPE Architecture — v15S

> This document describes the engine's internal structure: the kernel, the
> per-world simulation object, the module system, data structures, and the
> domain layers. It is written for developers and auditors; users should
> start with [`readme.md`](../readme.md).

---

## 1. Design Principles

MPE's headless core uses libc, libm, libdl, and pthreads. GTK4 + OpenGL 3.3
Core (via libepoxy) provide the GUI; ncurses provides the `mpe-tui` terminal
debugger. Four priorities drive every design decision:

1. **Mathematical transparency** — every integrator, solver, and collision
   test is hand-written and inspectable. No black-box physics library.
2. **Cache-efficient data layouts** — tightly packed structs and contiguous
   instance buffers.
3. **Deterministic simulation** — fixed-timestep physics decoupled from
   render framerate; bitwise-identical twins on GCC/Clang IEEE targets.
4. **Real-time scaling** — GPU instancing and a spatial-hash broadphase with
   expected near-linear behavior for ordinary distributions.

---

## 2. Domain Layers

| Layer | Directory | Responsibility |
|---|---|---|
| `core` | `v15S/src/core/` | Kernel: physics world, math, determinism, diagnostics, module ABI, loader, registry, primary world |
| `physics` | `v15S/src/physics/` | Broadphase, narrowphase, solver, constraints, joints, CCD, depenetration, islands |
| `render` | `v15S/src/render/` | OpenGL 3.3 Core rendering, instanced draws, shaders, meshing |
| `scene` | `v15S/src/scene/` | Scene I/O (v200 format, CRC32, ID remap), boundary walls, init/load/save |
| `ui_input` | `v15S/src/ui_input/` | GTK4 UI, input handling, debug terminal, FTC drive, editor |
| `tui` | `v15S/src/tui/` | `mpe-tui`: ncurses inspector + pipeable state dumps |
| `config` | `v15S/src/config/` | Tunable registry, runtime config store |
| `tests` | `v15S/src/tests/` | Headless C test suite (canonical + paranoia) |
| `ecosystem` | `v15S/src/ecosystem/` | Hot-pluggable modules: capsule shape, MFS robotics, MGB S2 bridge |

---

## 3. The Kernel: `core/physics_world`

The kernel holds **zero simulation state**. Every step takes an explicit
`physics_world*`. The single GUI process owns its primary world via
`core/mpe_primary.c` (app-ownership TU, the same role as UE's `GWorld`);
headless code should declare its own worlds.

### 3.1 `physics_world` struct

The world owns, per instance:

- **Body pool** — growable `rigidbody[]`, ×2 on demand up to `mpe_max_bodies`
  (16384). An empty world costs kilobytes.
- **Contact cache** — warm-start impulse cache (`cached_contact[]`), O(1)
  hash probes, per-world.
- **Solver scratch** — pair buffer, manifolds, island union-find arrays,
  CCD scratch arrays, tick_v0 snapshot, id→index cache.
- **Joint pools** — spring joints and revolute constraints (fixed /
  distance / prismatic / rope share the constraint slot pool).
- **Per-world config** — `mpe_config_t*` (NULL = global `g_cfg`).
- **Module slots** — up to 16 tick modules (broadphase / solver / generic
  hooks) plus per-world broadphase/solver interface overrides.
- **Diagnostics** — contact cache hits/misses/hits-applied, manifold
  overflow count, clamp counters.

Key accessors:

```c
void physics_world_init (physics_world *world);
void physics_world_cleanup (physics_world *world);
void physics_world_step (physics_world *world, float dt);
void physics_world_set_config (physics_world *world, mpe_config_t *cfg);
mpe_config_t *physics_world_get_config (physics_world *world);
static inline const mpe_config_t *mpe_world_cfg (const physics_world *world);
int physics_world_add_sphere / add_cube / add_cylinder / add_custom (...);
physics_world *physics_world_get_primary (void);
int physics_world_attach_module (physics_world *world, const mpe_module_desc_t *desc);
int physics_world_detach_module (physics_world *world, const char *name);
```

### 3.2 Body: `rigidbody`

Stored per body: position, velocity, acceleration; orientation (quaternion),
angular velocity, angular acceleration; mass / inverse_mass / restitution;
inertia tensors (local + system); force/torque accumulators; dimensions
(radius, half_extensions, cylinder_half_length); sleep state; kinematic flag;
static flag; friction_static / friction_kinetic; anisotropic friction
fields; object_id / object_generation / body_index; type (sphere / cube /
cylinder / custom); no_collide flag.

Effective-mass helpers (`rigidbody_effective_inv_mass`,
`rigidbody_effective_inv_inertia`) treat sleeping / static / kinematic bodies
as infinite mass **without mutating stored fields** — observable state is
never corrupted mid-tick and the path is thread-safe.

### 3.3 Step Pipeline (`physics_world_step`)

```
1. sanitize bodies (order-independent, parallelizable)
2. snapshot config once per tick (menu/terminal edits mid-tick can't change behavior halfway through)
3. CCD sweep clamp (time-of-impact pose, per-body remainder recorded)
4. broadphase pair generation (spatial hash, adaptive cell size)
5. narrowphase collision detection per pair (registry dispatch + builtins,
   parallel into disjoint slots); ordered wake/contact bookkeeping and solver
   preparation after the worker barrier
6. floor contact (static plane at y=0, gated by static_plane_enabled)
7. build solver islands (union-find over pairs + joints)
8. snapshot start-of-tick velocities (tick_v0, for exact free flight)
9. apply forces (gravity per mass)
10. springs (weak-linked canonical entry)
11. constraint motors (pre-integration drive)
12. foreign pre_step modules (snapshot table, live-state re-resolve)
13. integrate velocity (linear + angular damping)
14. zero sleeping body velocities
15. refresh impact velocities (Poisson gate input)
16. rebuild islands (motor wake may have changed sleep state)
17. solver iterations: per-island parallel manifold sweeps + serial joint solve
18. buckling guard (≤32 extra iterations for tall stacks with residual > slop)
19. Poisson restitution + joint re-solve
20. friction-only relaxation sweeps (×2)
21. joint re-solve (post-relaxation equalization)
22. split impulse (positional depenetration, zero velocity change)
23. late-manifold wake re-solve (newly awake pairs get solve this tick)
24. rolling resistance + joint re-solve (ABUSE-TILT protection)
25. axis-drift correction (once per tick, end-of-step)
26. contact cache save
27. integrate position (exact free-flight for contact-free/joint-free bodies;
    symplectic Euler for constrained bodies)
28. world-edge safety net (perfectly plastic backstop, gated by config)
29. positional depenetration pass
30. foreign post_step modules
```

---

## 4. Data Structures

| Structure | Size / Cap | Notes |
|---|---|---|
| Body pool | 512 → 16384 | growable ×2, ceiling is not preallocation |
| Contact cache | 4K → 64K | warm-start impulses, O(1) hash |
| Manifolds | 1K → 8K | growable atomically |
| Broadphase pairs | 65536 | spatial hash grid |
| Tick modules | 16 | per-world, named ceiling |
| Live worlds | 64 | registry for auto-purge on unload |
| Diag ring | 128 records | plus per-source totals |
| Id cache | 2048 | open addressing, linear fallback |

Bodies and contact caches store **ids (never pointers)** into the cache, so
pool reallocation during `add_*` (outside any step) is safe.

---

## 5. Module System (MPI)

Stable C ABI in `core/mpe_module.h` (`MPE_MODULE_ABI 1`). Every modular
artifact lives under `ecosystem/`; the loader jails paths there. A plain
`.so` must export `mpe_module_desc`; an ecosystem bundle exports
`mpe_ecosystem_desc` and takes precedence.

```c
typedef struct {
    uint32_t abi;          /* must equal MPE_MODULE_ABI */
    const char *name;
    const char *version;
    const char *kind;      /* "shape" | "broadphase" | "solver" | "forcefield" | "constraint" | "generic" */
    bool deterministic;
    int  (*attach)  (mpe_world_t *world, void **mod_state);
    void (*detach)  (mpe_world_t *world, void *mod_state);
    void (*pre_step)(mpe_world_t *world, float dt, void *mod_state);
    void (*post_step)(mpe_world_t *world, float dt, void *mod_state);
    void (*stage_detach)(mpe_world_t *world);  /* append-only, ABI still 1 */
} mpe_module_desc_t;
```

Rules:

- Modules never touch `g_cfg` / `g_physics_world` / `bodies[]` directly; use
  `mpe_world_*` accessors + per-world cfg (`mpe_world_cfg`).
- Per-module per-world state via `void*` (attach stores, detach frees).
- Deterministic modules must use `det_math.h`, never bare sin/cos/pow.
- Pair handlers self-unregister in a destructor; the loader purges leftovers
  by code address.
- ABI stays 1 via append-only tail fields; old `.so` binaries read new
  fields as NULL via the loader's validated accessor.
- Unload refuses `-2` (busy) while any live world references the code —
  detach/reset first, then retry. Unregister detaches every live world
  pre-`dlclose`; pair handlers self-unregister via destructor with a
  `dladdr` purge backstop.
- Tick hook tables are snapshotted, so hooks may attach/detach mid-tick.
- Modules never touch globals: per-world config via `mpe_world_cfg(world)`,
  per-module per-world state via `attach`.

### Symbol interposition warning

Several plugins export a global named `mpe_module_desc`. A library that
locates *itself* with `dladdr(&mpe_module_desc)` can be handed another
plugin's address once more than one is loaded. Use a `static` symbol as the
`dladdr` anchor instead — a local binding cannot be interposed.

### Ecosystem members

| Member | Purpose | Registry name |
|---|---|---|
| `capsule` | True segment capsule shape (`R = √(h²+rc²)`) | capsule-shape |
| `mfs` | FTC robotics fleet (mecanum, drivetrain, observer, BioBuzz) | mfs-simulator |
| `mgb` | S2 molecular bridge (display-only, S2→MPE) | s2-bridge |

---

## 6. Configuration System

`config/mpe_config.h` + `config/mpe_config_schema.h`. 80 tunables in 13
categories (world, timestep, sleep, solver, depenetration, broadphase,
joints, boundary, spawner, body_defaults, camera, render, ui). Hot-path code
reads `g_cfg.<group>.<field>` directly; cold-path code (menu, terminal,
save/load) iterates the registry.

Key behaviors:

- `mpe_config_init()` must run **before any body is built** — body materials
  are stamped from `g_cfg` at construction and never retro-fitted. Building
  a body while `g_cfg` is still zero yields a body with zero friction and
  zero restitution that can never be repaired (the `SCENEORDER-2026-09-29`
  bug, fixed structurally via `mpe_config_ensure_ready()` at every body
  initialization choke point).
- Every setter clamps to registered bounds; returns false on missing key,
  type mismatch, non-finite float, or a value that required clamping.
- `mpe_config_save_guarded()` refuses to publish F11 torture values as the
  next boot's defaults.
- Corrupt or missing config file does not crash the engine.
- Degenerate config (solver_iterations < 1, slop ≤ 0, bias ≤ 0,
  max_substeps < 1) is detected by the test harness (`mpe_cfg_is_degenerate`)
  and refuses to certify results — the old suite ran with solver iterations =
  0 and was 34/34 green.

---

## 7. Diagnostics (`core/mpe_diag.h`)

Every failure is **named**: source (subsystem), stable machine code, exact
call site (`file:line`), human message with runtime values. Each record is
fanned out in one locked pass to stderr (flushed), the event log, an
optional UI sink (debug terminal), and a 128-record ring with per-source
totals. `core/` never links `ui_input/` — the debug terminal registers a
sink callback.

Distinct negative codes for every attach/detach/loader rejection (all `< 0`,
so existing `>= 0` callers are unchanged):

| Code | Meaning |
|---|---|
| `MPE_ATTACH_E_NULL` (-1) | null world or descriptor |
| `MPE_ATTACH_E_ABI` (-2) | descriptor built against another ABI |
| `MPE_ATTACH_E_COUNT` (-3) | world tick-module count out of range |
| `MPE_ATTACH_E_TABLE` (-4) | module table full |
| `MPE_ATTACH_E_HOOK` (-5) | module's attach hook refused |
| `MPE_ATTACH_E_NAME` (-6) | module name missing |
| `MPE_DETACH_E_NULL` (-1) | null world or name |
| `MPE_DETACH_E_NOTFOUND` (-2) | nothing with that name attached |
| `MPE_DETACH_E_COUNT` (-3) | tick-module count corrupt |
| `E_NOFILE` / `E_JAIL` / `E_STALE` / `E_TABLE` / `E_DLOPEN` / `E_ABI` / `E_NODESC` / `E_BUSY` | loader rejections |

`diag`, `diag tail N`, `diag sources`, `diag on|off`, `diag clear`,
`diag status` commands in the debug terminal.

---

## 8. Rendering

Hardware-instanced OpenGL 3.3 Core:

- CPU packs model matrices + colors into contiguous buffers.
- GPU batches all dynamic bodies into **three instanced draws** (spheres,
  cubes, cylinders).
- Grid, selection outline, and spring-joint overlay share a utility shader
  with cached uniform locations.
- Custom GLSL Phong lighting (ambient + diffuse + specular).
- Equatorial axis rings (red/green/blue) on every object so rotation is
  visible at a glance.
- Shader compilation / link / missing-file failures are reported clearly;
  the engine does not silently continue in a broken render state.
