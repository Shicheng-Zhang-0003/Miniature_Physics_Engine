# MPE Public API Reference — v15S

> Who this is for: **embedders** (code that owns a world in-process) and
> **module authors** (code loaded over the MPI C ABI). This document
> catalogs the public surface and the ownership / lifetime rules the header
> contracts enforce. The source of truth is the headers under `v16R1/src/`;
> signatures below are quoted from them. The interactive module lifecycle
> (`mod load|attach|detach|unload|...`) is covered in
> [`v16R1/MODULES.md`](../v16R1/MODULES.md); a complete worked example lives in
> [`docs/MODULE_TUTORIAL.md`](MODULE_TUTORIAL.md).

Header paths are relative to `v16R1/src/`. Layout notes:

| Layout element | File |
|---|---|
| Kernel: world, bodies, math, determinism, diagnostics, module ABI, loader, registry | `core/` |
| Physics stages: broadphase, narrowphase, solver, joints, CCD, depenetration, islands | `physics/` |
| Scene I/O (v200 format, CRC32, ID remap), boundary walls | `scene/` |
| Config system (80 tunables) | `config/` |
| Ecosystem members (hot-plug modules) | `ecosystem/` |

---

## 1. World lifecycle — `core/physics_world.h`

The kernel holds **zero global simulation state**. Every entry point takes an
explicit `physics_world *` (the type alias `mpe_world_t` is the opaque name
the module ABI passes; it is the same struct). (The application's "primary
world" lives in `core/mpe_primary.c`; headless code should declare its own.)

```c
void physics_world_init    (physics_world *world);
void physics_world_cleanup (physics_world *world);
void physics_world_set_config (physics_world *world, mpe_config_t *cfg); /* rebind per-world config */
void physics_world_clear   (physics_world *world);           /* remove all bodies */
void physics_world_step    (physics_world *world, float dt); /* the canonical advance */
```

Rules worth knowing before embedding:

- **Init / re-init**: `init` zeroes the struct, pins FP state
  (`det_pin_fp_state`), registers builtins, and adds the world to the
  process live list. If the address is *already live* (previously
  initialized without cleanup), init cleans the old state first —
  re-init without cleanup is otherwise caller discipline.
- **Failure degrades, never crashes**: the init allocation paths are
  fail-soft; a world that failed to grow is safely unusable (add-calls
  reject, solves skip) rather than UB.
- **Cleanup** frees the world's growable pools and detaches its tick
  modules; after it the `physics_world` value is dead (a stale pointer
  placed back into init-zeros is handled by the live-list address check).
- **A world does not share solver data with another world**; worlds may be
  stepped concurrently (see §10).

### Adding bodies

```c
int  physics_world_add_sphere  (physics_world *world, float radius, float mass, vector3 position);
int  physics_world_add_cube    (physics_world *world, vector3 position, vector3 half_extensions, float mass);
int  physics_world_add_cylinder (...); /* radius, half_length, mass, ... */
int  physics_world_add_custom  (physics_world *world, int custom_shape, vector3 position, float mass, float radius);
```

Return the body index; `-1` on error. Entry points reject NULL gracefully
(add calls become no-ops). The floor plane body is pre-reserved at
`y = 0` with `object_id 0xFFFFFFFF` and neutral configuration
(`static_plane_enabled = false` by default; regulation walls come from
`scene/boundary.c`).

### Housekeeping

```c
int  physics_world_grow_bodies (physics_world *world);          /* body pool x2 */
int  physics_world_grow_contact_cache (physics_world *world);
int  physics_world_grow_manifolds (physics_world *world);
int  physics_world_index_by_id (physics_world *world, uint32_t id); /* O(1) id->index cache hit, verified against live array */
void physics_world_bump_revision (physics_world *world);       /* forces scene / cache honest rebuild */
int  physics_world_live_list  (physics_world **out, int cap);  /* every world in the process */
```

## 2. Per-world config — `config/mpe_config.h`

Config is **snapshot at tick start** (a config change mid-step takes effect
next tick). The per-world binding is `world->cfg`; the global default store
is `g_cfg` (a world defaults to it until `set_config`).

```c
void mpe_config_init (void);
void mpe_config_reset_defaults (void);
bool mpe_config_get_float (const char *key, float *out);   /* Int / Bool variants */
bool mpe_config_set_float (const char *key, float value);  /* Int / Bool variants */
bool mpe_config_load (const char *path);
bool mpe_config_save (const char *path);
bool mpe_config_save_guarded (const char *path);
```

- 80 tunables in 13 categories; the authoritative list is
  [`v16R1/CONFIG.md`](../v16R1/CONFIG.md).
- Module authors: read tunables through `world->cfg` **by struct field**
  (e.g. `cfg->solver.penetration_slop`), never through the key-string API
  inside the tick (keep hot paths table-free), and treat `cfg` as read-only.

## 3. Rigid bodies — `core/rigidbody.h`

`rigidbody` is a packed POD: position, orientation (unit quaternion,
renormalized every tick), velocity, angular velocity, mass properties
(`inverse_mass`, local / system inverse inertia tensors), material pair
inputs (`restitution`, `friction_static`, `friction_kinetic`, optional
`friction_anisotropic` + `friction_anisotropy_axis` +
`friction_anisotropy_frame`), bounding properties (`radius`,
`cylinder_half_length`, `cached_axes`), identity (`object_type`,
`object_custom`, `object_id`, `object_generation`, `body_index`), and
lifecycle (`static_state`, sleep state).

- **`radius` is the bounding radius** — custom shapes must keep their true
  solid inside it or broadphase misses pairs (the capsule module derives
  its cross-section from `R² − h²` rather than trusting a separate field).
- **`object_id`/`object_generation`** identify bodies across array churn
  (removal compacts indices; ids survive).

## 4. Module ABI — `core/mpe_module.h`  (full contract in [`v16R1/MODULES.md`](../v16R1/MODULES.md))

Kinds: `"shape" | "broadphase" | "solver" | "forcefield" | "constraint" | "generic"`.

```c
int  (*attach)   (mpe_world_t *world, void **mod_state);   /* store per-world state, return 0 */
void (*detach)   (mpe_world_t *world, void *mod_state);    /* free it */
void (*pre_step) (mpe_world_t *world, float dt, void *mod_state);
void (*post_step)(mpe_world_t *world, float dt, void *mod_state);
void (*stage_detach) (mpe_world_t *world);                  /* append-only tail; null-check before use (old .so) */
```

- `mpe_module_desc_t.abi` must equal `MPE_MODULE_ABI` (1). The ABI
  transitively includes `rigidbody` / `broadphase_pair` / `collision_data`
  layouts (passed by pointer, **not frozen**): any engine change to those
  structs **must bump `MPE_MODULE_ABI`** and rebuild all plugins.
- Pair-handler typedef: `bool (*mpe_collide_fn)(rigidbody *a, rigidbody *b,
  void *manifold_out, mpe_world_t *world)` — with `MPE_THREADS > 1`
  handlers for different pairs run **concurrently**; be reentrant,
  read-only on world state, and write only the supplied manifold.
- Tick-hook tables are **snapshotted** per tick: a hook may detach itself
  mid-tick (not called again this tick); a hook attached mid-tick is
  deferred to the next tick. Never call through a snapshotted state
  pointer — re-look-up in the live table
  (`physics_world_module_live_state`) each time.

## 5. Registry — `core/mpe_registry.h`

Process-global type/plugin repository (intentionally *not* simulation
state). Builtins self-register at startup; `.so` plugins register through
the same functions via the loader.

```c
int  mpe_register_pair_handler (int type_a, int type_b, int custom_a, int custom_b, mpe_collide_fn fn, const char *name);
mpe_collide_fn mpe_find_pair_handler (int type_a, int type_b, int custom_a, int custom_b);
int  mpe_unregister_pair_handler (mpe_collide_fn fn);          /* self-unregister from a destructor */
int  mpe_register_broadphase (const char *name, const mpe_broadphase_if_t *iface);
int  mpe_register_solver    (const char *name, const mpe_solver_if_t *iface);
int  mpe_unregister_broadphase / _solver (const char *name);
const mpe_broadphase_if_t *mpe_find_broadphase (const char *name);
const mpe_solver_if_t     *mpe_find_solver     (const char *name);
int  mpe_register_module    (const mpe_module_desc_t *desc);
int  mpe_unregister_module  (const char *name);
void mpe_registry_lock / mpe_registry_unlock (void);
```

Wildcards & takeovers:

- `-1` is the wildcard for a pair dimension; `type == 3` means
  `object_custom` and then `custom_a/b` select the custom shape id
  (see `mpe_capsule.c`: `(3, 0, 100, -1, ...)` = custom-100 vs sphere).
- Stage registration refuses a **silent takeover of a builtin**:
  registering `"hash"` (broadphase), `"seq-impulse"` (solver), or the six
  builtin pair rows fails instead of shadowing them. Use your own name.
- Register results (`mpe_find_*`, indexed `broadphase_row` / `solver_row` /
  `pair_fn_at`) are **unowned interior pointers / views** — copy what you
  keep; never store across an unload.
- `mpe_registry_*_truncate_*` exist only for the **loader's** rollback path
  (they restore exactly a pre-`dlopen` snapshot); do not use them elsewhere.
- `registry_lock/unlock` brackets multi-step loader transactions; dispatch
  paths lock internally — **never** hold the lock across `physics_world_step`.

## 6. Loader — `core/mpe_loader.h`

```c
int  mpe_loader_load (const char *path, char *errbuf, int errlen);
int  mpe_loader_unload (const char *path_or_name);
void *mpe_loader_symbol (const char *path_or_name, const char *sym); /* RTLD_NOLOAD; never loads */
void mpe_loader_retain_module (const void *desc);
void mpe_loader_release_module (const void *desc);
```

- **Jail**: paths must resolve inside `ecosystem/` **relative to the
  process working directory** (normally `v16R1/src`). Anything else fails
  closed (`E_JAIL` / `E_NOFILE`).
- **Load codes**: `0` loaded (or already loaded, file unchanged); `-1` bad
  path/jail/dlopen/registry; `-3` **stale** — path already loaded but the
  file changed on disk, so the in-memory image is old code (unload, then
  load again). Stale code is never silently run.
- **Unload codes**: `0`; `-1` unknown; `-2` **busy** (a live world still
  references the image as an attached tick module, active stage backend, or
  in-range pair handler — detach, then retry); `-3` name does not fit the
  teardown buffer (never half-tear-down the registry).
- **Admin runs at a tick boundary** (quiesce via `physics_halt` or between
  steps); concurrent loader use from worker threads is misuse.
- `retain/release` is the attachment accounting that prevents `dlclose`
  while a world can still call a module callback; static descriptors
  compiled into the process are harmless no-ops. Stage slots (which keep
  only an interface pointer) are released via
  `mpe_loader_call_stage_detach[_for_fn]`, which validates the
  append-only `stage_detach` pointer (null-check + `dladdr` image check)
  before invoking it.

## 7. Joints — engine-managed (`physics/spring_joint.h`, `physics/revolute_joint.h`)

Joints are **not** replaceable pipeline plugins (the solver interface is;
joints are engine-managed). Spring pool:

```c
void joint_init_pool (physics_world *world);
int  add_joint (physics_world *world, int ia, int ib, float equilibrium_length, ...);
int  add_joint_by_ids (physics_world *world, uint32_t id_a, uint32_t id_b, ...);
void remove_joint / remove_joints_from_object[_id] (...);
void apply_force_all_joints_(world, ...) /* plain and _dt variants */
```

Constraint family (one engine-side solve each, driven by a `*_params`
struct): `revolute`, `prismatic`, `fixed`, `distance`, `rope` —
`*_solve`, `*_pre_step`, `*_apply_motor`, and drift-correction helpers all
take the world's `mpe_config_t`. The equations they implement are derived
step-by-step in [`v16R1/PHYSICS.md`](../v16R1/PHYSICS.md).

## 8. Deterministic math — `core/det_math.h`

Deterministic modules must use this header; **bare `sin/cos/exp/log/pow`
inside the tick make `deterministic = true` a lie.**

| Allowed inside the tick | Not allowed (libm varies ~1e-7..1e-16) |
|---|---|
| `+ - * / fmod sqrt` (`IEEE 754`-exact) | `sin cos exp log pow` |

Deterministic replacements (documented error bounds in the header):
`det_sin / det_cos` (single-double Cody-Waite reduction, `<5e-13` under
`|x| < 1e4`), `det_sin_small / det_cos_small` (`<1e-15` on
`|x| <= pi/4`), `det_ln_pos`, `det_exp_small`, `det_pow_retention`
(air damping). Out-of-contract input **falls back to libm and counts it**:
`det_fallback_pow_count` / `det_fallback_trig_count` are atomic,
release-active telemetry — a test can assert zero, so a desync is never
silent. `det_pin_fp_state` (called at world init) pins the portable FP
subset (rounding mode + x86/ARM denormal flushing). Bitwise-twin claims
are scoped to GCC/Clang IEEE targets; MSVC `/fp:strict` is unproven
(`v16R1/docs/NUMERICAL_GUARANTEES.md` has the precise budgets).

## 9. Diagnostics & validation — `core/mpe_diag.h`, `core/event_log.h`

```c
void event_log_init (void);
void event_log_push (log_level level, const char *format, ...);
int  event_log_get_count (void);
void event_log_clear (void);
void validation_report_print (void);
```

Every module rejection (loader / registry / attach) emits a diagnostics
record naming the subsystem, a stable code, the engine `file:line`, and
runtime values — rejections are never silent. Runtime self-audit:
`MPE_TESTS`-style validation scenes run from the F5–F11 keys report
through `validation_report`. Details and the S2 bridge transcript:
[`v16R1/docs/DIAGNOSTICS.md`](../v16R1/docs/DIAGNOSTICS.md).

## 10. Threading contract (summary) — full model in [`v16R1/MULTICORE.md`](../v16R1/MULTICORE.md)

Parallel under `MPE_THREADS > 1`: per-body work, CCD sweeps, pair-local
collision detection, narrowphase dispatch, solver sweeps **across
independent islands** (island-internal Gauss-Seidel order is preserved —
no cross-isle race exists because islands are connected components).
Still serial: shared-body contact preparation and joint solving.

Determinism twins (headline from `docs/VALIDATION.md`): the multithreaded
engine is **bitwise identical at `MPE_THREADS = 1/4/8`** on GCC/Clang IEEE
targets.

## 11. Ownership & lifetime cheat-sheet

| Object | Owner | Rule |
|---|---|---|
| Body pool, manifolds, contact cache, scratch | the `physics_world` | freed by `cleanup`; copies taken by callers |
| Registry strings (`name [64]`) | the registry | owned copies — never dangle into `.so` rodata |
| Pair/stage entries | the registry until *your* `unregister` | pair handlers self-unregister in a destructor |
| `mpe_module_desc_t` | your module's image | must stay mapped while attached (loader enforces via `-2` busy) |
| `mod_state` (`void*`) | your attach/detach | per-world; detach **must** free it |
| Stage state (`broadphase_state`/`solver_state`) | your module | declared via `stage_detach` — engine drops only the pointer |
| `mpe_find_*` results | temporary | unowned interior pointers; copy, never store across unload |
| dlopen handle | the loader | released on `unload == 0`; retained on `-2` busy |

---

*Corrections to this document follow the same discipline as the code in
_tree_: quote the header, and—if a signature changed—fix both in the same
commit. See [`v16R1/DEV_GUIDE.md`](../v16R1/DEV_GUIDE.md).*
