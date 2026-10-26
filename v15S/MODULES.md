# MPE Module System (MPI) — v15S

> Hot-plug supported physics components through a stable C ABI. Every
> modular artifact lives under `ecosystem/`; the loader jails paths there.

---

## 1. Overview

The supported extension points are the broadphase, solver interface,
narrowphase shape-pair handlers, and per-world pre/post-step modules. Force
fields can be applied by tick modules. Joints are engine-managed and are not
replaceable pipeline plugins. `.so` modules use the stable C ABI in
`core/mpe_module.h` (`MPE_MODULE_ABI 1`).

Modules are loaded live via the debug terminal (`mod load|unload|attach|
detach|use-broadphase|use-solver|ls`) or programmatically. Everything
modular lives under `v15S/src/ecosystem/<member>/…`; the old top-level
`plugins/` no longer exists. A complete worked example — module source,
build line, `mod load → attach → detach → unload`, and the pitfalls audit
history — lives in [`docs/MODULE_TUTORIAL.md`](../docs/MODULE_TUTORIAL.md).

---

## 2. The ABI

```c
#define MPE_MODULE_ABI 1

typedef struct {
    uint32_t abi;              /* must equal MPE_MODULE_ABI */
    const char *name;          /* e.g. "capsule-shape" */
    const char *version;       /* e.g. "1.0" */
    const char *kind;          /* "shape" \| "broadphase" \| "solver" \| "forcefield" \| "constraint" \| "generic" */
    bool deterministic;        /* true if bit-identical across IEEE targets */
    int  (*attach)  (mpe_world_t *world, void **mod_state);
    void (*detach)  (mpe_world_t *world, void *mod_state);
    void (*pre_step)(mpe_world_t *world, float dt, void *mod_state);
    void (*post_step)(mpe_world_t *world, float dt, void *mod_state);
    void (*stage_detach)(mpe_world_t *world);  /* append-only; ABI still 1 */
} mpe_module_desc_t;
```

ABI versioning: `abi` must equal `MPE_MODULE_ABI` (1). The ABI transitively
includes `rigidbody` / `broadphase_pair` / `collision_data` layouts, which
are passed by pointer and **NOT frozen** — any engine change to those structs
requires bumping `MPE_MODULE_ABI` and rebuilding all plugins.

`stage_detach` was added at the END of `mpe_module_desc_t`, so a `.so`
built against the older header keeps working — readers MUST null-check
`stage_detach` before calling (an old `.so` has no such pointer; the loader
further validates the address via `dladdr` before invoking). Any future
field reordering, removal, or semantic change requires bumping
`MPE_MODULE_ABI`.

---

## 3. Rules for Module Authors

1. **Modules never touch `g_cfg` / `g_physics_world` / `bodies[]` directly.**
   Use `mpe_world_*` accessors + per-world cfg (`mpe_world_cfg`).
2. **Per-module per-world state via `void*`** — attach stores, detach frees.
3. **Deterministic modules must use `det_math.h`**, never bare sin/cos/pow
   (`sqrtf`/`isfinite` are IEEE-exact and allowed).
4. **Pair handlers self-unregister in a destructor** (see
   `ecosystem/capsule/mpe_capsule.c`); the loader purges leftovers by code
   address.
5. **Registry find results are unowned interior pointers** — copy what you
   keep, never store them across unload.
6. **Unregistering a module/stage detaches it from every live world first.**
7. **Pair handlers may run concurrently.** When `MPE_THREADS > 1`, narrowphase
   dispatch can call a handler from worker threads for different pairs. A
   handler must be reentrant, read-only with respect to bodies/world state,
   and write only to its supplied manifold. Shared plugin state needs its own
   synchronization.

---

## 4. Lifecycle

| Operation | Behavior |
|---|---|
| `mod load <path>` | ABI-checked `dlopen`, path-jailed to `ecosystem/`; rejects escapes (`E_JAIL`) and missing files (`E_NOFILE`) |
| `mod attach <name>` | Returns distinct negative code per cause (`MPE_ATTACH_E_*`); all `< 0` |
| `mod detach <name>` | Detaches from all live worlds, releases loader handle |
| `mod unload` | Refuses `-2` (busy) while any live world references the code |
| `mod use-broadphase\|use-solver <name>` | Overrides the corresponding stage on the primary world |

Attach rejections carry **distinct negative codes** (all `< 0`, so existing
`>= 0` callers are unchanged):

| Code | Meaning |
|---|---|
| `MPE_ATTACH_E_NULL` (-1) | null world or descriptor |
| `MPE_ATTACH_E_ABI` (-2) | ABI mismatch |
| `MPE_ATTACH_E_COUNT` (-3) | tick-module count corrupt |
| `MPE_ATTACH_E_TABLE` (-4) | table full (16 slots) |
| `MPE_ATTACH_E_HOOK` (-5) | module's attach hook refused |
| `MPE_ATTACH_E_NAME` (-6) | module name missing |

Loader rejections: `E_NOFILE`, `E_JAIL`, `E_STALE`, `E_TABLE`, `E_DLOPEN`,
`E_ABI`, `E_NODESC`, `E_BUSY`.

Every rejection emits a diagnostics record naming source, stable code,
engine `file:line`, and runtime values (`core/mpe_diag.h`).

---

## 5. Loader Jail

`mpe_loader_load()` accepts any path that resolves inside `ecosystem/` and
rejects:

- paths that escape `ecosystem/` → `E_JAIL`
- missing files → `E_NOFILE`
- stale images (on-disk mtime/size changed since load) → `E_STALE` (changed
  files now fail with -3 instead of silently running the old in-memory image)

Windows accepts `/` and `\`, both extensions, case-insensitively; Linux
keeps exact `.so` behaviour.

---

## 6. Symbol Interposition Warning

Several plugins export a global named `mpe_module_desc`. A library that
locates *itself* with `dladdr(&mpe_module_desc)` can be handed another
plugin's address once more than one is loaded. Use a `static` symbol as the
`dladdr` anchor instead — a local binding cannot be interposed.

---

## 7. Tick Hooks

Tick hook tables are **snapshotted**, so hooks may attach/detach mid-tick.
A hook that detaches itself mid-tick is never invoked again this tick; a
hook attached mid-tick is deferred to the next tick. Never call through a
snapshotted state pointer — look the descriptor up in the LIVE table each
time and skip it if it is no longer attached (`physics_world_module_live_state`).

---

## 8. Ecosystem Members

| Member | Purpose | Registry name | Build artifact |
|---|---|---|---|
| `capsule` | True segment capsule shape (`R = √(h²+rc²)`) | capsule-shape | `ecosystem/capsule/mpe_capsule.so` |
| `mfs` | FTC robotics fleet (mecanum, drivetrain, observer, BioBuzz) | mfs-simulator | `ecosystem/mfs/mfs_ecosystem.so` |
| `mgb` | S2 molecular bridge (display-only, S2→MPE) | s2-bridge | `ecosystem/mgb/build/mgb_bridge.so` |

Ecosystem bundles (`mpe_ecosystem_desc`) load from `ecosystem/mfs/*.so`;
a bundle links its inner modules' objects and therefore exports their
descriptors too, taking precedence over plain `.so` modules.

---

## 9. Example

```bash
cd v15S/src
./engine
# in the debug terminal:
mod load ecosystem/capsule/mpe_capsule.so
mod attach capsule-shape
mod load ecosystem/mgb/build/mgb_bridge.so
mod attach s2-bridge
diag            # anything that failed is named here
```

```c
/* C API */
mpe_register_pair_handler(3, 0, 100, -1, capsule_vs_sphere, "capsule-sphere");
physics_world_set_solver(world, mpe_find_solver("seq-impulse"));
physics_world_attach_module(world, desc);
```
