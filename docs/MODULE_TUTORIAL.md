# Your First MPE Module — a Worked Tutorial

> Build, hot-load, attach, verify, and unload a complete MPI module in about
> fifteen minutes. The contract is in [`v16R1/MODULES.md`](../v16R1/MODULES.md);
> the public API surface is in [`docs/API_REFERENCE.md`](API_REFERENCE.md);
> the reference implementation this tutorial mirrors is
> `v16R1/src/ecosystem/capsule/mpe_capsule.c` (a *shape* module — the hardest
> kind). Here we build the *easiest* useful kind: a forcefield tick module.

## 0. What you will build

`ecosystem/flow/` — a module named **`flow-wind`** that pushes awake dynamic
spheres in a fixed direction, once per tick, before the solver runs. It
demonstrates the whole lifecycle, including per-world state.

Fifteen-minute path: create the directory → paste the module → build →
`mod load` → `mod attach` → watch the physics → `mod detach` → `mod unload`.

## 1. The three contract rules that matter here

Every rule lives in `v16R1/src/core/mpe_module.h`; these three bite first:

1. **Per-world state via `void *mod_state`** — `attach` mallocs it, `detach`
   frees it. Never allocate anywhere else, never share across worlds.
2. **Read the world, don't fight it** — use `world->cfg` (per-world config,
   read-only in a tick hook), never the global `g_cfg`.
3. **`deterministic = true` is a promise** — only if you use IEEE-exact ops
   (`+ - * / fmod sqrt`) or `det_math.h` inside the tick. This module uses
   only exact ops, so the flag is honest.

## 2. The module source

```c
/* ecosystem/flow/mpe_flow.c — MPI forcefield: constant wind on spheres.
 *
 * Attached per world; each tick (pre_step) it adds an impulse
 * GRADIENT * dt to every awake dynamic sphere. Extraction uses only the
 * fields promised by the loader ABI view. NOTE rigidbody layout is NOT
 * frozen: re-bump MPE_MODULE_ABI at any engine change to those structs. */
#include "core/mpe_module.h"
#include "core/mpe_platform.h"
#include <math.h>
#include <stdlib.h>

/* Wind, world axes (m/s²). Exact ops only: products are IEEE-exact. */
#define FLOW_AX   0.0f
#define FLOW_AY   0.0f
#define FLOW_AZ   2.0f

typedef struct { unsigned long ticks; } flow_state_t;

static void flow_tick (mpe_world_t *world, float dt, void *mod_state) {
    if (!world || !world -> bodies || !(dt > 0.0f)) { return; }
    for (int i = 0; i < world -> body_count; i++) {
        rigidbody *b = &world -> bodies [i];
        if (b -> object_type != object_sphere) { continue; }      /* spheres only */
        if (b -> static_state)                 { continue; }      /* statics immovable */
        if (b -> is_sleeping)                  { continue; }      /* sleepers stay asleep */
        b -> velocity.x += FLOW_AX * dt;
        b -> velocity.y += FLOW_AY * dt;
        b -> velocity.z += FLOW_AZ * dt;                          /* exact, no trig */
    }
    if (mod_state) { ((flow_state_t *) mod_state) -> ticks++; }
}

static int flow_attach (mpe_world_t *world, void **mod_state) {
    (void) world;
    flow_state_t *st = (flow_state_t *) malloc (sizeof (flow_state_t));
    if (!st) { return -1; }         /* non-zero tells the caller "refused" */
    st -> ticks = 0;
    *mod_state = st;
    return 0;
}

static void flow_detach (mpe_world_t *world, void *mod_state) {
    (void) world;
    free (mod_state);               /* frees OUR state only */
}

/* No MPE_CTOR needed: tick modules need no registry rows. The loader finds
 * the EXPORTED mpe_module_desc symbol in the image. */
const mpe_module_desc_t mpe_module_desc = {
    .abi = MPE_MODULE_ABI,          /* must match the engine's header exactly */
    .name = "flow-wind",
    .version = "1.0",
    .kind = "forcefield",
    .deterministic = true,          /* honest: exact ops on all bodies */
    .attach = flow_attach,
    .detach = flow_detach,
    .pre_step = flow_tick,
    .post_step = 0,
    .stage_detach = 0,              /* no stage state (broadphase/solver) stow */
};
```

Notes on what is deliberately *not* in this example:

- **No pair handlers** — those need a destructor that self-unregisters
  (`mpe_unregister_pair_handler`) so `dlclose` never leaves dangling
  function pointers; see `mpe_capsule.c` lines `MPE_CTOR/MPE_DTOR`.
- **No config reads** — for a real forcefield, read per-world tunables via
  `world->cfg` at tick start and validate non-finite/config-went-crazy
  values (the module must not crash a world that has a hostile config;
  see `capsule_slop()` in `mpe_capsule.c` for the defensive pattern).

## 3. Build

The loader jail accepts anything resolving inside `ecosystem/<member>/`.
Create the directory and build exactly as the engine builds the capsule module
(rule at `v16R1/src/makefile`, `ecosystem/capsule/mpe_capsule$(PLUGIN_EXT)`):

```bash
cd v16R1/src
mkdir -p ecosystem/flow
gcc -g -I. -fPIC -shared -Wall -Wextra -Werror -ffp-contract=off \
    ecosystem/flow/mpe_flow.c -o ecosystem/flow/mpe_flow.so
```

Or make it a first-class makefile rule (put it next to the capsule one):

```make
ecosystem/flow/mpe_flow$(PLUGIN_EXT): ecosystem/flow/mpe_flow.c core/mpe_module.h core/mpe_platform.h
	$(CC) $(BASE_CFLAGS) -I. -fPIC -shared ecosystem/flow/mpe_flow.c -o ecosystem/flow/mpe_flow$(PLUGIN_EXT)
```

`-ffp-contract=off` is not optional in spirit: it is the flag that keeps
your arithmetic bit-stable (the engine pins it on every compile; see
`make check-flags`).

## 4. Load → attach → verify → detach → unload

Run the engine from `v16R1/src` (the jail is relative to the **process CWD**)
and open the debug page: **`1`** key. Then:

```text
mod load ecosystem/flow/mpe_flow.so
mod attach flow-wind
mod ls          # your pair/stage/module rows must appear
diag            # anything that failed is named here: file:line + stable code
```

Behavior to check after `attach`: set up two spheres (debug camera, `0` toggles
Game/Debug) beside some cubes; only the spheres should drift at ~2 m/s² along
+Z — cubes untouched, statics untouched.

Unload cycle (everything admin is a tick-boundary operation):

```text
mod detach flow-wind    # per-world state freed
mod unload ecosystem/flow/mpe_flow.so
```

Expect `unload` to return **`E_BUSY` (-2)** if anything still references the
image (attach counts, stage activations, pair rows in range). That is the
loader protecting you from a `dlclose` under a live callback; detach first,
then retry. Re-`load` with the file changed on disk returns **`-3 stale`** —
the engine refuses to silently run the old in-memory image; unload and load
again.

## 5. The mistakes that cost the maintainers real time

(the pitfalls each cost an audit entry somewhere in `v16R1/docs/DESPOT_*.md`)

1. **Duplicate shared-symbol interposition.** Several plugins export a
   global named `mpe_module_desc`. If your `.so` locates *itself* with
   `dladdr(&mpe_module_desc)`, once more than one plugin is loaded you can
   be handed another plugin's address (interposition). Anchor `dladdr` on a
   `static` symbol instead — a local binding cannot be interposed.
2. **Registry find results are views, not ownership.** `mpe_find_*` returns
   an interior pointer into the registry table. Copy what you need; never
   hold it across an unload.
3. **`static_state` vs sleep.** Statics never move; *sleeping dynamics*
   wake on their own when forces act — check `sleep_contact_wake` semantics
   in `v16R1/src/tests/` before assuming wind should wake them (this tutorial
   intentionally leaves sleeping bodies asleep, which is what world-honest
   physics wants).
4. **`deterministic = true` without `det_math`.** One bare `powf` in your
   tick is enough to desync every long run across every machine — the
   engine counts libm fallbacks (atomic counters, `"det_fallback_*"`), and
   tests attesting zero fallbacks will catch you.
5. **Paying the ABI-mismatch toll.** Engine changes to `rigidbody`,
   `broadphase_pair`, or `collision_data` bump `MPE_MODULE_ABI`; a `.so`
   built against the old header bails at `attach` with `E_ABI` (-2). That
   is the system working; rebuild.
6. **Holding the registry lock across `step`.** `mpe_registry_lock` brackets
   loader transactions. `step` dispatches through the registry — the lock
   must be released before any world steps.

## 6. Where to go next

| Goal | Read / copy |
|---|---|
| Write a new **shape** (pair handlers, bounding invariant, slop, contacts) | `ecosystem/capsule/mpe_capsule.c` — the reference module |
| Replace the **broadphase / solver** (hooks) | `mpe_broadphase_if_t` / `mpe_solver_if_t` in `core/mpe_module.h`; indexing rules in `v16R1/ARCHITECTURE.md` §data structures |
| Build a **bundle** of several modules | `ecosystem/mfs/mfs_ecosystem.c` (`mpe_ecosystem_desc`) and `v16R1/src/ecosystem/mfs/readme.md` |
| Understand the **biggest module in the repo** (robotics fleet) | `v16R1/FTC.md` + `v16R1/src/ecosystem/mfs/docs/ARCHITECTURE.md` |
| Understand why your module must respect tick boundaries | `core/mpe_loader.h` header contract |

*Keep the module contract in view: `core/mpe_module.h` is short and is the
easiest 70 lines in the tree to read. Enjoy.*
