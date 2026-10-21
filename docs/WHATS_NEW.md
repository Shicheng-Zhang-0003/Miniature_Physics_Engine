# What's New in v15S

`v15S` is the GTK4 + modular-kernel evolution of `v15R3` (whose release record
is preserved in [`release_notes_v15R3.md`](../release_notes_v15R3.md)). Highlights
over `v15R3`:

- **GTK4 port** — event controllers, gestures, overlay, dialogs; Wayland-safe
  input (no more X11-only mouse lock). GTK4 is the only supported toolkit;
  the GTK3 body was removed in 2026-09.
- **Module system (MPI)** — stable C ABI (`core/mpe_module.h`):
  register shape-pair handlers, broadphase, solver stages, and per-world tick
  modules; load `.so` plugins live (`mod load`), attach/detach per world.
- **Per-world configuration** — every physics function takes the owning
  world's config (`NULL` = global default); two worlds can run different
  gravity/iterations/slop side by side.
- **2026-10-20 second pass** — the audited-post-fix round (see the
  "Second-pass fixes" section of [`AUDIT_REPORT_2026-10-20.md`](AUDIT_REPORT_2026-10-20.md))
  covers A/B-matched GUI-vs-canonical step stages, a loader purge backstop
  for plugin stage ifaces, honest MGB transcript lineage, the makefile
  command-line-override trap, and many small UI/loader edges.
- **Upgraded data structures** — growable body/contact pools
  (512→16384, 4K→64K ceilings), O(1) contact-pair hash probes,
  per-world id→index cache (islands O(J+B)), small-first broadphase node
  pool, process-wide determinism counters.
- **No kernel globals** — the last old-series global (`g_physics_world`)
  moved to an app-ownership TU (`core/mpe_primary.c`); the kernel holds zero
  simulation state. The type/plugin registry stays process-global by design
  (documented in `core/mpe_registry.h`).
- **TUI stress suite** — new `stress` (300 bodies + every joint type) and
  `ccd` (60/144/300 m/s battery) scenes, `--broadphase/--solver` backend
  flags, per-scene configs, pool visibility in dumps.
- **Self-reported failures** — `core/mpe_diag.h` names every error with a
  subsystem, a stable code, the engine `file:line`, and the values that
  caused it; records land on stderr, the event log and (live) the debug
  terminal, and `diag` replays them. Replaces error paths that collapsed
  into a bare `-1` and a guessed message. Full reference:
  [`v15S/docs/DIAGNOSTICS.md`](../v15S/docs/DIAGNOSTICS.md).
- **Everything modular lives under `ecosystem/`** — the former top-level
  `plugins/` is gone. The loader jails paths to `ecosystem/` (any nesting
  depth), and a plain `.so` must export `mpe_module_desc`.
- **S2 molecular bridge** (`ecosystem/mgb`, registry name `s2-bridge`) —
  spawns a live `s2tui`, drives its existing shell over three pipes, reads
  strict `S2SAVE1` frames from `sync`, and stages them as MPE bodies and
  zero-force bond lines. Display-only and one-way: positions flow S2 → MPE,
  never back. `mod load ecosystem/mgb/build/mgb_bridge.so` then `mod attach
  s2-bridge`. **Phase 0 stages a single snapshot — the dish renders but does
  not drift under gravity or contact response** (kinematic bodies, `k=0`
  bonds, stage-once cadence);
  **Phase 1 moves existing bodies in place each refresh tick** (`cfg->every >
  0`), repositioning staged bodies rather than re-spawning them, with stable
  S2-atom→MPE-body mapping and zero-force bond lines that follow. See
  [`v15S/docs/DIAGNOSTICS.md`](../v15S/docs/DIAGNOSTICS.md) §4 and
  `v15S/REMAINING_WORK.md`.
- **Multicore solver** (`core/physics_world.c`) — `MPE_THREADS` (1–8,
  default 1) parallelizes per-body work, two-phase CCD, pair-local collision
  detection, and solver sweeps across independent islands. Shared-body contact
  preparation and joint solving stay serial; Gauss-Seidel order within each
  island is preserved. Determinism twins pass at 1/4/8 threads.
- **Warm start** (`core/physics/collision_solver.c`) — damped normal
  warm-start (`solver.warm_start_damping`, default 0.0). Nonzero damping
  scales the cached normal impulse and uses it as the Coulomb cone's `fn`,
  restoring cross-tick information carry. Cold (0.0) is the shipped default,
  stable across all five regimes (default/light/heavy/brittle/sticky);
  damped warm-start is experimental.
- **CCD rotational** (`core/physics/collision_ccd.c`) — floor plane CCD now
  uses the lowest point's velocity (`v_center + w × r_lowest`), so tumbling
  bodies' dipping corners clamp before tunneling. Pure spinners about Y are
  unaffected.
- **-Werror by default** — `make WARNINGS_AS_ERRORS=1` is now the default.
  All engine sources compile warning-free under `-Wall -Wextra -Werror`.

Inherited from `v15R3`:

- **Domain-driven architecture** — clean `core`, `physics`, `render`,
  `scene`, `ui_input` modules.
- **Multi-point Sutherland–Hodgman contact manifolds** with support-first
  ordering for stable stacking.
- **Full constraint framework** — revolute, fixed, prismatic, distance, and
  rope constraints plus spring joints (scene v200 persists springs + all
  five constraint types; all types live in the headless suite and the TUI
  demo).
- **3D spatial-hash grid broadphase** with adaptive cell sizing.
- **Interactive spring-joint system** with live magenta rendering.
- **POSIX-style debug terminal** — drive the whole simulation from a shell
  (now with a `mod` command for hot-plugging physics).
- **Built-in validation suite** (F5–F11), including a 60-second long-run
  stability test and config torture test.
- **Shader/render failure visibility** — the engine no longer continues
  silently in a broken render state.
- **Physics-truth pass** — Verlet-exact free flight, post-integration Poisson
  gate, CCD remainder integration, strict match-gated contact caching, true
  cylinder SDF geometry, no velocity clamps or restitution caps; game-only
  damping (`nice_value`, angular scale) labeled and defaulted off/vacuum.
- **Sleep truth** — three-gate wake (first-touch pair novelty, fast-other,
  deep overlap): slow pushers wake sleepers at any speed, resting stacks
  proceed to sleep and stay settled (F10 root cause, fixed).
- **Terminal debugger + output suite** — `mpe-tui`: live ncurses inspector
  (bodies, joints, constraints, math, scene graph) plus pipeable
  `--snapshot`/`--stream` state dumps.
