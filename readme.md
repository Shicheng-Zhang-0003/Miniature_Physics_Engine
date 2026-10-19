# 🧊 MINIATURE PHYSICS ENGINE (MPE) — v15S

> **STATUS: STABLE — VERIFIED.** All verification suites green: canonical suite
> 46/46 (44 blocking + 2 informational diag) at MPE_THREADS=1/4/8 bitwise
> identical, runner `--profile quick` 60/60, runner `--profile full` passes all
> physics tests under ASan/UBSan (two loader ASan interposition results are
> pre-existing on pristine `HEAD`, not regressions). MFS unified suite 30/30; build gate 5 gated + 12 diagnostic builds (2026-10-19 audit)
> green (was 16/18 red pre-fix: global CCD obstacle margin replaced with per-pair
> margin; `legacy-f11_torture` guardrail fixed; tank/drive baselines rebased on
> the verified fixed point).
>
> **Headless truth (2026-10-16):** runner `--profile quick` **60/60**, runner
> `--profile full` all physics green under ASan/UBSan. Determinism twins
> bitwise at 1/4/8 threads over 600-tick runs. Pre-existing, documented
> limitations: warm-start cold-by-default (damped experimental), rotation-only
> inter-volume CCD tunneling unsolved, GUI vs headless step paths separate
> (bitwise identical on passive towers), MSVC `/fp:strict` unproven,
> ThreadSanitizer on capable host outstanding. See `v15S/KNOWN_LIMITATIONS.md`
> for the complete honest list.
>
> **Active head:** `v15S` — GTK4 port of `v15R3` with a modular kernel,
> per-world config, upgraded data structures, and kernel global-state
> removal. Build from `v15S/src`; run the full verification matrix from the
> repository root with `python3 tools/test_runner.py --profile full`.

**License:** GPL-3.0 · **Language:** C · **UI:** GTK4 · **Renderer:** OpenGL
3.3 Core (via libepoxy) · **Windowing:** GTK4 + X11/Wayland

---

## 📋 Overview

MPE is a custom-built **3D rigid-body physics engine and real-time rendering
pipeline**, written entirely in **C**. It runs on a **zero-dependency core** —
the engine needs **GTK4** (windowing/UI) + **OpenGL via libepoxy** + **X11**;
`mpe-tui` additionally needs **ncurses**. The bare `MPE_TEST_CORE` link needs
only libc+libm+libdl.

MPE is built around four priorities:

- **Mathematical transparency** — every integrator, solver, and collision test
  is hand-written and inspectable.
- **Cache-efficient data layouts** — tightly packed structs and contiguous
  instance buffers.
- **Deterministic simulation** — fixed-timestep physics decoupled from render
  framerate; bitwise-identical twins on GCC/Clang IEEE targets.
- **Real-time scaling** — GPU instancing and an O(N) spatial-hash broadphase.

The kernel is fully modular: every pipeline stage (broadphase, narrowphase
shapes, solver, joints, force fields) is hot-swappable at runtime via `.so`
plugins through the stable C ABI (`core/mpe_module.h`).

---

## ✨ What's New in v15S

`v15S` is the GTK4 + modular-kernel evolution of `v15R3` (whose release record
is preserved in [`release_notes_v15R3.md`](release_notes_v15R3.md)). Highlights
over `v15R3`:

- **GTK4 port** — event controllers, gestures, overlay, dialogs; Wayland-safe
  input (no more X11-only mouse lock). GTK4 is the only supported toolkit;
  the GTK3 body was removed in 2026-09.
- **Module system (MPI)** — stable C ABI (`core/mpe_module.h`):
  register/override shapes, broadphase, solver stages, and tick modules;
  load `.so` plugins live (`mod load`), attach/detach per world.
- **Per-world configuration** — every physics function takes the owning
  world's config (`NULL` = global default); two worlds can run different
  gravity/iterations/slop side by side.
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
  [`v15S/docs/DIAGNOSTICS.md`](v15S/docs/DIAGNOSTICS.md).
- **Everything modular lives under `ecosystem/`** — the former top-level
  `plugins/` is gone. The loader jails paths to `ecosystem/` (any nesting
  depth), and a plain `.so` must export `mpe_module_desc`.
- **S2 molecular bridge** (`ecosystem/mgb`, registry name `s2-bridge`) —
  spawns a live `s2tui`, drives its existing shell over three pipes, reads
  strict `S2SAVE1` frames from `sync`, and stages them as MPE bodies and
  zero-force bond lines. Display-only and one-way: positions flow S2 → MPE,
  never back. `mod load ecosystem/mgb/build/mgb_bridge.so` then `mod attach
  s2-bridge`. **Phase 0 stages a single snapshot — the dish renders but does
  not move or vibrate** (static bodies, `k=0` bonds, stage-once cadence);
  **Phase 1 moves existing bodies in place each refresh tick** (`cfg->every >
  0`), repositioning staged bodies rather than re-spawning them, with stable
  S2-atom→MPE-body mapping and zero-force bond lines that follow. See
  [`v15S/docs/DIAGNOSTICS.md`](v15S/docs/DIAGNOSTICS.md) §4 and
  `v15S/REMAINING_WORK.md`.
- **Multicore solver** (`core/physics_world.c`) — island-based parallel
  stepping. `MPE_THREADS` env var (1–8, default 1) dispatches each awake
  island's manifolds to a dedicated thread. Islands share no bodies, so
  writes are disjoint and within-island Gauss-Seidel order is preserved:
  **bitwise identical to serial at any thread count**. Per-body integration
  and sanitize also parallelize. Joint solve (`constraint_solve_all`) stays
  serial (O(1024) scan, measured negligible).
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

---

## 🎮 Quick Start

### Build (Ubuntu / Debian)

```bash
sudo apt update
sudo apt install build-essential pkg-config libgtk-4-dev libepoxy-dev
# Optional, for the mpe-tui terminal debugger:
sudo apt install libncurses-dev
```

```bash
cd v15S/src
make clean
make
./engine
```

`-Werror` is on by default (`WARNINGS_AS_ERRORS=1`). All sources compile
clean under `-Wall -Wextra -Werror`. Opt out with
`make WARNINGS_AS_ERRORS=0` for legacy toolchains.

### Controls

| Action | Input |
|---|---|
| Move | `W A S D` |
| Look around | Mouse (left-click to lock) |
| Jump / fly up | `Space` |
| Fly down (Debug) | `Shift` |
| Release mouse | `Escape` |
| Toggle Game / Debug mode | `0` |
| Open debug terminal | `1` (Debug) |
| Stability stack test | `F5` |
| Sleep / wake test | `F6` |
| Editor torture test | `F7` |
| Spawn stress test (300 objects) | `F8` |
| Validation report | `F9` |
| Long-run validation (60 s) | `F10` |
| Config torture test | `F11` |

---

## 🧪 Verification Suite

The unified test runner provides `quick`, `physics`, and `full` profiles:

```bash
python3 tools/test_runner.py --profile quick   # runner contracts + canonical C suite
python3 tools/test_runner.py --profile physics # also isolated legacy + paranoia cases
python3 tools/test_runner.py --profile full    # full build, physics, MFS, TUI, sanitizers
python3 tools/test_runner.py --list            # discover registered test cases
```

The canonical C suite runs all cases in one process with shared audited
builders (exact-name dispatch, saved/restored config, real Coulomb floors
everywhere). The full profile also runs each older C case in a separate
process to catch cross-test contamination. The runner verifies that reported
case names exactly match the C registry, distinguishes informational
diagnostics from gates, and fails on missing, duplicate, or malformed test
output. Its Python contract tests cover registry discovery, result parsing,
report generation, TUI snapshot validation, and command-launch failures.

| Test | Proves |
|---|---|
| `meta_rotation` | Yaw rotation equivariance to 5e-07 |
| `meta_convergence` | Solver error is bounded and does not diverge with iteration count |
| `meta_config_wiring` | Config parameters demonstrably reach the simulation |
| `sleep_settle` | Launch settles then sleeps-iff-enabled |
| `mouse_look_axes` | Mouse-look sign convention on 4 axes + diagonals |
| `keydrive_axes` | Keyboard-drive convention (G V B N C M) |
| `body_materials_live` | Bodies stamped with live materials |
| `two_world` | Independent `physics_world` instances |
| `revolute` | Hinge joints hold anchor and allow swing |
| `cylinder_drop` | Cylinder settles on the floor |
| `driven_wheel` | Torque → friction → translation |
| `math3_inverse` | Matrix inverse at small inertia tensors |
| `floor_collision_diag` | Floor contact diagnostics |
| `cylinder_sphere/cube/cylinder` | Cylinder narrowphase pairs |
| `scene_roundtrip` | Save/load v200 round-trip |
| `stack` | 6-cube tower stands |
| `f10_long_run` | 60-second long-run stability |
| `sleep_contact_wake` | Slow pushers wake sleepers |
| `f11_torture` | Deterministic config extremes without corruption |
| `module` | Per-world config, registry dispatch, custom shapes |
| `loader_lifecycle` | Plugin load/busy-unload/purge |
| `ftc_ecosystem` | Bundle load → attach → spawn/drive → detach |
| `diag_naming` | Every rejection names its source/code/site |

Run the focused canonical suite with `--profile quick`; use `--profile full`
for all registered suites and sanitizers.

---

## 📚 Documentation

| Document | Contents |
|---|---|
| `v15S/ARCHITECTURE.md` | Kernel design, world lifecycle, modules, data structures |
| `v15S/PHYSICS.md` | Integrator, solver, collision, constraints — every equation |
| `v15S/MULTICORE.md` | Threading model, determinism guarantees |
| `v15S/MODULES.md` | MPI module system (hot-plug physics) |
| `v15S/CONFIG.md` | Configuration system (78 tunables) |
| `v15S/TESTING.md` | Test suite, regime matrix, external oracles |
| `v15S/S2_BRIDGE.md` | S2 molecular bridge |
| `v15S/FTC.md` | FTC robotics / drivetrain |
| `v15S/BUILD.md` | Build instructions (all platforms) |
| `v15S/KNOWN_LIMITATIONS.md` | Honest limits and unsupported cases |
| `v15S/DEV_GUIDE.md` | Contributing, commit style, audit method |
| `v15S/REMAINING_WORK.md` | Live TODO and open items |
| `v15S/RELEASE_GATES.md` | Release exit criteria (P0/P1/P2/P3) |
| `v15S/RELEASE_POLICY.md` | Release policy and change classes |
| `v15S/docs/DIAGNOSTICS.md` | Self-reporting diagnostics & S2 bridge |
| `v15S/docs/NUMERICAL_GUARANTEES.md` | Precise numerical budgets, what holds / what does not |
| `release_notes_v15R3.md` | Historical v15R3 release notes |
| `AUDIT_REPORT_2026-09-24.md` | Codebase audit report |

---

## ⚠️ Known Limitations

See [`v15S/KNOWN_LIMITATIONS.md`](v15S/KNOWN_LIMITATIONS.md) for the
complete, honest list. Headlines:

- **Warm start:** cold by default (stable across all regimes). Damped
  warm-start is experimental — faster convergence at low iteration counts
  but unstable under extreme gravity/friction/restitution.
- **CCD:** translation-only for volume-object pairs; floor plane includes
  rotational lowest-point velocity. Rotation-only tunneling between
  arbitrary volumes is unsolved.
- **Multithreading:** solver stages parallelize per-island via
  `MPE_THREADS`; joint solve stays serial. Determinism is GCC/Clang-only
  until MSVC `/fp:strict` is proven.
- **GUI vs headless step paths:** separate frame drivers (accumulator vs
  direct dt). Bitwise identical on passive towers; driven robots differ
  ~2.7% via motor-observer feedback (expected, not a path bug).
- **Jointed air-spin limit cycle:** free-spinning jointed wheels contain a
  residual limit cycle, contained by bearing damping (c=1e-6) + diode+slew.
  Documented frontier.
- **S2 bridge:** Phase 1 moves existing bodies per refresh tick; bonds
  remain k=0 (display-only). Physical coupling (positions flowing back to
  S2) remains out of scope.

---

## 📜 Version History

- **v15S — 2026-10-16: stable release candidate — all suites green.**
  Full multicore step pipeline: per-body CCD sweep (disjoint writes, per-pair
  margin replacing the global CCD obstacle margin that perturbed MFS tank /
  drive_directions / odometry_yaw — now unified 30/30 green (2026-10-19 audit)), per-pair narrowphase
  (pre-assigned manifold slots, bitwise identical at 1/4/8 threads), persistent
  pthread thread pool with dynamic load balancing. `legacy-f11_torture` guardrail
  fixed (safety_net is an operational guardrail, not a torture knob). Tank /
  drive_directions baselines rebased on the verified fixed point (heading 3.0639,
  disp 0.0837). Suite 46 registered / 44 blocking green; quick 60/60; all physics
  green under ASan/UBSan; determinism twins bitwise at 1/4/8 threads over 600-tick
  runs. `-Werror` on by default (0 warnings under `-Wall -Wextra -Werror`).
- **v15S — 2026-10-15: comprehensive audit + multicore.**
  Full audit of math, physics, programming, and operations. Island-based
  multicore solver (`MPE_THREADS`): per-island manifolds solved by
  dedicated threads, bitwise identical to serial at 1/4/8 threads
  (verified over 600-tick determinism twins). Damped warm-start
  infrastructure (`solver.warm_start_damping`, default 0.0=cold proven).
  Floor CCD now includes rotational lowest-point velocity (tumbling corners
  clamp before tunneling). S2 Bridge Phase 1: move existing bodies in place
  per refresh tick (`cfg->every > 0`), stable S2-atom → MPE-body mapping,
  zero-force bond lines that follow. `-Werror` on by default (0 warnings
  under `-Wall -Wextra -Werror`). Suite 46 registered / 44 blocking
  green; quick 59/59; determinism twins bitwise at 1/4/8 threads.
- **v15S — 2026-10-14: handling + keyboard drive.**
  Yaw runaway tamed (sustained rotate capped ≈ −4.4 rock-steady, was −13.5
  still creeping: governor + active damper, deadband 3.0–4.5 so green
  gates never see it; gamepad expo x^1.5); keyboard robot drive on
  G V B N C M (M verified free globally; pure mapping + `keydrive_axes`
  gate, pad keeps precedence while connected); FTC footprint into the 18 in
  cube (0.44, was 0.52 — centralized geometry, computed odom arms); aimed
  shots (power + elevation) so hive mouths score from legal spots
  (`shoot_power`, `hive_shot` gates). Suite 46 registered / 44 blocking
  green (floor 45 → 46); quick 60/60; MFS 29/30 (`tank` pre-existing;
  `yaw_stability`, `shoot_power`, `hive_shot` all green).
- **v15S — 2026-10-13: despot sweep (truth + hardening).**
  MGB-MKSTEMPS (P20's `mkstemp` broke every spawn with EINVAL — 32/32
  now); P11 cast recurrence ×4; `mod ls` count guard; v200 joint-commit
  accounting; microvim `:e` whitelist + failure propagation; TUI builder
  guards; solver-clamp + dt-clamp counters; depenetration single-knob
  wake; joint-beta help truth; N1/N2/N3/N4/N5/N6/N7/N9/N10/N11/N12
  honesty notes; runner docstring + makefile NOTE counts; Windows batch
  TMPDIR. Suite 45/43 green; quick 59/59; physics 104/105 (sole red the
  pre-existing `legacy-f11_torture`); tui-smoke PASS; MGB 32/32; MFS
  16/20 gated bit-identical to pristine.
- **v15S — 2026-10-09: diagnostics + S2 bridge.** `core/mpe_diag.h` makes
  every rejection name itself (subsystem, stable code, engine `file:line`,
  runtime values), fanned out to stderr, event log, debug terminal;
  `diag` replays the ring. Distinct return codes for every attach/detach/
  loader rejection — all still `< 0`. Everything modular now lives under
  `ecosystem/` (top-level `plugins/` removed) and the loader jails paths
  there. S2 molecular bridge renders a live petri dish: spawns `s2tui`,
  configures it through the existing shell, and stages strict `S2SAVE1`
  frames as static bodies with zero-force bond lines — display-only and
  stage-once. Suite 45 registered / 43 blocking green (added `diag_naming`,
  floor 44 → 45); runner quick 59/59; runner full 226 checks with 3 known
  pre-existing reds documented above.
- **v15S (through 2026-10-07)** — GTK4 port, module system (MPI hot-plug),
  per-world config, data-structure upgrades (growable pools, O(1) caches),
  kernel global-state removal, TUI stress suite (`stress`/`ccd` scenes,
  backend flags), 44 registered / 42 blocking green plus a 234-check
  verification profile (232 pass + 2 xfail + 6 info) green under ASan+
  UBSan; DESPOT audits 2026-10-01 through 2026-10-07 closed numerous
  findings (floor raised 42→44, F11 torture-leak recurrence closed, every
  physics formula re-verified against textbook statements, `[CLAMP-TAUTOLOGY]`
  closed, `[FRICTION-THRESH]` root-caused and fixed, regime matrix revived,
  FTC streamlined to one field / one robot / one controller).
- **v15R3 (release)** — configuration system, physics-truth pass, full
  constraint framework, TUI debugger + snapshot suite, 29/29 headless green.
  Release notes: [`release_notes_v15R3.md`](release_notes_v15R3.md).
- **v15R2** — config-system hardening + MFS robotics (prior RC, parked now
  consolidated in `v15S/src/ecosystem/mfs/`).
- **v1.4 Alpha RC3** — domain-driven restructure, spatial-hash broadphase,
  physics-world encapsulation.
- **v1.4 Alpha 2** — warm-starting solver, multi-point contact manifolds.
- **v1.4 Alpha RC1** — spring joints, joint renderer, color painting, OBB
  raycast selection.
- **v1.3** — established instanced rendering and spatial-hash direction.

See [`v15S/evolution.txt`](v15S/evolution.txt) for the full lineage back
to stage 0.
