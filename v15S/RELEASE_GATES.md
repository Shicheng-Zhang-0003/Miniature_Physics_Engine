# MPE Release Gates (v15R3 record + v15S addendum)

This document defines the exit criteria for tagging `v15R3`, plus the
v15S addendum gates covering the GTK4 port, module system, and
data-structure upgrades. The v15R3 record below is historical; v15S
must additionally pass §15.

`v15R3` is the MPE-only release of the v15 series, carrying the
centralised configuration system (prior RCs: v15R1, v15R2).

---

## Gate Rules

### P0 Gates
P0 gates are mandatory.
If any P0 gate fails, `v15R3` must not be tagged.

### P1 Gates
P1 gates are strongly recommended.
A P1 gate may be deferred only if:
1. it is explicitly documented as a known limitation, and
2. it does not undermine overall stability.

### P2 / P3 Gates
P2 and P3 gates are optional for `v15R3`.
They should be recorded as post-stable work items.

---

## Mandatory P0 Release Gates

### 1. Release Freeze
- [X] The `v15R3` release policy is present and acknowledged.
- [X] No new features beyond the config system are being added.
- [X] Only correctness, stability, validation, documentation, and hygiene changes are accepted.

### 2. Build
- [X] `make clean` succeeds.
- [X] `make` succeeds.
- [X] The engine binary is produced.
- [X] There are no new compiler errors.
- [X] Compiler warnings are reviewed and understood.

### 3. Startup
- [X] Engine starts using the documented workflow.
- [X] Startup prints the correct version string (`MPE v15S-dev (GTK4)` from `a3_version_string`; window title derives from it).
- [X] Config system initialises (prints `[config] loaded` or `[config] defaults active`).
- [X] Shaders load successfully.
- [X] The main window opens.
- [X] The grid renders.
- [X] The overlay renders.

### 4. Shader / Render Failure Visibility
- [X] Shader compilation failure is reported clearly.
- [X] Shader link failure is reported clearly.
- [X] Missing shader files are reported clearly.
- [X] The engine does not silently continue in a broken render state.

### 5. Input and Lifecycle
- [X] Closing the window quits the program.
- [X] Config is saved on clean exit.
- [X] Mouse lock can be acquired.
- [X] Mouse lock can be released.
- [X] Focus loss clears stuck keyboard state.
- [X] Focus loss clears stuck mouse state.
- [X] Dialogs do not leave editor state stuck.
- [X] Config menu (key 6) opens and closes correctly.
- [X] Config menu keys 0-9 work inside the menu.
- [X] Config menu does not interfere with other menus (7/8/9).

### 6. Editor Stability
- [X] Selecting an object does not crash.
- [X] Deleting the selected object does not crash.
- [X] Deleting a jointed object does not crash.
- [X] Deleting a marked joint object does not crash.
- [X] Opening menus with an invalid selection does not crash.
- [X] Save/load with menus open does not crash.

### 7. Physics Stability
- [X] Objects rest on the floor without explosive jitter.
- [X] Cubes stack with reasonable stability.
- [X] Spheres and cubes collide correctly.
- [X] Restitution produces bounce.
- [X] Friction affects sliding.
- [X] Sleeping objects wake when hit.
- [X] Sleeping stacks remain sleeping once settled.
- [X] No NaNs appear after normal use.
- [X] No NaNs appear after stress testing.
- [X] Physics behaviour at default config is identical to v14S, except intentional truth fixes: angular damping defaults to vacuum (1.0; v14S hardcoded 0.97 rotary damping) and velocity/restitution clamps are removed.

### 8. Broadphase / Solver Visibility
- [X] Broadphase node overflow is visible.
- [X] Broadphase pair overflow is visible.
- [X] Manifold overflow is visible.
- [X] Pair-dedupe exhaustion is visible or safely handled.
- [X] Debug counters are visible in overlay and/or validation report.

### 9. Validation Tests
- [X] F5 stability stack passes.
- [X] F6 sleep/wake test passes.
- [X] F7 editor torture test passes.
- [X] F8 spawn stress test passes.
- [X] F9 validation report prints useful state including config dump.
- [X] F10 long-run validation passes (three-gate wake: first-touch novelty + fast-other + deep).
- [X] F11 config torture test runs without crash.
- [X] F11 verdict is robustness-only (no NaN, nothing fallen); speeds reported, never gated.
- [X] F11 pins solver resolution (gravity −17…−1, ≥96 iterations — proven envelope for the 10:1 column); material/world extremes stay fully random.
- [X] Headless suite 32/32 green (Suite v2 `test_mpe_suite --all` — 29 physics + 3 diag-informational), including `f10_long_run`, `sleep_contact_wake`, `f11_torture`, `loader_lifecycle`, `ftc_ecosystem`.
- [X] `mpe-tui` snapshot suite green for all scenes (`make tui-smoke`: demo/tower/pendulum/springlab/f10/stress/ccd).
- [X] The engine can idle for several minutes without explosion.

### 10. Configuration System
- [X] Config menu (key 6) opens and navigates correctly.
- [X] All 78 tunable parameters are editable via the menu.
- [X] Debug-only parameters are refused in Game Mode.
- [X] Config saves to `status/engine.cfg` on exit.
- [X] Config loads on startup and overrides defaults.
- [X] Corrupt or missing config file does not crash the engine.
- [X] Terminal `env` lists all parameters grouped by category.
- [X] Terminal `export KEY=value` works for any registered key.
- [X] Terminal `config save|load|reset` works correctly.
- [X] Extreme values are clamped to registered bounds.
- [X] F11 torture test randomises without NaN or crash.
- [X] Config reset restores v14S-identical behaviour (same truth-fix exceptions as above).

### 11. Documentation
- [X] README matches the code.
- [X] User guide matches the code.
- [X] Validation checklist matches the current version.
- [X] Broadphase description matches the implementation.
- [X] Physics timestep description matches the implementation.
- [X] Config system is documented.
- [X] Known limitations are documented.

### 12. Repository Hygiene
- [X] Build artifacts are not tracked.
- [X] Object files are not tracked.
- [X] Dependency files are not tracked.
- [X] Backup shader files are removed or isolated.
- [X] Duplicate documentation is reduced or clarified.
- [X] A `.gitignore` exists.

### 13. Sanitizer / Debug Validation
- [X] A debug build with AddressSanitizer is available or manually used.
- [X] A debug build with UndefinedBehaviorSanitizer is available or manually used.
- [X] Normal validation passes under sanitizer builds.
- [X] No severe sanitizer errors are present.

---

## Recommended P1 Release Gates

### Scene Save / Load
- [X] Saving a scene works.
- [X] Loading a scene works.
- [X] Loading resets editor/menu/selection state.
- [X] Save/load failure is reported.
- [X] Scene format limitations are documented.

### Performance Sanity
- [X] CPU usage drops when the scene is sleeping.
- [X] Overlay updates do not dominate frame time.
- [X] Redundant sanitization passes are reduced.
- [X] Stress scenes remain usable.

### User Feedback
- [X] Object capacity exhaustion is visible to the user.
- [X] Save/load failure is visible to the user.
- [X] Shader failure is visible to the user.
- [X] Config load failure is visible to the user.

---

## Deferred / Post-Stable Work

The following are not required for `v15R3`:
- multithreading,
- in-engine creation UI and scene persistence for fixed/distance/prismatic/rope
  (solver supports all five constraint types + springs; menus persist
  springs + all five constraint types),
- complete UI state-machine rewrite,
- per-object config persistence in scene files.

Completed since this list was written (no longer deferred): full
global-state removal (sim state is per-world), `PhysicsWorld`
encapsulation, continuous collision detection (swept TOI clamp), solver
islanding (union-find sleep islands), scene format v2 (stable IDs,
joints, CRC32).

These belong after `v15R3`.

---


### 14. Joints and constraints
- [X] Revolute joints hold anchors and allow swing (`revolute` test)
- [X] Revolute axis drift corrected (positional Baumgarte, once per tick after the loop)
- [X] Revolute angle limits tracked once per tick with velocity-level enforcement
- [X] Fixed / prismatic / distance / rope constraints solved (2-D perpendicular lock, measured limits, pull-only rope)
- [X] Spring joints with live rendering
- [X] Joint pre-step runs once per tick (angle/slide tracking never integrates per-iteration)
- [X] Fixed-timestep accumulator (60Hz deterministic)
- [X] Scene save/load preserves joint assemblies (v200: springs + all five constraint types, `scene_roundtrip` test)

---

## v15S Addendum Gates (§15)

### 15. Modularity, data structures, GTK4
- [X] Engine builds and runs on GTK4; mouse lock works on X11 (verified). Wayland path is Wayland-safe by design (no X11-only lock) but P1-unverified until a Weston/Sway matrix is logged.
- [X] No simulation globals in the kernel: every step takes an explicit `physics_world`; the single GUI primary is app-owned (`core/mpe_primary.c`); registry is process-global by design (documented).
- [X] Module system: shapes/broadphase/solver/tick-modules register and hot-load (`.so` ABI-checked, CWD-jailed); unload refuses busy (`-2`) and purges all live worlds pre-`dlclose`; builtins refuse silent takeover; per-stage `mod_state` threaded; `mod` terminal command and TUI `--broadphase/--solver` flags work; `module` + `loader_lifecycle` headless tests green (lifecycle proven against the real capsule `.so`: load/busy/detach/unload/purge/reload).
- [X] Per-world config authoritative in every hot path (narrowphase/solver/CCD/joints/depenetration/broadphase); NULL-call convention keeps direct unit callers working.
- [X] O(1) contact-pair probes and id→index lookups (verified + linear fallback); islands joint pass O(J+B); growable body/contact pools with ceilings; small-first broadphase nodes; process-wide determinism counters asserted zero in-contract.
- [X] Single narrowphase dispatch (registry-first) shared by world step, legacy GUI tick, depenetration pass, and spawn resolver; single canonical spring entry for both step paths.
- [X] TUI stress green: `stress`/`ccd` scenes, 3600-tick f10 settle (maxV 0.0), byte-identical reruns, per-scene configs, pool visibility in dumps.


---

## Release Decision

`v15R3` may be tagged only when:
1. all P0 gates pass,
2. all accepted P1 gates pass or are documented as known limitations,
3. the validation checklist has been run,
4. the repository tree is clean,
5. and the release notes are written.

If any mandatory gate fails, the correct action is:
- fix the gate failure,
- rerun validation,
- and only then re-evaluate `v15R3`.

### Release verdict (v15R3, tagged)

All P0 gates pass: clean build with zero new errors, 32/32 headless green (29 physics + 3 diag),
`tui-smoke` green, F10 settle verdict green (headless 3600-tick equivalent
plus committed `f10_long_run`), F11 robustness green in-engine and headless
(`f11_torture`). P1 known limitations are documented in
[`release_notes_v15R3.md`](../release_notes_v15R3.md) and
[`how_to_use.md`](how_to_use.md) (joint creation UI +
persistence scope, SIMD/multithreading). Tree frozen (`a3_release_freeze = 1`):
correctness, stability, validation, documentation, and hygiene changes only.
