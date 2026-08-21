# MPE v15R1 Release Gates

This document defines the exit criteria for tagging `v15R1`.

`v15R1` is the first release candidate of the v15 series, introducing
the centralised configuration system.

---

## Gate Rules

### P0 Gates
P0 gates are mandatory.
If any P0 gate fails, `v15R1` must not be tagged.

### P1 Gates
P1 gates are strongly recommended.
A P1 gate may be deferred only if:
1. it is explicitly documented as a known limitation, and
2. it does not undermine overall stability.

### P2 / P3 Gates
P2 and P3 gates are optional for `v15R1`.
They should be recorded as post-stable work items.

---

## Mandatory P0 Release Gates

### 1. Release Freeze
- [X] The `v15R1` release policy is present and acknowledged.
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
- [X] Startup prints the correct version string (`MPE v15R1`).
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
- [X] Physics behaviour at default config is identical to v14S.

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
- [X] F10 long-run validation passes.
- [X] F11 config torture test runs without crash.
- [X] The engine can idle for several minutes without explosion.

### 10. Configuration System
- [X] Config menu (key 6) opens and navigates correctly.
- [X] All 57 tunable parameters are editable via the menu.
- [X] Debug-only parameters are refused in Game Mode.
- [X] Config saves to `status/engine.cfg` on exit.
- [X] Config loads on startup and overrides defaults.
- [X] Corrupt or missing config file does not crash the engine.
- [X] Terminal `env` lists all parameters grouped by category.
- [X] Terminal `export KEY=value` works for any registered key.
- [X] Terminal `config save|load|reset` works correctly.
- [X] Extreme values are clamped to registered bounds.
- [X] F11 torture test randomises without NaN or crash.
- [X] Config reset restores v14S-identical behaviour.

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

The following are not required for `v15R1`:
- full global-state removal beyond config extraction,
- full `PhysicsWorld` encapsulation,
- multithreading,
- continuous collision detection,
- generic constraint framework,
- solver islanding,
- scene format version 2,
- complete UI state-machine rewrite,
- Wayland mouse-lock support,
- per-object config persistence in scene files.

These belong after `v15R1`.

---

## Release Decision

`v15R1` may be tagged only when:
1. all P0 gates pass,
2. all accepted P1 gates pass or are documented as known limitations,
3. the validation checklist has been run,
4. the repository tree is clean,
5. and the release notes are written.

If any mandatory gate fails, the correct action is:
- fix the gate failure,
- rerun validation,
- and only then re-evaluate `v15R1`.
