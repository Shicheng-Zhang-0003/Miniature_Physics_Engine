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
- [ ] The `v15R1` release policy is present and acknowledged.
- [ ] No new features beyond the config system are being added.
- [ ] Only correctness, stability, validation, documentation, and hygiene changes are accepted.

### 2. Build
- [ ] `make clean` succeeds.
- [ ] `make` succeeds.
- [ ] The engine binary is produced.
- [ ] There are no new compiler errors.
- [ ] Compiler warnings are reviewed and understood.

### 3. Startup
- [ ] Engine starts using the documented workflow.
- [ ] Startup prints the correct version string (`MPE v15R1`).
- [ ] Config system initialises (prints `[config] loaded` or `[config] defaults active`).
- [ ] Shaders load successfully.
- [ ] The main window opens.
- [ ] The grid renders.
- [ ] The overlay renders.

### 4. Shader / Render Failure Visibility
- [ ] Shader compilation failure is reported clearly.
- [ ] Shader link failure is reported clearly.
- [ ] Missing shader files are reported clearly.
- [ ] The engine does not silently continue in a broken render state.

### 5. Input and Lifecycle
- [ ] Closing the window quits the program.
- [ ] Config is saved on clean exit.
- [ ] Mouse lock can be acquired.
- [ ] Mouse lock can be released.
- [ ] Focus loss clears stuck keyboard state.
- [ ] Focus loss clears stuck mouse state.
- [ ] Dialogs do not leave editor state stuck.
- [ ] Config menu (key 6) opens and closes correctly.
- [ ] Config menu keys 0-9 work inside the menu.
- [ ] Config menu does not interfere with other menus (7/8/9).

### 6. Editor Stability
- [ ] Selecting an object does not crash.
- [ ] Deleting the selected object does not crash.
- [ ] Deleting a jointed object does not crash.
- [ ] Deleting a marked joint object does not crash.
- [ ] Opening menus with an invalid selection does not crash.
- [ ] Save/load with menus open does not crash.

### 7. Physics Stability
- [ ] Objects rest on the floor without explosive jitter.
- [ ] Cubes stack with reasonable stability.
- [ ] Spheres and cubes collide correctly.
- [ ] Restitution produces bounce.
- [ ] Friction affects sliding.
- [ ] Sleeping objects wake when hit.
- [ ] Sleeping stacks remain sleeping once settled.
- [ ] No NaNs appear after normal use.
- [ ] No NaNs appear after stress testing.
- [ ] Physics behaviour at default config is identical to v14S.

### 8. Broadphase / Solver Visibility
- [ ] Broadphase node overflow is visible.
- [ ] Broadphase pair overflow is visible.
- [ ] Manifold overflow is visible.
- [ ] Pair-dedupe exhaustion is visible or safely handled.
- [ ] Debug counters are visible in overlay and/or validation report.

### 9. Validation Tests
- [ ] F5 stability stack passes.
- [ ] F6 sleep/wake test passes.
- [ ] F7 editor torture test passes.
- [ ] F8 spawn stress test passes.
- [ ] F9 validation report prints useful state including config dump.
- [ ] F10 long-run validation passes.
- [ ] F11 config torture test runs without crash.
- [ ] The engine can idle for several minutes without explosion.

### 10. Configuration System
- [ ] Config menu (key 6) opens and navigates correctly.
- [ ] All 57 tunable parameters are editable via the menu.
- [ ] Debug-only parameters are refused in Game Mode.
- [ ] Config saves to `status/engine.cfg` on exit.
- [ ] Config loads on startup and overrides defaults.
- [ ] Corrupt or missing config file does not crash the engine.
- [ ] Terminal `env` lists all parameters grouped by category.
- [ ] Terminal `export KEY=value` works for any registered key.
- [ ] Terminal `config save|load|reset` works correctly.
- [ ] Extreme values are clamped to registered bounds.
- [ ] F11 torture test randomises without NaN or crash.
- [ ] Config reset restores v14S-identical behaviour.

### 11. Documentation
- [ ] README matches the code.
- [ ] User guide matches the code.
- [ ] Validation checklist matches the current version.
- [ ] Broadphase description matches the implementation.
- [ ] Physics timestep description matches the implementation.
- [ ] Config system is documented.
- [ ] Known limitations are documented.

### 12. Repository Hygiene
- [ ] Build artifacts are not tracked.
- [ ] Object files are not tracked.
- [ ] Dependency files are not tracked.
- [ ] Backup shader files are removed or isolated.
- [ ] Duplicate documentation is reduced or clarified.
- [ ] A `.gitignore` exists.

### 13. Sanitizer / Debug Validation
- [ ] A debug build with AddressSanitizer is available or manually used.
- [ ] A debug build with UndefinedBehaviorSanitizer is available or manually used.
- [ ] Normal validation passes under sanitizer builds.
- [ ] No severe sanitizer errors are present.

---

## Recommended P1 Release Gates

### Scene Save / Load
- [ ] Saving a scene works.
- [ ] Loading a scene works.
- [ ] Loading resets editor/menu/selection state.
- [ ] Save/load failure is reported.
- [ ] Scene format limitations are documented.

### Performance Sanity
- [ ] CPU usage drops when the scene is sleeping.
- [ ] Overlay updates do not dominate frame time.
- [ ] Redundant sanitization passes are reduced.
- [ ] Stress scenes remain usable.

### User Feedback
- [ ] Object capacity exhaustion is visible to the user.
- [ ] Save/load failure is visible to the user.
- [ ] Shader failure is visible to the user.
- [ ] Config load failure is visible to the user.

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
