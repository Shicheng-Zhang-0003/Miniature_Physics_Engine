# MPE v14S Release Gates

# Legacy Backlog --> v14S stabilised. v14t15 cycle commencing soon.

This document defines the exit criteria for promoting `v14A3` to `v14S`.

`v14S` is the stable release form of `v14A3`.

The stable release is not required to be perfect.
It is required to be:

- buildable,
- runnable,
- stable,
- observable,
- documented,
- and repeatable.

---

## Gate Rules

### P0 Gates

P0 gates are mandatory.

If any P0 gate fails, `v14S` must not be tagged.

### P1 Gates

P1 gates are strongly recommended.

A P1 gate may be deferred only if:

1. it is explicitly documented as a known limitation, and
2. it does not undermine overall stability.

### P2 / P3 Gates

P2 and P3 gates are optional for `v14S`.

They should be recorded as post-stable work items.

---

## Mandatory P0 Release Gates

### 1. Release Freeze

- [x] The `v14A3` release freeze policy is present and acknowledged.
- [x] No new features are being added.
- [x] Only correctness, stability, validation, documentation, and hygiene changes are accepted.

### 2. Build

- [x] `make clean` succeeds.
- [x] `make` succeeds.
- [x] The engine binary is produced.
- [x] There are no new compiler errors.
- [x] Compiler warnings are reviewed and understood.

### 3. Startup

- [x] Engine starts using the documented workflow.
- [x] Startup prints the correct version string.
- [x] Shaders load successfully.
- [x] The main window opens.
- [x] The grid renders.
- [x] The overlay renders.
- [x] There is no uncontrolled GL error spam.

### 4. Shader / Render Failure Visibility

- [x] Shader compilation failure is reported clearly.
- [x] Shader link failure is reported clearly.
- [x] Missing shader files are reported clearly.
- [x] The engine does not silently continue in a broken render state.

### 5. Input and Lifecycle

- [x] Closing the window quits the program.
- [x] Mouse lock can be acquired.
- [x] Mouse lock can be released.
- [x] Focus loss clears stuck keyboard state.
- [x] Focus loss clears stuck mouse state.
- [x] Dialogs do not leave editor state stuck.

### 6. Editor Stability

- [x] Selecting an object does not crash.
- [x] Deleting the selected object does not crash.
- [x] Deleting a jointed object does not crash.
- [x] Deleting a marked joint object does not crash.
- [x] Opening menus with an invalid selection does not crash.
- [x] Save/load with menus open does not crash.

### 7. Physics Stability

- [x] Objects rest on the floor without explosive jitter.
- [x] Cubes stack with reasonable stability.
- [x] Spheres and cubes collide correctly.
- [x] Restitution produces bounce.
- [x] Friction affects sliding.
- [x] Sleeping objects wake when hit.
- [x] Sleeping stacks remain sleeping once settled.
- [x] No NaNs appear after normal use.
- [x] No NaNs appear after stress testing.

### 8. Broadphase / Solver Visibility

- [x] Broadphase node overflow is visible.
- [x] Broadphase pair overflow is visible.
- [x] Manifold overflow is visible.
- [x] Pair-dedupe exhaustion is visible or safely handled.
- [x] Debug counters are visible in overlay and/or validation report.

### 9. Validation Tests

- [x] F5 stability stack passes.
- [x] F6 sleep/wake test passes.
- [x] F7 editor torture test passes.
- [x] F8 spawn stress test passes.
- [x] F9 validation report prints useful state.
- [x] The engine can idle for several minutes without explosion.

### 10. Documentation

- [x] README matches the code.
- [x] User guide matches the code.
- [x] Validation checklist matches the current version.
- [x] Broadphase description matches the implementation.
- [x] Physics timestep description matches the implementation.
- [x] Known limitations are documented.

### 11. Repository Hygiene

- [x] Build artifacts are not tracked.
- [x] Object files are not tracked.
- [x] Dependency files are not tracked.
- [x] Backup shader files are removed or isolated.
- [x] Duplicate documentation is reduced or clarified.
- [x] A `.gitignore` exists.

### 12. Sanitizer / Debug Validation

- [x] A debug build with AddressSanitizer is available or manually used.
- [x] A debug build with UndefinedBehaviorSanitizer is available or manually used.
- [x] Normal validation passes under sanitizer builds.
- [x] No severe sanitizer errors are present.

---

## Recommended P1 Release Gates

### Scene Save / Load

- [x] Saving a scene works.
- [x] Loading a scene works.
- [x] Loading resets editor/menu/selection state.
- [x] Save/load failure is reported.
- [x] Scene format limitations are documented.

### Performance Sanity

- [x] CPU usage drops when the scene is sleeping.
- [x] Overlay updates do not dominate frame time.
- [x] Redundant sanitization passes are reduced.
- [x] Stress scenes remain usable.

### User Feedback

- [x] Object capacity exhaustion is visible to the user.
- [x] Save/load failure is visible to the user.
- [x] Shader failure is visible to the user.

---

## Deferred / Post-Stable Work

The following are not required for `v14S`:

- full global-state removal,
- full `PhysicsWorld` encapsulation,
- multithreading,
- continuous collision detection,
- generic constraint framework,
- solver islanding,
- scene format version 2,
- complete UI state-machine rewrite.

These belong after `v14S`.

---

## Release Decision

`v14S` may be tagged only when:

1. all P0 gates pass,
2. all accepted P1 gates pass or are documented as known limitations,
3. the validation checklist has been run,
4. the repository tree is clean,
5. and the release notes are written.

If any mandatory gate fails, the correct action is:

- fix the gate failure,
- rerun validation,
- and only then re-evaluate `v14S`.

