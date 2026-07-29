# MPE v14S Release Gates

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

- [ ] The `v14A3` release freeze policy is present and acknowledged.
- [ ] No new features are being added.
- [ ] Only correctness, stability, validation, documentation, and hygiene changes are accepted.

### 2. Build

- [ ] `make clean` succeeds.
- [ ] `make` succeeds.
- [ ] The engine binary is produced.
- [ ] There are no new compiler errors.
- [ ] Compiler warnings are reviewed and understood.

### 3. Startup

- [ ] Engine starts using the documented workflow.
- [ ] Startup prints the correct version string.
- [ ] Shaders load successfully.
- [ ] The main window opens.
- [ ] The grid renders.
- [ ] The overlay renders.
- [ ] There is no uncontrolled GL error spam.

### 4. Shader / Render Failure Visibility

- [ ] Shader compilation failure is reported clearly.
- [ ] Shader link failure is reported clearly.
- [ ] Missing shader files are reported clearly.
- [ ] The engine does not silently continue in a broken render state.

### 5. Input and Lifecycle

- [ ] Closing the window quits the program.
- [ ] Mouse lock can be acquired.
- [ ] Mouse lock can be released.
- [ ] Focus loss clears stuck keyboard state.
- [ ] Focus loss clears stuck mouse state.
- [ ] Dialogs do not leave editor state stuck.

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
- [ ] F9 validation report prints useful state.
- [ ] The engine can idle for several minutes without explosion.

### 10. Documentation

- [ ] README matches the code.
- [ ] User guide matches the code.
- [ ] Validation checklist matches the current version.
- [ ] Broadphase description matches the implementation.
- [ ] Physics timestep description matches the implementation.
- [ ] Known limitations are documented.

### 11. Repository Hygiene

- [ ] Build artifacts are not tracked.
- [ ] Object files are not tracked.
- [ ] Dependency files are not tracked.
- [ ] Backup shader files are removed or isolated.
- [ ] Duplicate documentation is reduced or clarified.
- [ ] A `.gitignore` exists.

### 12. Sanitizer / Debug Validation

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

