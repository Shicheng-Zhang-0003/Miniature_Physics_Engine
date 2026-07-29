# MPE v14A3.41 Validation Checklist


<!-- MPE_RELEASE_GATES_SECTION_BEGIN -->
## v14S Release Gates

The stable release is controlled by [`RELEASE_GATES.md`](RELEASE_GATES.md).

Minimum mandatory gates:

- [ ] Release freeze policy is active.
- [ ] Build passes.
- [ ] Startup prints the correct version.
- [ ] Shader/render failures are visible.
- [ ] Input and focus-loss behaviour is stable.
- [ ] Editor deletion/joint torture tests pass.
- [ ] Physics stacks settle without explosion.
- [ ] Sleeping stacks remain sleeping.
- [ ] Overflow counters are visible where applicable.
- [ ] F5-F9 validation tests pass.
- [ ] Documentation matches code.
- [ ] Repository artifacts are cleaned.
- [ ] Sanitizer validation passes.

Do not tag `v14S` until all mandatory P0 gates pass.
<!-- MPE_RELEASE_GATES_SECTION_END -->

This checklist validates the full 1-41 A3 patch sequence.

## Built-in Test Keys

| Key | Test |
|---|---|
| F5 | Spawn 10-cube stability stack |
| F6 | Spawn sleeping cube + moving projectile |
| F7 | Editor torture test: select, joint, delete, reset |
| F8 | Spawn stress test: up to 300 mixed objects |

## Core Lifecycle

- [ ] Closing the window quits the program.
- [ ] Losing window focus clears stuck input.
- [ ] Mouse lock state does not remain stuck after focus loss.

## Editor State

- [ ] Deleting selected object does not crash.
- [ ] Deleting jointed object does not crash.
- [ ] Deleting marked joint object does not crash.
- [ ] Loading a scene resets selection/menu/cache state.
- [ ] Object menu closes when selection becomes invalid.

## Physics

- [ ] Objects rest on floor without explosive jitter.
- [ ] Cubes stack with reasonable stability.
- [ ] Spheres and cubes collide correctly.
- [ ] Objects with restitution bounce on floor.
- [ ] Surface friction affects sliding.
- [ ] Sleeping objects wake when hit.
- [ ] Joints remain valid after deletion.

## Broadphase

- [ ] Sleeping objects are still discoverable.
- [ ] High object counts do not allocate quadratic dedupe memory.
- [ ] Broadphase overflow counters are visible if overflow occurs.
- [ ] Node pool grows instead of silently dropping objects.

## Rendering

- [ ] Ground grid is visible.
- [ ] Wireframe selection renders.
- [ ] Joint lines render.
- [ ] No per-frame uniform query spam in grid/wireframe/joint paths.

## Build

- [ ] make -C src clean succeeds.
- [ ] make -C src succeeds.
- [ ] Startup prints: MPE v14A3.41

## Final Pass

Run:

    make -C src clean
    make -C src
    ./src/engine

Then manually test:

1. Press F5 — stack settles.
2. Press F6 — sleeping cube wakes on impact.
3. Press F7 — editor torture does not crash.
4. Press F8 — spawn stress remains observable and does not silently fail.
5. Save/load scene with menus open — no crash.
6. Delete selected/jointed objects — no crash.
7. Set friction to 0 and restitution above 0 — objects slide and bounce.

If all pass, the A3 patch sequence is complete.
