# MPE v14A3 Release Freeze Policy

This tree is in **v14A3 RC freeze**.

The purpose of this stage is to stabilise the engine for the upcoming `v14S`
stable release.

## Freeze Rule

Until `v14S` is tagged, the following rule applies:

> No new features are to be added to this branch.

Only the following change classes are accepted:

1. Correctness fixes.
2. Crash fixes.
3. Stability fixes.
4. Validation and testing improvements.
5. Documentation corrections.
6. Build and repository hygiene.
7. Small performance fixes only where they remove obvious waste or instability.

## Explicitly Deferred

The following are deferred until after `v14S`:

- New physics features.
- New rendering features.
- New editor systems.
- New constraint types.
- Large architectural refactors.
- Full global-state removal.
- Multithreading.
- Continuous collision detection.
- Generic constraint framework.
- Scene format version 2.

## Release Goal

The goal of `v14S` is not to make the engine perfect.

The goal is to make the current engine:

- build cleanly,
- run predictably,
- fail visibly,
- pass validation,
- and be release-worthy as the stable form of `v14A3`.

