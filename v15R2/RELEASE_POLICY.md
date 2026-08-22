# MPE v15R2 Release Policy

This tree is in **v15R2 development**.

## Cycle Goal

The v15 series introduces the centralised configuration system.
This release candidate covers:

1. `src/config/` folder with LOCKED constants manifest and tunable registry.
2. Engine code migration to read from the config store.
3. Permanent storage of tunables to `status/engine.cfg`.
4. In-engine menu toggle (key `6`) for live parameter editing.
5. Terminal `env`/`export` rewire to the registry.

## Change Classes Accepted

During v15R2 development:

1. Configuration system implementation (Tasks 24–41).
2. Correctness fixes required by the config migration.
3. Build and repository hygiene.
4. Documentation updates to match new architecture.
5. Validation improvements for the new system.

## Explicitly Deferred

- Full global-state removal beyond config extraction.
- Multithreading.
- Continuous collision detection.
- Generic constraint framework.
- Scene format version 2.
- Wayland mouse-lock support.

## Release Goal

`v15R2` may be tagged when:
- all MPE_TASK_24 through MPE_TASK_41 are complete,
- all P0 gates pass,
- the config system round-trips (save → restart → load),
- the menu and terminal both edit live parameters,
- and physics behaviour at defaults is identical to v14S.
