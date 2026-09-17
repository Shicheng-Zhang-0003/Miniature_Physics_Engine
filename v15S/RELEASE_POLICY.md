# MPE v15R3 Release Policy

This tree is tagged **v15R3 release** (`a3_release_freeze = 1`).
Accepted changes from here on: correctness, stability, validation,
documentation, and hygiene only — no new features.

## What v15 Delivered

The v15 series introduced the centralised configuration system:
1. `src/config/` folder with LOCKED constants manifest and tunable registry.
2. Engine code migration to read from the config store.
3. Permanent storage of tunables to `status/engine.cfg`.
4. In-engine menu toggle (key `6`) for live parameter editing.
5. Terminal `env`/`export` rewire to the registry.

## Change Classes Accepted

Under the v15R3 freeze:

1. Correctness fixes with headless proof (new or extended tests).
2. Stability fixes required by validation (F5–F11, 29-test suite).
3. Build and repository hygiene.
4. Documentation updates to match the architecture.
5. Validation improvements (tests, TUI snapshot scenes, gates).

## Explicitly Deferred (post-release)

- Multithreading (islands currently skip-only, solve stays single-threaded).
- In-engine creation UI and scene persistence for fixed/distance/prismatic/
  rope (solver supports all five constraint types + springs; v200 persists
  springs + revolutes).
- Complete UI state-machine rewrite (magic-level dispatch split, not yet FSM).
- Wayland mouse-lock support.
- Per-object config persistence beyond nice_value.
- Quadratic aero drag (current drag is linear-viscous retention).
- SIMD math (scalar core; ~1136-object perf wall stands).

## Landed Since the Original List (no longer deferred)

- Full sim-state global removal (physics_world owns bodies/IDs/joints/caches).
- Continuous collision detection (swept TOI + remainder integration).
- Solver islanding (union-find sleep islands).
- Scene format v200 (stable IDs, joints, CRC32, atomic staged load).
- Poisson restitution, split impulse, rolling resistance, gyroscopic torque.
- Physics-truth pass: Verlet-exact free flight, post-integration Poisson gate,
  strict warm-start, true cylinder SDF, hysteresis deleted, speed-clamp and
  restitution-cap deleted. Intentional behavior change vs v14S: default
  angular_damping_scale is now 1.0 (truth vacuum; v14S hardcoded 0.97 rotary
  damping) and sleep can be disabled via sleep.enable for truth validation.
- F11 torture guardrails: gravity −17…−1, solver_iterations ≥96 (proven
  envelope for the 10:1 validation column; convergence proof in
  `src/scene/scene_init.c`). F11 verdict is robustness-only by spec.
- Sleep truth: three-gate wake (first-touch pair novelty via the contact
  cache, fast-other velocity gate, deep overlap) — resting stacks settle
  and sleep; slow pushers and kinematic platforms wake sleepers at any
  speed. Fixes the release-cycle F10 10-stack 13 m/s runaway.
- Terminal debugger + snapshot suite (`mpe-tui`, `src/tui/`): live ncurses
  inspector plus deterministic pipeable state dumps; `make tui-smoke`.
- Adversarial headless tests: `f10_long_run` (settle gates incl. run-max),
  `sleep_contact_wake` (first-touch wake + no-churn control), `f11_torture`
  (fixed-seed config extremes, corruption gates). Suite total: 29/29 green.
- Driven-wheel truth: test moved into the resolvable spin regime with
  load-bearing gates (grounded height, rolling coupling, spin cap).

## Release Goal (met)

`v15R3` is tagged:
- all MPE_TASK_24 through MPE_TASK_41 are complete,
- all P0 gates pass (see `RELEASE_GATES.md` release verdict),
- the config system round-trips (save → restart → load),
- the menu and terminal both edit live parameters,
  - and physics behaviour at defaults is identical to v14S except the
  documented truth-fix exceptions (vacuum angular damping, removed clamps).
