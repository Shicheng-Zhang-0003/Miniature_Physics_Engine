# MPE Testing — v15S

> The verification suite: quick/physics/full profiles, regime matrix,
> metamorphic tests, external oracles, and the honest count discipline.

---

## 1. Profiles

```bash
python3 tools/test_runner.py --profile quick   # runner contracts + canonical C suite (46 cases)
python3 tools/test_runner.py --profile physics # quick + isolated legacy (30) + paranoia (14)
python3 tools/test_runner.py --profile full    # physics + ASan/UBSan + MFS (30 gated) + TUI snapshots (9) + engine build
python3 tools/test_runner.py --list            # discover registered test cases
```

All run artifacts are written below the project-local `temp/qa_runs/`
directory. The newest 10 runs are retained by default; set `MPE_KEEP_QA_RUNS`
to change the limit. The runner uses only the Python standard library.

**Strict contract:**
- Suite cases that print `[SKIP]` fail the run unless `--allow-skip`.
- `[XFAIL]` markers are surfaced in summary counts but never block.
- Registry floors: `MIN_SUITE_ENTRIES=46`, `MIN_LEGACY_ENTRIES=30`,
  `MIN_PARANOIA_ENTRIES=14`.
- Summary contract: `Total == registry size` and `Blocking failures == 0`.

The floor exists because a count floor that sits **below** the real registry
size is worse than no floor — it reads as protection while silently
covering deleted coverage (reproduced inside a week in 2026-09-29). The
floor is now pinned at the exact registry size (46) with a contract test.

---

## 2. Canonical C Suite (46 cases)

One binary `test_mpe_suite` (`make build_suite`), exact-name dispatch, one
registry, no substring fan-out. Every test runs under a saved/restored
global config (no leakage). Floor setup is a single audited helper: real
Coulomb contact, either as an explicit mass-0 slab (`mpe_floor_slab`,
preferred: true manifold, per-body friction) or as the infinite solver
plane with synced friction (`mpe_floor_plane`).

Assertions print `file:line` + values on failure and **keep going**, so one
run reports every broken oracle instead of aborting at the first. NaN
watchdog on every step; determinism counters asserted zero.

Adversarial config cases (`f10_long_run`, `f11_torture`) pin the documented
convergence prerequisites before randomizing (gravity −17…−1, ≥96
iterations, `sleep.enable = 1`, `solver.warm_start_damping = 0`, safety net
on). `f11_torture` gates the 0.05 m cube-cube overlap bound; the GUI F11
reports the 10-cube column state but only gates corruption.

### Harness Contract (DESPOT-2026-09-29)

The single worst bug of the audit was not a physics bug: for the entire
life of the v1 suite, `mpe_suite_main.c` never called `mpe_config_init()`,
so every test ran against a zero-initialized `g_cfg` — 0 solver iterations,
0 bias, 0 slop, 0 restitution, 0 friction. The solver was not iterating
and 34/34 were green.

The harness now has two jobs it previously skipped:

1. **ESTABLISH state, not merely save it.** `mpe_test_begin()` now
   **initialises** `g_cfg` (it only used to save/restore, faithfully
   preserving inherited garbage). Save-and-restore without initialise is a
   harness that launders the bug.
2. **ASSERT its own preconditions.** If the config is degenerate, every
   measurement below it is fiction, and the suite must say so loudly
   instead of reporting a confident green. `mpe_test_end()` refuses to
   certify a result produced under a degenerate config.

---

## 3. Regime Matrix

The entire suite re-runs under five configurations via `MPE_TEST_REGIME`:

| Regime | Iterations | Gravity × | Friction × | Restitution | Sleep |
|---|---|---|---|---|---|
| default | 64 | 1.0 | 1.0 | 0.0 | on |
| light | 8 | 0.25 | 0.25 | 0.0 | on |
| heavy | 128 | 3.0 | 4.0 | 0.95 | on |
| brittle | 64 | 1.0 | 1.0 | 0.95 | on |
| sticky | 64 | 1.0 | 4.0 | 0.0 | on |

A physics invariant that holds at one setting and not another is a bug that
a single golden number cannot see. The regime matrix found a failure on its
first run. Defaults are chosen to stay inside the engine's documented
parameter ranges so we exercise the shipped model, not an unreachable corner.

---

## 4. Metamorphic Tests (`tests/mpe_suite_d.c`)

Six registered metamorphic tests — **no golden numbers**. They assert
relations any correct engine satisfies:

- `meta_rotation` — yaw rotation equivariance (fixture fixed 2026-10-01:
  floorless + yaw-only probe; the earlier "stale contact" theory was a
  misread of per-body flags).
- `meta_convergence` — solver error bounded, knob demonstrably live
  (error is **non-monotonic** in iteration count for a friction stack;
  gate asserts boundedness, not monotonicity).
- `meta_config_wiring` — config parameters demonstrably reach the simulation.
- `sleep_settle` — moves → settles → sleeps-iff-enabled, all regimes.
- `mouse_look_axes` — mouse-look sign convention on 4 axes + diagonals.
- `body_materials_live` — bodies stamped with live materials (config-ordering guard).

Two of these were fixed 2026-10-03 after being found unable to fail:
`meta_rotation`'s failure branch printed `[XFAIL]` without incrementing
`failures`, and `meta_convergence` measured a fixture that was bitwise
identical at 1 through 128 solver iterations. Both fixed and re-verified
by deliberately breaking the engine.

---

## 5. Paranoia Tests (14)

Isolated legacy cases targeting edge cases:

`paranoia_broadphase`, `paranoia_ccd`, `paranoia_config`,
`paranoia_constraints`, `paranoia_contact_solver`,
`paranoia_cylinder_collision`, `paranoia_depenetration_sleep`,
`paranoia_determinism`, `paranoia_energy_momentum`, `paranoia_fp_edges`,
`paranoia_freeflight_exact`, `paranoia_integration_edges`,
`paranoia_scene_persistence`, `paranoia_spring_joints`.

---

## 6. External Truth Battery (`temp/ext_truth.c` + `temp/ext_oracle_check.py`)

Independent of the suite above: separately-coded harness, oracles hand-derived
from closed forms (kinematics, Newton, Coulomb, Hooke, compound pendulum,
Poisson series, moments of inertia), cross-checked by a **second
implementation in Python**. 22/22 green, representative errors:
free-fall 4e-5, projectile 5e-5, pendulum period 0.1%, spring period 0.8%,
sliding stop 1.3%, inertias ≤2e-5 absolute (≤8.1e-7 relative), elastic
exchange exact, hold-creep 0.0, Galileo exact.

Covers free-fall, projectile, bounce heights, pendulum, spring, 1D elastic
exchange, Coulomb stop/hold/slide angles, sphere/box/cylinder inertia via
torque, tower equilibrium, range linearity, mass-independence of fall, and
the restitution threshold.

Not committed (lives in gitignored `temp/`); re-run by compiling
`temp/ext_truth.c` against the `v15S/src` core objects plus `temp/
stubs_ext.c` (exact recipe in `v15S/docs/DESPOT_AUDIT_2026-10-01.md`),
then `python3 temp/ext_oracle_check.py`.

**Known honest limits** confirmed by the battery:
- sliding stop applies to sliding bodies (spheres roll — rolling resistance
  governs them);
- tall-tower equilibrium includes the sleep optimizer (sleepless towers
  lean on solver micro-jitter, documented; F11 pins it on);
- bounce-height oracles carry slop-scale error.

---

## 7. MFS Robotics Suite (`ecosystem/mfs/`)

`build_tests.sh` — unified `mfs_suite --all` (30 gated) plus build gates
and ungated diagnostics; CWD-independent. **Status 2026-10-14: 29/30 green**
— only `tank` is red (pre-existing baseline, moved with cause by the
observer Q-filter, untouched by the handling work). `odometry_yaw` went
red→green on the Q-filter.

The script runs with errexit scoped off around the suite so a failure still
produces its full summary (`FTC RESULT: gated pass=.. fail=..`).

See [`v15S/src/ecosystem/mfs/README.md`](v15S/src/ecosystem/mfs/README.md)
and [`docs/FTC.md`](FTC.md).

---

## 8. TUI Snapshot Suite

`make tui-smoke` — all scenes (demo/tower/pendulum/springlab/f10/stress/ccd)
dump finite state; snapshots are bit-deterministic across runs. Snapshot
sections (`[engine]`, `[body i]`, `[springs]`, `[constraints]`, `[pairs]`,
`[islands]`, `[stats]`, `[result]`) are fixed-format and deterministic.
Dumps report pool usage (`bodies=n/cap`, `cacheCount=n/cap`).

---

## 9. Sanitizer Coverage

Full profile repeats the C, MFS, and TUI checks under combined
AddressSanitizer + UndefinedBehaviorSanitizer. The FTC hotload test
intentionally loads a plugin image containing a second `mpe_module_desc`;
its invocation disables only ASan's duplicate-global heuristic. Leak
detection, address checks, and UBSan remain enabled for that test.

Missing world cleanup in legacy `cylinder_drop`, `driven_wheel`, and MFS
`ftc_integration` fixtures was fixed after LeakSanitizer identified fixture
leaks.

---

## 10. Count Discipline

Headline counts are re-derived from their cited source (registry vs
`MIN_SUITE_ENTRIES` vs readme vs gates vs makefile NOTE) — never assumed.
The v15R3 tag is frozen at 29/29; the v15S head is 46 registered / 44
blocking green (41 physics + 2 diag-informational + ...; see
`RELEASE_GATES.md`). "29 physics + 3 diag" was a v15R3-era figure
contradicting the total in the same sentence; it is gone rather than
restated.

A case that cannot run now reports SKIP and is **excluded from the green
count** — it previously returned 0, so `make test_suite` could report
"29/29 green" with two cases having executed nothing.
