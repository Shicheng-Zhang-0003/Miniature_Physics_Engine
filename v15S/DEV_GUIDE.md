# MPE Contributor / Developer Guide — v15S

> How to contribute, the commit message style, the audit method, and the
> definition of "done" for a change.

---

## 1. Commit Message Style

Format (mirror the existing history):

```
DD/MM/YY, HHMMSS: type: description
```

Examples from the tree:

```
15/10/26, 164733: test: hive_shot gate (legal-spot cell entry, 30-gate table, shooting guide)
14/10/26, 164633: feat: shoot power-elev verbs (terminal args, RT analog, stick elevation, predictor follows)
13/10/26, 185348: fix: yaw governor plus active damper (sustained rotate caps minus 4.4, was runaway)
13/10/26, 143552: fix: observer Q-filter (unfiltered differencing was bang-bang, free speed -61pc to -0.03pc)
```

Types: `feat`, `fix`, `docs`, `test`, `refactor`, `build`, `ci`.

**Every behavioral change must be verified against a pristine `HEAD` stash**
(`git stash` A/B — the despot's own regression test). A fix that moves a
baseline must prove the baseline moved for the documented reason — never by
assertion.

**Retract false claims publicly.** If you wrote something that was
fabricated or wrong, retract it in the same document where it appeared and
label the retraction (see the `Tank turn` retraction in `REMAINING_WORK.md`).

---

## 2. The Audit Method ("The Despot")

The mathematical lie and the programming weakness must fear you like a
despot. Every claim is read-not-assumed:

- **Math:** every formula re-derived against a textbook (Mirtich, Baraff,
  Catto GDC 2005/2011/2013/2014, Gottschalk 1996, Coumans 2005, Ericson
  RTCD, Goldstein mechanics, Press numerical recipes). Oracles checked for
  circularity (does the expected value restate the code?).
- **Programming:** full-tree C read. Git history mined for recurrence
  (torture-leak, regime-wipe, SKIP-sentinel, count-drift, P11 cast class).
- **Operational:** every headline count re-derived from its cited source.
- **Regression A/B:** every behavior-changing fix verified against a pristine
  `HEAD` worktree.

All scratch goes under `temp/` (project-local). No directory outside the
project tree is written.

---

## 3. Definition of Done

A change is done when:

- [ ] It compiles clean under `-Wall -Wextra -Werror` (`-Werror` is on by
      default).
- [ ] It passes `--profile quick` (60/60, 0 blocking failures).
- [ ] Determinism twins agree bitwise at 1/4/8 threads (`MPE_THREADS=1,4,8
      ./test_determinism`).
- [ ] It is verified against pristine `HEAD` (not a regression unless
      documented as an intended baseline move).
- [ ] Any documentation it touches is updated (readme, ARCHITECTURE,
      PHYSICS, MULTICORE, MODULES, CONFIG, TESTING, S2_BRIDGE, FTC, BUILD,
      KNOWN_LIMITATIONS, REMAINING_WORK, RELEASE_GATES).
- [ ] Any new tunable is added to `config/mpe_config_schema.c` (exactly
      once).
- [ ] Any failure path is named via `mpe_diag` (source, stable code,
      file:line, runtime values) — no bare `-1` guesses.
- [ ] Committed in a batch of similar files, pushed to `main`.

---

## 4. Directory Layout (where to add code)

| To change… | Edit… |
|---|---|
| Integrator / step pipeline | `core/physics_world.c` |
| Solver (impulses, friction, Poisson, split) | `physics/collision_solver.c` |
| Narrowphase shapes | `physics/collision_*.c` |
| Broadphase | `physics/broadphase.c` |
| Constraints / joints | `physics/constraint.c`, `physics/revolute_joint.c`, `physics/spring_joint.c` |
| CCD | `physics/collision_ccd.c` |
| Islands / sleep | `physics/islands.c` |
| Determinism math | `core/det_math.c`, `core/det_math.h` |
| Diagnostics | `core/mpe_diag.c`, `core/mpe_diag.h` |
| Module ABI / loader | `core/mpe_module.h`, `core/mpe_loader.c`, `core/mpe_registry.c` |
| Config | `config/mpe_config.h`, `config/mpe_config_schema.c` |
| World | `core/physics_world.h`, `core/physics_world.c` |
| Rigid body | `core/rigidbody.h`, `core/rigidbody.c` |
| Rendering | `render/` |
| Scene I/O | `scene/` |
| UI / input / terminal | `ui_input/` |
| TUI | `tui/` |
| FTC drivetrain | `ecosystem/mfs/modules/ftc/submodules/` |
| MGB bridge | `ecosystem/mgb/` |
| Capsule shape | `ecosystem/capsule/mpe_capsule.c` |
| Tests | `tests/` (canonical `mpe_suite_*.c`, `paranoia_*.c`) |

---

## 5. Adding a Tunable

Add one line to `config/mpe_config_schema.c` (exactly once):

```c
{"category.key", "Display Name", "help text",
 p_float, cat_world, &g_cfg.world.field, def, min, max, debug_only},
```

Hot-path code reads `g_cfg.<group>.<field>` directly. The menu, terminal,
and save/load iterate the registry automatically.

---

## 6. Adding a Test Case

1. Add the test function to `tests/mpe_suite_a/b/c/d.c`.
2. Register it in the suite registry (`tests/mpe_suite_main.c`).
3. Run under all five regimes (`MPE_TEST_REGIME=default|light|heavy|brittle|sticky`).
4. Verify against pristine `HEAD`.
5. Update `TESTING.md` if the table of tests changes.

---

## 7. Documentation Files

| File | Purpose |
|---|---|
| `readme.md` | Project overview, features, build, quick start |
| `ARCHITECTURE.md` | Kernel design, world, modules, data structures |
| `PHYSICS.md` | Every equation: integrator, solver, collision, constraints |
| `MULTICORE.md` | Threading model, determinism guarantees |
| `MODULES.md` | MPI module system (hot-plug physics) |
| `CONFIG.md` | Configuration system (78 tunables) |
| `TESTING.md` | Test suite, regime matrix, external oracles |
| `S2_BRIDGE.md` | S2 molecular bridge |
| `FTC.md` | FTC robotics / drivetrain |
| `BUILD.md` | Build instructions (all platforms) |
| `KNOWN_LIMITATIONS.md` | Honest limits and unsupported cases |
| `REMAINING_WORK.md` | Live TODO and open items |
| `RELEASE_GATES.md` | Release exit criteria (P0/P1/P2/P3) |
| `RELEASE_POLICY.md` | Release policy and change classes |
| `docs/DIAGNOSTICS.md` | Self-reporting diagnostics & S2 bridge |
| `docs/NUMERICAL_GUARANTEES.md` | Precise numerical budgets, what holds / what does not |
