# MPE v15R3 — Release Notes

**Status:** tagged release. Tree frozen (`a3_release_freeze = 1`):
correctness, stability, validation, documentation, and hygiene changes only.

**What it is:** the v15 configuration-system release of the Miniature
Physics Engine — a hand-written 3D rigid-body engine in C (GTK3 + OpenGL
3.3), MPE-only run (MFS robotics parked in `robotics_backup/`).

---

## 1. Headline: the F10 runaway, found and fixed

Late in the cycle, in-engine **F10 failed** while **F11 passed**. Forensics
(TUI portal traces, per-tick attribution, A/B against the pre-sweep tree)
gave a definitive mechanism:

- The top cube of the F10 10-stack crept sideways from tick ~0, friction
  never holding, and was ejected at **13.06 m/s on tick 909** — while final
  speeds read ~0 (it landed and slept). Only the post-transient run-max
  gate sees this failure mode; final-only gating passes it.
- Root cause: wake-on-any-contact kept every resting contact awake forever.
  The stack could never sleep, and never-sleeping micro-motion pumped into
  runaway slide. The pre-sweep tree slept early and stayed calm (run-max
  0.0), which is why the regression hid from every test except F10's
  run-max gate.
- Fix: **three-gate wake** — first-touch pair novelty via the contact cache
  (fires at *any* speed), fast-other velocity gate (backstop), deep overlap
  (backstop). Persistent resting contacts no longer veto sleep.
- Proof: headless 3600-tick F10 equivalent is dead calm (run-max 0.0, all
  27 bodies asleep, KE = 0 at 60 s); committed as `f10_long_run` (1500-tick
  regression test with full F10 gates).

**Release-ritual recommendation:** re-run in-engine F10/F11 on a display
(`run_all.sh` covers build + headless suite; F5–F11 need the GTK window).
The headless equivalents (`f10_long_run`, `f11_torture`) are green here.

## 2. Validation ledger

- **Headless suite: 29/29 green** (`python3 tools/test_runner.py`): the
  prior 26 plus `f10_long_run` (settle gates incl. run-max),
  `sleep_contact_wake` (0.05 m/s pusher wakes a sleeper with 1-tick latency;
  lone sleeper undisturbed), `f11_torture` (fixed-seed config extremes,
  corruption gates).
- **F11 semantics (unchanged):** robustness-only verdict — no NaN, nothing
  fallen; speeds reported, never gated. Resolution pinned (gravity −17…−1,
  ≥96 iterations).
- **Sanitizers:** ASan+UBSan build available (`v15R3/validation/V01.sh`);
  no severe errors.
- **TUI suite:** `make tui-smoke` — all five scenes dump finite state;
  snapshots are bit-deterministic across runs.

## 3. Physics truth ledger (this release)

- Verlet-exact free flight (contact-gated), post-integration Poisson gate,
  CCD swept-TOI pre-clamp with symmetric two-phase application and
  remainder-only integration, translation-gated floor TOI, strict
  both-side warm-start with frame-consistent friction memory, split-impulse
  (no velocity Baumgarte), Hertz-patch rolling + spin, gyroscopic torque.
- No velocity clamps, no restitution caps, no hysteresis. Game-only knobs
  stay labeled: `nice_value` (default 0), `angular_damping_scale`
  (default 1.0 = vacuum), linear-viscous `world.drag` (1.0 = vacuum).
- Sleep truth per §1; `sleep.enable = 0` runs sleepless validation.
- Friction policy (measured, all three variants A/B-tested): normal +
  primary tangent restored from cache, second disc tangent always cold;
  blind t2 restore toppled stacks, per-tick zeroing under-held creep.
- `driven_wheel` moved into the resolvable spin regime (0.080 N·m) with
  load-bearing gates: grounded height, rolling coupling 0.20–1.15, spin
  cap. (π-rad/tick contact is undefined for any discrete scheme — the old
  torque "passed" at y=488 lunacy.)
- Determinism: fixed 1/60 s step, fixed iteration order, exact-IEEE +
  fixed-coefficient transcendentals, `-ffp-contract=off`; `determinism`
  twins agree bitwise; TUI snapshots are byte-identical run to run.

## 4. Joint support matrix (honest)

| Type | Solves | In-engine creation UI | Persists in v200 |
|---|---|---|---|
| Spring | ✅ Hooke + damping + Courant guard | ✅ object menu link | ✅ |
| Revolute (+limits, motor, drift fix) | ✅ | via scene/terminal flows | ✅ |
| Fixed / distance | ✅ | ❌ | ❌ |
| Prismatic (+limits, motor) | ✅ | ❌ | ❌ |
| Rope (pull-only) | ✅ | ❌ | ❌ |

All six live in the headless suite and the `mpe-tui --scene demo` world.
UI + persistence for the last four rows is post-release work.

## 5. New: terminal debugger (`mpe-tui`)

Live ncurses inspector (overview / object+math / joints / scene-graph /
help) plus deterministic pipeable dumps (`--snapshot`, `--stream`,
`--scene demo|tower|pendulum|springlab|f10`). Needs only ncurses. It was
built as a debugger and ended up convicting the F10 bug — it is now part
of the release ritual (`make tui-smoke`).

## 6. Configuration

78 tunables, 13 categories, live menu (key `6`), terminal `env`/`export`,
`status/engine.cfg` round-trip, corrupt-safe load. F11 torture guardrails
unchanged. Config file and v200 scenes load forward; ≤v153 scenes load via
the legacy reader. Behaviour at defaults matches v14S except the
documented truth exceptions (vacuum angular damping, removed clamps).

## 7. Known limitations (carried, documented)

- Wayland: mouse lock needs X11 (`GDK_BACKEND=x11` or an X11 session).
- Windows/macOS: unsupported.
- No SIMD; single-threaded solve; ~1136-object practical wall.
- Linear-only CCD sweep; rotation-heavy sub-tick motion is the solver's job.
- Joint UI/persistence scope per §4. Big-endian hosts untested (LE format).
- GTK4 port planned separately (render path verified portable; mouse-look
  needs an input redesign — no action in this release).

## 8. Release ritual

```bash
cd v15R3/src && make clean && make        # engine
python3 ../../tools/test_runner.py        # 29 headless, expect 29 green
make tui-smoke                            # all TUI scenes finite
./engine                                  # F5–F11 in-engine matrix
```

Or `run_all.sh` from the repo root (build + headless suite).
