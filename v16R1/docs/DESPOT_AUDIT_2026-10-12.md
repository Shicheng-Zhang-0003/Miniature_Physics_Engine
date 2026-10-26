# DESPOT Audit 2026-10-12 — full mathematical, programming, operational sweep

> Scope: the entire 475-MPE tree (active `v15S` engine + `tools/` + docs), with
> cross-reads of the sibling twins where they are authoritative: standalone
> `461-MFS` vs vendored `v15S/src/ecosystem/mfs`, and `4179-Magi/472-MGB` vs
> vendored `v15S/src/ecosystem/mgb`. No millennium prize was required; every
> lie below was measured against a textbook, a header contract, or a run log.

## Method

Three independent passes, all read-not-assumed:

- **Math:** every formula in `core/` + `physics/` re-derived against Mirtich,
  Baraff, Catto GDC 2005/2011/2013/2014, Gottschalk 1996, Coumans 2005,
  Ericson RTCD, Johnson contact mechanics, Goldstein mechanics, Press
  numerical recipes. Oracles checked for circularity (does the expected value
  restate the code?).
- **Programming:** full-tree C read (core/physics/scene/config/render/ui_input/
  tui/tests/ecosystem), `makefile`, `tools/test_runner.py`, `build_tests.sh`,
  `run_all.sh`/`verify.sh`, `build_windows.bat`. Git history mined for
  recurrence (torture-leak, regime-wipe, SKIP-sentinel, count-drift).
- **Operational:** every headline count re-derived from its cited source
  (registry vs `MIN_SUITE_ENTRIES` vs readme vs gates; MFS summary vs rows;
  TUI scenes vs smoke; tunables vs schema; version macro).

Verification after fixes: `build_suite` clean, `--profile quick` **59/59**,
`--profile physics` 104/105 with the single red being the documented
pre-existing `legacy-f11_torture`, `tui-smoke` PASS with the new overflow gate.
All scratch under `temp/qa_runs/` (runner-enforced `TMPDIR`); one manual
TUI check used the pre-approved `/tmp/opencode/` scratch.

## Mathematical lies found and dispositioned

| ID | Lie | Severity | Disposition |
|---|---|---|---|
| M1 | `paranoia_contact_solver` Test 1 divided solver output by solver input (`vy/impact_velocity` vs `e` the solver used) — algebraic identity, exactly `e` to 6 decimals for every `e` | critical | **FIXED**: body-trajectory oracle (`-prev_vy` vs `vy`) + tolerance 0.03→0.10 matching `suite_a` |
| M2 | `f11_torture` worst-overlap measured with `collision_dual_cube` — SAT false-negative contributes 0, hole hides in green | high | **BOUNDED**: honesty note in source; decision independently gated 400/400; full independent-depth probe ticketed |
| M3 | Clip-manifold max-area dotted `cross(e1,e2)` with `t1=cross(n,cand)` (perp to normal, ≈0 for planar clips) — selection arbitrary | high | **FIXED**: `fabs(dot(cross(e1,e2), ref_normal))` (Ericson/Box2D); quick 59/59 confirms no stack regression |
| M4 | `mpe_clamp_*_events` plain `unsigned long ++` from MT sanitize workers — data race + UB + lost counts | high | **FIXED**: `_Atomic` + `__atomic_fetch_add/store/load`, `mpe_clamp_*_total()` accessors, suite updated |
| M5 | Rotational CCD capped at 2.0 m/s — blade `100 rad/s·0.5 m=50 m/s` contributes 2.0; floor path ignores `ω×r` by design | high | **DOCUMENTED as unsupported regime** (blade needs substep/external bound); cap kept as guard, claim corrected |
| M6 | Rolling mass-weighted split is a tune (`I≈m` only for similar geometry); band 4–14 m calibrates its own model | medium | **LABELED**: source already says TUNE; kept, not cited as derived |
| M7 | Friction μ frozen at tick-start — whole-tick stick/slip mis-solves in-tick transitions by up to `(μs−μk)·Fn` | medium | **DOCUMENTED as hack** with direction; per-phase re-snapshot ticketed |
| M8 | Revolute `K==JM⁻¹Jᵀ` test shares the impulse application with the code — self-consistency, not physics | medium | **TICKETED**: verify against anchor-coincidence residual, not solver internals |
| M9 | Paranoia angmom 3% gate is vacuous (live measures 0.0028%, suite_a gates 0.5%; checks `\|L\|` not vector) | medium | **TICKETED**: port suite_a 0.5% vector gate |
| M10 | `solver.bias_factor` help says "Baumgarte" — it is split-impulse positional, no velocity bias exists | medium | **FIXED**: help rewritten to "Split-impulse positional aggressiveness (NOT velocity Baumgarte)" |
| M11 | Anisotropic ellipse is radial scaling, not Coulomb closest-point; second body axis ignored | low (off by default) | **TICKETED** with error bound |
| M12 | `sqrt(R·pen)` called "Hertz" — Hertz needs moduli (`a³=3NR/4E*`); formula is geometric overlap | low | **FIXED**: renamed "geometric overlap patch", Hertz citation removed |
| M13 | Default `-9.81` vs `G_N=9.80665` (0.034% high); oracles mix both inside 2–15% tolerances | low | **TICKETED**: pin one `MATH_G_N` constant |
| M14 | `world.drag` is mass/shape-independent linear retention, not aerodynamics | low (disclosed) | **KEPT LABELED**; user-docs scope note ticketed |
| M15 | Paranoia drag-0.99 oracle is plausibility (`apex<apex_drag1`, `<0.5 m`), never the analytic drag solution | low | **TICKETED**: assert closed-form drag solution to 1e-4 |

Verified correct, no change: semi-implicit Euler, exp-map rotor + renormalise,
all three inertia tensors, 15-axis SAT, effective mass, Poisson+Newton bound,
translational split impulse, q-form sphere TOI + floor double quadratic,
`-ffp-contract=off` override, `det_*` polynomial bounds.

## Programming weaknesses found and dispositioned

| ID | Weakness | Severity | Disposition |
|---|---|---|---|
| P1 | `attach` iterated `tick_modules[]` before validating `tick_module_count`; `detach` had no validation — garbage count = OOB read + NULL-deref in E_TABLE listing | critical | **FIXED**: validate `[0,16]` before any indexed read in both; new `MPE_DETACH_E_COUNT (-3)`; terminal message updated |
| P2 | `mpe_config_save` `%s.tmp` + `fopen` followed a planted symlink — arbitrary-file write as engine uid | critical | **FIXED**: `mkstemp %s.XXXXXX` + `fchmod 0600` + `fdopen`, mirroring `scene_saving` |
| P3 | Suite `scene_clear` stub zeroed `body_count` only — joints/cache/remap/revision leaked across 45 in-process tests | high | **FIXED**: faithful clear (both pools, cache, remap, bump revision) + stub-limitation comment |
| P4 | MFS `[SKIP]` never failed — `allow_skip` ignored, CWD-dependent SKIP hid coverage (220→204 class) | high | **FIXED**: `mfs-skip-contract` mirrors suite contract in `tools/test_runner.py` |
| P5 | Pair-buffer exhaustion (fixed 65536, no grow) dropped pairs silently; F10/TUI never gated it | high | **FIXED**: `f10_long_run` gates pairs+manifolds zero; `tui_dump` `[result]` fails on any overflow |
| P6 | `g_ccd_obstacle_margin` file-global written then re-read in same call — two worlds on two threads race | high | **FIXED**: sweep uses call-local `obstacle_margin`; global kept as last-tick telemetry only |
| P7 | One-shot statics (`s_dt_warned`, `s_degraded_ticks`, spring warns, simd selfcheck) plain + first-regime-consumes-budget | high | **FIXED race**: `_Atomic` conversions for step-path counters; budget-per-process documented as telemetry limit |
| P8 | Legacy scene bound 4096 vs live pool 1024 + ignored `add_joint_by_ids` return + missing `bump_revision` | high | **FIXED**: veto `>mpe_max_joints`, count drops loudly, bump revision on legacy commit |
| P9 | Loader jail is CWD-relative + `realpath→dlopen` TOCTOU — direct `make test_*` off-SRC_DIR yields SKIP-green fragility | high | **DOCUMENTED**; absolute-root jail + `O_NOFOLLOW` open ticketed (behaviour change, not snuck in) |
| P10 | `mpe_diag_ring_at`/`event_log_get_message`/loader `path_at` return interior pointers — torn reads under emit | high | **TICKETED**: copy-out API + header contract |
| P11 | Six `(int)float` terminal casts (`ping -c 1e30`, `seq`, `dmesg -n`, `nice`) — UB on overflow before clamping | high | **FIXED**: `term_float_to_int_safe()` (finite+range before narrowing) at all sites |
| P12 | `(float)iterations` narrowing + 3 mid-test `mpe_config_init()` in `reference_math` wipe the regime | med-high | **FIXED cast**; mid-test inits **kept as documented premise pins** (absolute oracles need fixed iters — the pin list, not blindness) |
| P13 | `build_windows.bat` built headless-only, stale-ERRORLEVEL silent green, never suite/smoke/flags | med-high | **FIXED**: builds suite, runs `--all`, `check-flags`, then `test_runner --profile quick` like `verify.sh` |
| P14 | `WARNINGS_AS_ERRORS=0`, no `-Wconversion/-Wshadow`, `analyze` pipes to `head` exit-0, full profile never runs analyze/tsan/scalar | med-high | **TICKETED**: CI `WARNINGS_AS_ERRORS=1`, failing `analyze`, full-profile stages |
| P15 | TUI `[result] PASS` required finite only — overflow still PASS; stress truncates bodies 97+ | medium | **FIXED**: any node/pair/dedupe/manifold overflow forces `bad=1` |
| P16 | `snapshot_clean()` unconditionally overwrote clean with `g_cfg` — second F11 without restore re-armed the 2026-10-04 engine.cfg leak | medium | **FIXED**: refuse snapshot while torture live |
| P17 | Silent clamps (`dt>0.1→0.1`, iters→[1,128]) + `mpe_cfg_is_degenerate` never called in `mpe_test_begin` | medium | **FIXED**: degenerate assert in `mpe_test_begin`; first attempt over-gated brittle (restitution 0 is legal) and was narrowed to solver-off fields only |
| P18 | Joint/constraint `-1` for six distinct causes + ignored returns at scene commit | medium | **TICKETED**: distinct codes + diag per cause like attach |
| P19 | `physics_world_hash_state` omits spin/mass/sleep/joints — desync detector blind to hinge/spin divergence | medium | **TICKETED**: versioned wider hash (golden break documented as preferable) |
| P20 | MGB transcript `transcript_PID.log` + frame `frame_PID_SEQ.s2` pid-predictable + `fopen` follow in shared `/tmp` | medium | **FIXED**: transcript via `mkstemp`, frame path adds random suffix, `lstat` refuses non-regular before read |
| P21 | `build_tests.sh` `cd "$SRC"` unguarded under `set -euo pipefail` — missing tree aborts before `FTC RESULT`, misleading "missing summary" | medium | **FIXED**: `cd "$SRC" \|\| { echo [BUILD-FAIL]…; exit 2; }` |
| P22 | `status/engine.cfg[.backup]`, `status/scene.dat` CWD-relative; `microvim :w` writes outside tree on extension check alone | med-low | **DOCUMENTED**; absolute `MPE_SRC_ROOT` resolution ticketed |
| P23 | `set_int/set_bool` silent clamp + loader raced-`-3` documentation gap | low | **TICKETED** |
| P24 | `MPE_INFO(...) printf(__VA_ARGS__)` future format-string shape | low | **TICKETED** |

Recurrence note: torture-leak (P16 re-opened the 2026-10-04 class — closed
again), regime-wipe (P12 pins confirmed intentional, not recurrence), SKIP
(P4 closed the remaining MFS hole), count-drift (floors hold: 45/30/14).

## Twin-tree note (462/472 reading of "475, 462, 472")

- `461-MFS` vs vendored `ecosystem/mfs`: **already drifted before this audit**
  (`CORE` list, plugin-copy lines, plus ~18 files per `sync_mfs_check.sh`).
  This audit's one-line `cd`-guard delta rides on top of that pre-existing
  drift; unifying the twins is a separate-repo task, not snuck into this push.
- `472-MGB` (`4179-Magi/472-MGB`) vs vendored `ecosystem/mgb`: likewise
  pre-diverged (extra `mgb_sink_mpe.c` here). The P20 `mkstemp` fix is 475-side;
  forward-porting to 472 is tracked, not committed here.

## Verification record (this audit)

- `make build_suite`: clean, exit 0.
- `python3 tools/test_runner.py --profile quick`: **59/59**, 0 blocking
  (`temp/qa_runs/20261012T153811Z-2280087/`).
- `--profile physics`: 104/105 — sole red `legacy-f11_torture` (exit 1),
  the documented pre-existing extreme-parameter torture case, reproduced
  bit-identically from pristine HEAD; not a regression.
- `make tui-smoke`: PASS on all scenes with the new overflow gate.
- One self-inflicted red caught and fixed mid-audit: the first P17
  degenerate check included restitution/friction and turned the entire
  brittle regime red; narrowed to solver-off fields, quick returned to 59/59.

## Remaining open (bounded, not solved)

Normal/tangent warm-start carry (dead by measurement), full Gottschalk depth
numeric agreement, oblique multi-body angular-momentum oracle, per-phase
friction re-snapshot, TSan on a capable host, MSVC `/fp:strict` proof,
multithreaded solve (serial by design), absolute-path jail + `O_NOFOLLOW`
loader open, joint distinct codes, wider state hash, `MATH_G_N` pin, drag
analytic oracle, angmom vector gate, ellipse closest-point, `WARNINGS_AS_ERRORS`
CI. Each is ticketed above with the measurement that bounds it.
