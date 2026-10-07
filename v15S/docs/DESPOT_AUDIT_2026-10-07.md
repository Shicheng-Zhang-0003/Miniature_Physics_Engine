# Despot Audit 2026-10-07 — Math, Programming, Operational

Scope: full `v15S` tree (kernel, physics, render, scene, config, ui_input,
tui, plugins, ecosystem/mfs), `tools/test_runner.py`, validation scripts,
all user docs. Method: three independent audits (math/physics,
programming, operational) + external closed-form oracles + baseline
`--profile quick` 58/58 green + `test_mpe_suite --all` 44/42 green.
Scratch and probes under `temp/despot-20261007/` and `/tmp/opencode/` only.
No file outside the project was touched.

Commit style in this repo: `DD/MM/YY, HHMMSS: type: subject`
(types: fix/docs/test/build/style). This audit's fixes follow it.

## A. Mathematical truth — verified and fixed

External authorities (not the code): CGPM 1901 Dec.2 / CODATA 2022
(`g_n=9.80665` exact by convention), ISO 2533:1975 (1.225 kg/m3),
Mirtich, Baraff, Anitescu, Catto GDC 2005/2011/2013 + Solver2D 2024,
Gottschalk 1996, Ericson RTCD, Coumans/Schmidl, Stronge (Poisson),
Kuipers/Diebel (quaternion exp-map), Press (stable quadratic),
Goldstein/Hibbeler (inertias), Hairer (symplectic Euler).

Independent oracle `v15S/validation/ext_oracle_check.py` (mirrored at
`temp/despot-20261007/oracle_check.py` during the audit): 10/10 green
(free-fall, range, 3 inertias, Coulomb stop, bounce series, pendulum,
spring, stable-q TOI, min-vs-sqrt friction note).

Verified-correct (read + measured): semi-implicit Euler
(`physics_world.c` velocity-then-position + `rigidbody.c` symplectic gate),
quaternion exp-map rotor + renormalize, all three inertias
(sphere 2/5mr2, box m/12(h2+d2), cylinder 1/2mr2 + m/12(3r2+L2)),
SAT-15 + S-H clip `t=d1/(d1-d2)`, sequential impulse `λ=-vn·m`,
Poisson `e·C` + Newton bound + impact refresh, translational split
impulse (no velocity touch), q-form TOI + floor quadratic + slab sweep +
symmetric two-phase + analytic chaining.

Fixed:

1. **LIE-04 degraded free-flight double-apply (P1).**
   `physics_world.c` + `simulation_physics_loop.c` defaulted
   contact-free TRUE when `has_contact==NULL`, then treated post-force
   `v_post` as `v(0)` in the analytic path (v error `g·dt` ~0.08 m/s).
   Now: contact-free FALSE unless `tick_v0` restore actually ran;
   degraded ticks take safe symplectic. Both step paths.
2. **LIE-05 variable-dt silence (P1).** `physics_world_step()` accepted any
   `dt∈(0,0.1]`. Now loud once on non-1/60 dt (canonical 0.016667);
   twins must pin identical dt. GUI loop already fixed 1/60.
3. **LIE-03 stale L-revert comment (P1).** `rigidbody.c:999-1029` claimed
   L-conservation "Reverted; not in tree / 48% drift" while `:917-974`
   ships it at 0.0028%. Marked stale, kept for forensics; behaviour gate
   (`|L|` drift) is the warrant, not presence.
4. **LIE-07 false convexity proof (P1).** `collision_cylinder.c:486-492`
   claimed min-of-convex stays convex (false; max does). Replaced with
   distance-to-convex-set argument (Ericson). Result held, proof misled.
5. **LIE-10 min-vs-sqrt silence (P2).** `collision_solver.c` combined
   `min(μa,μb)` vs Box2D `√(μaμb)` (42% lower: 0.30 vs 0.52). Now
   documented at the site; thresholds pinned to `min`.
6. **LIE-11 universal contraction (P2).** "Converges for any inertia ratio"
   -> "converges for tested tumblers; falls through otherwise".
7. **LIE-12 rolling-split tune (P2).** Mass-weighted share labelled TUNE
   with `rolling_decay` band as sole warrant, not derived.
8. **LIE-13 float pi in double paths (P2).** `degrad/raddeg` now double via
   `MATH_PI`; `degrad_f/raddeg_f` keep float ABI.
9. **LIE-02 hits telemetry (P0).** Added `contact_cache_hits_applied`
   (per-world, reset in `contact_cache_stats_reset` + `physics_world_clear`).
   `hits` = hash coincidence; `hits_applied` = seed survived cone.
   Today `hits>0, applied==0` — the dead-warm-start proof. Gate applied.
10. **LIE-01 warm-start headline (P0, docs).** Engine has NO working warm
    start on any row (normal zeroed `:382`, tangent cone-killed with fn=0;
    measured `0.154651→0`). Needs 96-128 iters vs Catto 4-8. Code comment
    already honest; `readme.md` now says so in the solver section and the
    head truth line no longer claims it.
11. **LIE-06 MSVC scope (P1, docs).** `det_math.h` now scopes bitwise twins
    to GCC/Clang + `-ffp-contract=off`; MSVC needs `/fp:strict` (unproven).
12. **LIE-08 Hooke purity (P1, docs+test discipline).** Spring saturation
    (`|F|≤m_red·200` + Courant `k_stable`) already loud; tests must set
    `joints.max_acceleration=10000` + `k<k_stable` + assert no warn to
    measure Hooke, not the limiter.
13. **LIE-09 spinning-obstacle CCD hole (P1, docs).** Mover-only `|w|·R`;
    obstacle rotation unswept. Now documented in
    `collision_mechanics.h:338-347` as unsupported blade regime.

Open (declared, not solved): F11 buckling transient (~0.69 m chaotic),
normal-seeding formulation with stability proof, rotational CCD,
thread-sanitizer run, MSVC flag proof.

## B. Programming weakness — fixed

1. **P0-1 `physics_world_clear()` stale joints.** Zeroed counts but not
   `is_active`; solvers loop `is_active`, so cleared non-primary worlds
   kept constraining recycled IDs. Now zeroes both pools. Primary was
   safe via `scene_clear()`; headless/multi-world was not.
2. **P0-2 cached dlsym use-after-dlclose.** `gamepad_drive.c` latched
   `s_fleet_get/s_mecanum/...` with no unload invalidation; next tick
   after `mod unload` called unmapped code. Now re-resolves EVERY tick
   (NULL = absent) + `gamepad_drive_invalidate()` for unload hooks.
3. **P0-3 microvim jail escape.** Whitelist allowed any relative `..` path
   with matching ext (`vi ../../tmp/evil.c` passed, `:w` truncated).
   Now rejects `..` except exact blessed known-files; save checks
   fprintf/fflush/fsync/fclose (was unchecked) + `unistd.h`.
4. **P0-4 `mount` arbitrary read.** `cmd_mount` passed any absolute path to
   `scene_loading()->fopen()`. Now confined (no absolute, no `..`);
   status scenes still load.
5. **P1-1 plugin manifold NaN.** `a3_sanitize_plugin_manifold()` checked
   count+normal only; NaN penetration/1e30 position passed to solver.
   Now validates penetration range, position finite+boxed, locals, ra/rb,
   tangents; zero-counts on violation.
6. **P1-2 init memset-before-cleanup leak.** Zero-first orphaned loader
   attachments (unload `-2` forever). Now: address-only liveness check
   BEFORE memset, cleanup first if live, then zero. 64-world cap overflow
   now loud stderr instead of silent invisibility.
7. **P1-3 per-world leakage.** `add_sphere/cube/cylinder` re-stamp
   friction/restitution from owning `world->cfg` (was global `g_cfg`
   always). Step already per-world; construction now matches.
8. **P1-4 durability.** `term_tee` now fsyncs file + dir-syncs rename
   (was jail-correct but crash-lossy). Microvim save fsyncs (see P0-3).
9. **P1-5 registry brittleness (docs).** 4-slot rotating snapshot +
   interior-pointer invariants documented as footguns; probe `fopen`
   noted. No API break; callers warned.
10. **P1-6 degraded-tick silence.** Low-mem guard now covers
    island/ccd/has_contact/tick_v0 + counts degraded ticks loudly
    (first 8 + every 1000th). Twins with different OOM histories cannot
    diverge silently.
11. **P2-1 no analyzer/sanitizer targets.** Added `make analyze`
    (`-fanalyzer`) + `make asan` (`-fsanitize=address,undefined`).
    CI should set `WARNINGS_AS_ERRORS=1`.
12. **P2-2 CRC race + coverage.** `crc_table` now `pthread_once` (was
    racy lazy). `physics_world_hash_state()` documents exact blind
    classes (spin/mass/sleep/joints/revision omitted; translation
    lockstep only).
13. **P2-3 gamepad probing (docs).** `/dev` path, single-digit index,
    5 s retry, trigger heuristic noted as fragility; whitelist + backoff
    ticketed (annoyance, not corruption).

## C. Operational truth — fixed

Disk truth: latest full runs `total=234 pass=232 fail=0 xfail=2 info=6`.
Both xfails are ONE frontier (MFS-STRAFE-F2 phys 0.63 vs odom 0.91) ×
plain+sanitizer runs; MOTOR-III −61.3% is `[info]`, not xfail.

1. `234/234` -> `232 + 2 xfail (total 234)` in `readme.md`,
   `v15S/RELEASE_GATES.md`, `v15S/evolution.txt`, `docs/VALIDATION.md`.
2. "Two markers" -> "one frontier ×2 runs" in `readme.md`.
3. `tools/test_runner.py:6` docstring `41` -> `44`.
4. Regime matrix honesty: `readme.md` + `docs/VALIDATION.md` now state
   13 premise pins run identical config by design (oracle measures its
   law, not the regime); 42/42 claim scoped to unpinned dimensions.
   Full pin list in `docs/VALIDATION.md` § Regime pins (this audit).
5. CWD/shared-temp: `makefile` no longer clobbers runner `TMPDIR`
   (only defaults when unset); `mpe_suite_c.c` + `scene_roundtrip_test.c`
   honour `$TMPDIR` for `.dat` files (concurrent runs isolated).
   Plugin CWD=`v15S/src` requirement remains (documented fragility).
6. Headless-deps line corrected: bare core is libc-only, full ritual
   needs GTK4/epoxy/ncurses; F5–F8 need a live window (headless
   equivalents stand in where noted).
7. MFS counts: `15 inner (5 outer wrappers)` everywhere; `14` retired.
8. `temp/` stale logs noted; 22/22 ext-truth battery remains gitignored
   (unfalsifiable from clone) — this audit's 10/10 oracle IS vendored at
   `v15S/validation/ext_oracle_check.py` (run: `python3
   v15S/validation/ext_oracle_check.py`).
9. `make test_suite` now prints `42 physics only; use test_suite_all /
   runner for 44`. `make analyze`/`make asan` added.
10. This file supersedes counts in `DESPOT_AUDIT_2026-10-01.md`
    (41/42 era); that file is history, not live TODO.

## Verification record (this audit)

- `make -C v15S/src build_suite`: clean (one `-Wcomment` fixed).
- `v15S/src/test_mpe_suite --all` (CWD=`v15S/src`): 44/42 green, 0 fail.
- `python3 tools/test_runner.py --profile quick`: 58 checks, 58 pass,
  0 fail, 2 info, 0 blocking (run `temp/qa_runs/20261007T145327Z-48519/`).
- `temp/despot-20261007/oracle_check.py`: 10/10 green (vendored at
  `v15S/validation/ext_oracle_check.py`).
- Full `--profile full` not re-run here (needs GTK4+TUI+sanitizers in one
  shot); quick + canonical + oracle are the gates this audit touched.
  Runner's last full on disk remains `232+2 xfail/234`.

## Regime pins (which dimensions are toothless by design)

`mpe_test_begin()` applies `MPE_TEST_REGIME` mults; post-begin overwrites
erase it. Pinned (identical config ×5 regimes, oracle measures its law):
gravity (`mpe_suite_a.c` projectile/friction/incline/pendulum/bounce +
`mpe_suite_b.c` reference blocks + `mpe_suite_c.c` f10/f11), iterations
(stack/convergence-adjacent + reference blocks), sleep (settle/sleep pins),
friction (stop/hold/rolling pins), plus 4 full `mpe_config_init()` wipes
(`mpe_suite_b.c:734,786,821`, `mpe_suite_c.c:454`). Teeth live ONLY on
unpinned dimensions (e.g. drag, restitution-scale where not pinned).
A defect in regime application itself is invisible in pinned tests —
narrow the 42/42×5 claim accordingly (done in readme/VALIDATION).
