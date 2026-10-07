# Remaining Work

> Live TODO. Dated history below is preserved; current counts/gates are in
> `docs/DESPOT_AUDIT_2026-10-07.md` (44/42; 232+2 xfail/234). The 2026-10-01
> audit's 41/42 denominators are superseded.

## Despot 2026-10-07 — closed this audit

- [x] P0-1 world-clear joint `is_active`; P0-2 dlsym re-resolve + invalidate;
  P0-3 microvim `..` jail + fsync save; P0-4 mount confinement.
- [x] P1-1 plugin manifold full validation; P1-2 init order + 64-cap loud;
  P1-3 per-world material stamp; P1-4 tee/microvim fsync; P1-6 degraded-tick
  guard + counter; P2-2 CRC `pthread_once` + blind-class docs.
- [x] LIE-02 `hits_applied`; LIE-03 stale L comment; LIE-04 free-flight guard
  (both step paths); LIE-05 non-1/60 loud; LIE-06 MSVC scope; LIE-07 convex
  proof; LIE-10 min-vs-sqrt; LIE-11 contraction scope; LIE-12 rolling tune
  label; LIE-13 double pi; LIE-09 blade regime docs.
- [x] Ops: 234/234->232+2 xfail, two-markers->one frontier, docstring 41->44,
  TMPDIR isolation (`makefile` + suite `.dat`), headless-deps line, MFS
  15-inner/5-outer, `test_suite` 42-only warning, `make analyze`/`make asan`,
  vendored `temp/despot-20261007/oracle_check.py` (10/10).
- [x] Verified: `build_suite` clean, `test_mpe_suite --all` 44/42 green
  (CWD=`v15S/src`), `--profile quick` 58/58 green, oracle 10/10 green.

## Verification suite upgrade (2026-09-25)

- [x] Unified runner profiles: `quick`, `physics`, and `full`; dynamic C and
  Makefile test discovery; strict result accounting; project-local per-run logs,
  JSON summaries, JUnit reports, and preserved TUI snapshots.
- [x] Runner contract tests for registries, result parsing, MFS summaries, TUI
  snapshot validation, command-launch failure reporting, and report generation
  (17 Python tests).
- [x] Fixed-seed matrix inverse property sweep: 256 SPD matrices over scales
  from 2^-24 to 2^24, plus singular-axis and non-finite input gates.
- [x] Full profile passes all canonical, isolated legacy, paranoia, MFS, TUI,
  and engine build checks; the C, MFS, and TUI test groups also pass combined
  ASan/UBSan. **Count drift (2026-09-29):** the historical "220/220" figure no
  longer matched what the runner emitted. **CLOSED 2026-10-03:** the runner now
  reports **234 checks** and `MIN_SUITE_ENTRIES` is pinned at 44 (the exact
  registry size), so coverage can no longer be deleted silently. Worth recording
  how this actually went: the floor was added on 2026-09-29 and pinned at 42
  while the registry held 44, so it was blind for the two newest gates -- the
  same failure it was created to prevent, reproduced inside a week. A floor that
  sits below the real count is worse than no floor, because it reads as
  protection.
- [x] Added missing `physics_world_cleanup` calls to cylinder-drop, driven-wheel,
  and FTC integration test paths after LeakSanitizer identified fixture leaks.
- [x] Quick profile rerun after adding command-launch failure handling; all
  17 harness contracts and canonical cases pass.
- [x] External-truth battery 2026-10-02: 22 closed-form checks
  (free-fall, projectile, bounce heights, pendulum, spring, elastic exchange,
  Coulomb stop/hold/slide, 3 inertias via torque, tower, range linearity,
  Galileo, restitution threshold) green in C plus a second Python oracle
  implementation green; harness bugs caught and corrected; poisoned live
  config neutralized; see `docs/DESPOT_AUDIT_2026-10-01.md` §F.
- [ ] Uninitialized joint pools (P2 hardening, proposed 2026-10-02):
  `physics_world_init` zeroes the joint count but not `is_active` flags —
  every in-tree caller pairs it with `constraint_pool_init`, so nothing live
  is affected, but init-alone callers get phantom joints. Clear the pools
  inside init (one loop, zero behavior change for paired callers).
- [ ] Expand seeded property coverage from matrix inversion to collision,
  constraint, and configuration invariants; add differential oracles across
  multiple analytic systems and a ThreadSanitizer run.

## Fixed in 2026-09-23 despot audit (verified: engine + mpe-tui + 27/30 headless green, same 3 pre-existing FAILs as clean HEAD: stack, driven_wheel, list4_cylinder_floor)

- [x] det_reduce_pi4 quadrant correction (+1/+3, was +2), single-count trig fallback, lazy sin/cos branch.
- [x] det_pow_retention chunking (no libm desync in-tick), CCD floor linear fallback + double quadratic, Poisson gate sign-enforced.
- [x] Sphere-cube degenerate normal (face normal, was -Y), cylinder-inside contact point on surface, gyro stability cap, broadphase linear clamp 10m + stratified double probe.
- [x] Revolute 6x6 in double with equilibration; quat sanitize + quat->mat double parity; lookAt re-orthogonalized.
- [x] Registry owns pair/module strings (no .so rodata dangle); register_broadphase/solver return int; capsule destructor + NULL/slop-per-world; loader clears primary stage pointers on unload.
- [x] scene_saving NULL/type-bounds; scene_load full-float + duplicate-ID veto, legacy empty-scene, custom identity preserved (100); config localtime_r + truncation consume + 0700 dirs; tui_dump NULL/cache; tui header/ delwin/auto-config; terminal capture/alias hardening; term type mismatch repaired (engine builds).
- [x] test_runner --include-paranoia + exact-match; makefile installcheck grep, help, clean, module target, .PHONY, install plugins + uninstall, check-deps-tui, native hard-fail; .gitignore mpe-tui; docs (joints persist all 5, historical suite count 28+3, compat versions, freeze, version string, BE, Wayland-P1, native, V04 run-max, V03 F10/F11, deps).
- [x] Pre-existing FAILs confirmed on clean HEAD (not regressions): stack, driven_wheel, list4_cylinder_floor. fix_log contradiction corrected; run_all.sh and verify.sh keep logs under the project temp directory.

## High priority (still open)

- [ ] Continue migrating the remaining paranoia tests to canonical builders and tighten any weak gates.
- [x] Diagnose and fix cylinder collision and sleep/depenetration failures with
  minimal reproducible cases. *2026-09-29: two cylinder defects found and fixed
  -- (a) the static-slab fast path was gated only laterally, so a cylinder
  anywhere beneath a static box reported unbounded phantom penetration and was
  levitated at +0.1033 m/tick with gravity cancelled until it tunnelled through
  and slept on top; (b) the cylinder/sphere INSIDE branch negated its normal,
  driving an enclosed sphere deeper. Both now have regression tests proven to
  fail when the fix is reverted. See src/ecosystem/mfs/docs/KNOWN_FAILURES.md.*
- [ ] Deep-overlap ejection route for an enclosed sphere is unstable (open): 3
  of 4 sub-cases resolve correctly, one (sphere at +Y) ends up deeper because
  the pair's centre of mass is driven across the axis first. Narrowphase is
  correct in all four; this is a depenetration/split-impulse robustness issue.
- [x] Plugin load/attach/unload + stage-backend lifetime regression test
  (`loader_lifecycle` in Suite v2: real capsule .so, busy -2, purge, reload).
- [x] **Mouse lock broken on Wayland since the first playable release — ROOT-CAUSED
  AND FIXED (2026-09-29, `ui_input/mouse_lock.c`).** See
  `src/ecosystem/mfs/docs/KNOWN_FAILURES.md` → `REL-PTR-2026-09-29` for the full analysis. Summary:
  "lock" only ever *hid the cursor*; camera deltas came from absolute cursor
  position (`dx = x - last_x`). On X11 that was rescued by `XWarpPointer`
  re-centring, which GTK4 broke by dropping the GTK3-only `GDK_WINDOWING_X11`
  guard macro — so **v15S mouse lock was broken on X11 and Wayland both**.
  On Wayland warping is protocol-forbidden, so the cursor physically ran to the
  screen edge, the compositor clipped the motion, and the camera stalled; once
  the pointer left the surface no events arrived at all, so it could never
  re-lock. Fix: real lock via the `zwp_relative_pointer_manager_v1`
  relative-pointer protocol (wayland-scanner generated, wired into the
  makefile), which reports unbounded unclipped deltas and never moves the
  cursor; `on_mouse_movements()` now prefers it and ignores absolute
  coordinates. X11 warp restored behind an explicit define. Falls back to the
  old edge-limited path if the compositor lacks the protocol. Verified: the
  live compositor advertises the global (probe), engine builds warning-free,
  suite 34/34, full profile 206/206.

- [x] **GTK3/GTK4 duplication in `ui_input/` — DONE (2026-09-29).** 6,832 lines
  deleted across 18 files; the whole directory had carried a full duplicate body
  for every handler. This was the root cause of the `e_key_pressed` latch existing
  in two copies (a one-line fix that had to be made twice). Removed the GTK3
  halves and the now-vacuous `#ifdef MPE_GTK4` guards, so the code compiles
  unconditionally. `make GTK_PKG=gtk+-3.0` now `$(error)`s instead of silently
  offering a build that cannot link. Verified: engine builds clean with no
  warnings, suite 34/34, full profile 206/206.
- [ ] Replace the duplicate GUI and headless physics pipelines with one canonical step path.
  Evidence 2026-09-28 (despot audit, harness at /tmp/opencode/path_equiv.c):
  passive 6-cube tower, 600 ticks, 64 iters — legacy `simulation_physics_tick`
  vs canonical `physics_world_step` agree BITWISE (max deviation 0.00000 m all
  bodies). Driven-robot runs differ ~2.7% over 180 ticks via motor-observer
  feedback amplification (expected, not a path bug). Unification stays a
  maintenance item, not a correctness bug.
- [x] [MFS-STRAFE-F1 FIXED 2026-09-28] Mecanum strafe now transmits 3.40 m
  (was ~0.01 m vs 0.30 m gated) via analytic roller-kinematics lateral force
  at each wheel contact with rollers removed from the solver (no chassis-force
  cheat; verified by grep). F1 hard-gates. [F2 FIXED 2026-09-28]: tracking
  closed by voltage-scaling the implicit clamp + governor to the V-line
  no-load point (was min(spec, V-line): 6.7% path disagreement at fresh
  pack fed observer phantom load into peel; A/B isolated: 88% over with
  spec-fixed bounds, ~25% with V-line, transmit 3.40 m both ways).
  Deterministic (-O2 and -O1+ASan identical); XFAIL branch kept as
  fallback tripwire. See `src/ecosystem/mfs/docs/KNOWN_FAILURES.md`.
  (2026-09-26 despot audit; F1 fixed 2026-09-28 despot audit; F2 tracking
  fixed same-night despot sweep.)
- [ ] Jointed air-spin limit cycle: free-spinning jointed wheels oscillate
  (motor 86 rpm vs true wheel 799 rpm) via revolute-to-kinematic-chassis
  impulses vs slew/governor/implicit-solve. Contained (diode+slew bound it;
  an airborne bypass was tried and reverted after runaway to ±1900 rpm).
  T6/T8 now test the motor endpoint isolated (pass); the jointed-air plant
  is covered by T11 stability only. Proper fix is joint-aware motor
  integration or bearing damping. (2026-09-26 despot audit.)

## Correctness and validation

- [x] Correct and run CCD, constraint, friction, restitution, and free-flight invariant checks; audit continuation records the oracles and tolerances.
- [x] Build and run all 14 paranoia targets, including energy/momentum, scene persistence, and spring-joint checks.
- [x] Run canonical, isolated legacy, paranoia, MFS, and TUI checks under combined AddressSanitizer and UndefinedBehaviorSanitizer (full runner profile).
- [x] Add broader randomized/property-based physics tests and differential
  checks for simple analytic cases. *Partially done 2026-09-29: matrix-inverse
  property sweep, cylinder-SDF brute force, 15-axis SAT depth/orientation sweep,
  broadphase pair coverage, and an independent free-flight ODE comparison all
  exist (or were added as part of the audit). Still open: seeded property
  coverage for collision, constraint and configuration invariants, and a
  ThreadSanitizer run.*
- [x] **Torque-free angular momentum — CLOSED, and this entry was badly stale.**
  It previously read: *"still first-order … loses ~2.7% of |L| in 2 s … the
  `angmom` gate stays at 3%."* **Both numbers were wrong by the time anyone
  read them.** `tests/mpe_suite_a.c:414-424` records the actual state: the gate
  was tightened 3% -> **0.5%**, and measured drift is **0.0028% over 2 s, was
  2.68% — a ~950x improvement**. The mechanism is that torque-free bodies now
  solve the self-consistent `omega = I(rotor(w,dt) R)^-1 L` against the
  exactly-conserved world-frame `L`, so residual drift is float round-off. The
  "second-order scheme" future work described above is therefore already done
  and the residual 2.7% belongs to the FALLBACK path only
  (`docs/DESPOT_AUDIT_2026-10-01.md:23-26`), not to the live solver.

  Caveat recorded rather than smoothed over: this gate uses an anisotropic
  cube, while `docs/VALIDATION.md` retracts a *cube*-based `|I^-1 w|^2`
  measurement as physically meaningless (for an asymmetric top the world
  inertia changes as the body rotates, so `I^-1 w` is not conserved even with
  zero torque). Those two readings are not obviously compatible. The cube here
  measures `|L|` itself against a conserved world-frame `L`, which is a
  different and defensible quantity, but the tension is noted rather than
  resolved. A sphere control would settle it and is worth adding.

## Operational and release hygiene

- [x] Stale Makefile targets removed (`module:`, `ecosystem:`); MFS section
  delegates to ecosystem/mfs/Makefile (was duplicated and drifted).
- [x] Module stage detachment before unload: detach-everywhere +
  forget-pointers on every unregister path, proven by `loader_lifecycle`.
- [x] Runner now pins a floor on the discovered suite size
  (`MIN_SUITE_ENTRIES`, raised to 44 on 2026-10-03 (was 36, then 42 — and both times it sat BELOW the real registry, which is the exact failure the floor exists to prevent)) and has a contract test for it. Deleting
  a registry entry and its make target together used to shrink every headline
  count with no failure anywhere.
- [x] The SHIPPED frustum culler is now covered (2026-09-29). Plane extraction
  and the sphere test were inline inside `render_scene_current`, a GL function
  no headless test can call, so the real culler was untested while the legacy
  test re-implemented it locally and the canonical case projected one point.
  Both are now in `math4_special.h`, the renderer calls them, and
  `mpe_t_frustum_culler` (blocking) sweeps 6 camera poses for no false
  exclusions over 20,064 reference-inside samples. Writing that test caught a
  real transposition bug in the first extraction attempt (the inline code stored
  planes as `(d,a,b,c)`; the shared helper reads `(a,b,c,d)`).
- [ ] Document numerical guarantees and unsupported CCD/rotational cases precisely.
- [x] Root README exists (`readme.md`); release gates updated to verified
  behavior (32/32 v2, MFS 8 gated unified: strafe F1+F2 hard-gate since
  2026-09-28 (F2 margin thin, 25 vs 30, XFAIL tripwire kept); lifetime
  rules). Updated
  2026-09-28 (was "11 gated + 5 info" for the retired per-test binaries,
  then "incl. 2 loud strafe XFAILs").
- [x] Thread-safety boundaries: registry/MEI/MFS-internal locks, leaf-lock
  ordering, tick-boundary loader rule documented in headers (TSAN proof
  remains future work).

## Suite v2 (2026-09-23): testing suite reinvented in C from scratch

- [x] New harness `v15S/src/tests/mpe_test.h` (registry, file:line asserts,
  config save/restore, `mpe_floor_slab`/`mpe_floor_plane` Coulomb helpers).
- [x] All 30 tests reimplemented (`mpe_suite_a/b/c.c` + `mpe_suite_main.c`)
  as one binary `test_mpe_suite` (`make build_suite`, exact dispatch).
- [x] Root-caused v1's 3 FAILs as TEST bugs (missing frictional floor +
  wrong list4 axis); v2 ports pass 32/32 including the fixed setups.
- [x] v2 exposed + fixed an engine bug: uninitialized `cylinder_half_length`
  (sphere/cube inits) broke determinism across tests in one process.
- [x] Runner (`--suite`), `installcheck`, `run_all.sh` moved to v2 canonical.
- [ ] Migrate the 14 paranoia tests to v2 builders; remaining legacy suites still duplicate setup and need shared helpers.

## TUI ground-up validation (2026-09-23): 45/45 from raw dumps

Headless/raw-dump battery (the one-off helper was not retained; its results are preserved here, and current runs use workspace test targets):
- determinism byte-identical x7 scenes; tower 6-asleep exact heights KE=0;
  pendulum T=3.0556 vs sphere-compound 3.0105 (1.5%, nonlinear-correct);
  spring T=1.4051 vs 1.4050, dE<3.9%; CCD 60/144/300 stopped at face -0.55;
  roller grounded/moving/spinning; conveyor dx=3.000 exact; stress 304 awake
  finite; inertia 0.1066667 exact; rope free-fall matches Stokes velocity to
  1e-3 with exactly the documented N*1/2*g*dt^2 symplectic-position lag
  (lag-corrected err 0.00083 m) — dual-path selector proven by measurement.
- FOUND + FIXED: F10 scenes shipped with no frictional floor (same family as
  the v1 stack/driven/list4 setup bugs). Floorless, the pile disperses to
  +-235 m by 60 s (0/27 asleep, KE=30, runmax 10.7) while loose gates
  (fin<5, runmax<15) still pass. With matched Coulomb floor slab: 27/27
  asleep, KE=0, run-max 0.0000. Fixed in `scene_init.c` (in-engine F10),
  `tui_main.c` (TUI f10), v1 `f10_long_run_test.c` + v2 `mpe_t_f10_long_run`
  (floor + true settle gates fin<0.25/0.5, runmax<2.0, all-asleep).
- Fallen gates now skip static bodies (floor slab at y=-0.5 tripped y<-0.2).
- NOT fixed (honest limits): TUI/V04 in-engine F10 ritual needs a display;
  headless proof stands in its place until then.

## MPI + MFS audit fixes (2026-09-23): everything implemented + verified

MPI core (engine suite v2 32/32 green, incl. `loader_lifecycle` + `ftc_ecosystem`):
- P0 closed: load validates before registering (rollback truncates to
  snapshot); cleanup NULLs stage slots; retain/release resolve the OWNING
  handle by registry origin + dladdr (old pointer compare never matched);
  unload refuses -2 while any live world references the code and detaches
  + resets + dladdr-purges pairs pre-dlclose (all worlds, not primary);
  tombstoned registry slots (no memmove shift-alias); builtin takeover
  refused (6 pairs + hash/seq-impulse); pthread lock + once-init; per-pair
  builtins call removed.
- P1 closed: broadphase_state/solver_state slots threaded through BOTH
  step paths (7 sites); hook tables snapshotted before iteration;
  tangent2 no longer flipped on swap (prepare rebuilds it); capsule is a
  true segment capsule (exact vs sphere, sampled vs cube/cylinder, engine-
  parity guards, pen clamp, 1e-4 epsilons, bounding invariant
  R=sqrt(h^2+rc^2)); term_mod validates kinds, reports busy vs unknown,
  dumps all tables + actives, modinfo fixed via loader names; over-long
  names rejected; ABI versioning documented (append-only no_collide noted).
- Loader resolves ecosystem bundles (ecosystem-first precedence) and the
  jail covers ecosystem/mfs/*.so; proven by probe (load/attach x2 worlds/
  detach/unload, gone=1).
- Engine: sanitize preserves custom bounding radius (was xsqrt(3));
  initializers zero foreign/diagnostic fields; `no_collide` flag skips
  broadphase/dispatch/floor/CCD (GUI proxies non-colliding + despawn/
  clear APIs + no-double-step contract).

MFS (suite 11 gated + 5 info, 0 fail; was 13/3 + broken make):
- Builds: dup mpe_module_desc deleted; Makefile thin .so + build/ objs +
  fixed clean + shared mfs_sources.mk; include convention (no ../ crosses).
- Floors: shared tests/mfs_test_common.h (128 iters + tile 1.0/0.8 floor);
  all drive tests + physics_truth robot subtests use it.
- Drivetrain: torque free-speed governor; implicit-in-speed motor solve
  with disturbance observer (explicit stall at lock, stable at free);
  explicit-instantaneous feedforward twin for roller thrust; grip budgeted
  from wheel materials; torque-derived strafe with 1.1x breakaway margin
  (rollers unmodeled, flagged); 0.25 m/s idle-hold gate (was claimed,
  missing); 3 m/s clamp -> clamp_events telemetry; chassis sleep-wake on
  velocity; odom roller-fusion + odom_slip flag; copper thermal derating.
- module_1: pre_step routes drivetrain_update; canonical mixer; Start
  latch + A/B logic fixed; dead edge vars removed; flywheel spin axis
  tracks chassis tilt; roller torque about world axle; balls_fired honest;
  attach inits constraint pool (robot creation was dead); flywheel pylon
  raised clear of slop; test strengthened (1.36 m, 3969 rpm, fired=1).
- physics_truth: T3 matched floor; T6 airborne vacuum free-spin; T14
  position + no-runaway (stable pure-roll edge documented as engine
  follow-up); dup inits removed; T8/T12 documented as one curve.
- Battery: 20 A PTC fuse (brownout 1.2 V, auto-recovery) + 10 V OCV floor.
- MEI: per-world attach, locks, unregister, detach-everywhere;
  mfs_internal per-world slots, detach keeps registration, idempotent
  bundle attach; config_get shooter_rpm works, rest honestly -1.
- Suite: diags ungated (5 info), stale physics_truth_diag deleted,
  module_1 wired in, teleop heading gate (0.0029 rad), hotload 3.64 m
  vs 0.5 gate, docs corrected (FTC_SPECS x3, teleop ratio, drivetrain.h,
  README counts).
- Known follow-ups: anisotropic friction keystone (roller-correct strafe);
  engine cyl-plane rolling decay on slop contacts; property tests.

## FTC terminal wiring (2026-09-23): drive/configure/telemetry live

- `mpe_loader_symbol()` resolves plugin APIs by path/module/ecosystem
  name without new references (terminal drives APIs the engine never
  links; no stale pointers: resolved per invocation).
- `eco ls|attach|detach|command|config` terminal commands +
  `mpe_ecosystem_state()` per-world state lookup.
- `ftc spawn|list|drive|telemetry|preset` terminal commands (dlsym'd
  fleet API, auto-attach + auto tile floor, persisting commands).
- Bundle `command()` implements help/spawn/drive/list/telemetry;
  `config_get(shooter_rpm)` works; fleet lookup falls back through
  bundle-internal attachments (weak-linked, standalone .so unaffected).
- Suite `ftc_ecosystem` proves the terminal-identical path headlessly
  (3.59 m driven); how_to_use documents the full session.

## POSIX terminal commands (2026-09-23): libglib out of command logic

- New `ui_input/term_posix.h` (static-inline, zero link surface):
  ASCII case/compare/down (locale-independent), NULL-tolerant
  strdup/strndup, printf-to-malloc, prefix test, empty-keeping
  strsplit (g_strsplit contract), NULL-tolerant strfreev, quote-aware
  argv parser (g_shell_parse_argv subset: quotes + backslash, no
  expansion; unmatched quote errors), CLOCK_MONOTONIC time.
- Converted: debug_terminal x2 copies (compare/split/alias/argv/errno-
  free errors/time/types), term_fs/query/sys/admin (prefix/compare/
  int64_t), microvim x2 copies (dup/free/format/ascii).
- Deliberately kept at the display/input edge: GTK signal signatures,
  text buffers, key events (GdkModifierType), CSS refs, app quit.
- Proven by a one-off POSIX helper (parser/split/ascii/time unit checks; the helper was not retained)
  plus full gates: engine 32/32, MFS 11+5, tui-smoke green.

## Audit continuation (2026-09-24)

- [x] Full legacy + paranoia run: 44/44 pass, zero blocking failures
  (`temp/runner-final.log`).
- [x] End-to-end verification: warning-enabled engine build, canonical Suite v2
  32/32, MFS 11 gated + 5 informational, and TUI smoke finite
  (`temp/verify-overall.log`, `temp/verify-mfs.log`).
- [x] GCC `-fanalyzer` across the active v15S engine. Mesh allocation and
  terminal-editor buffer/undo failure paths were fixed; one retained-undo
  ownership warning remains from analyzer inability to follow the static undo
  ring, whose entries are freed on reload and close (`temp/gcc-analyzer-final.log`).
- [x] Headless runs disable gamepad probing; the canonical suite build refreshes
  the MFS bundle through its owning Makefile before loading it. Script/test
  scratch and logs use the project `temp/` directory.
- [ ] Run ASan/UBSan, randomized/property tests, differential analytic checks,
  and thread-sanitizer validation.
- [ ] Complete rotational CCD for rotation-only tunneling and consolidate the
  GUI/headless simulation step paths.
- [ ] Audit non-built historical artifacts. **The previous wording here was
  false and is corrected:** it claimed the frozen `v15R3` source tree was
  "absent from this workspace (only its release notes are present)". It was
  absent from the *checked-out working directory*, which is a different
  statement. Verified 2026-10-03: the `V1.5R3` tag is present and complete —
  `git ls-tree -r V1.5R3` returns **191 files, 142 of them under `v15R3/src/`**,
  including the full GTK3 tree and its makefile (readable, and it does
  `pkg-config --cflags gtk+-3.0 epoxy`, confirming it is the GTK3 generation).
  The repository is not shallow. So the tree is fully recoverable with
  `git checkout V1.5R3` — which is exactly what the readme tells a reader to do
  — and the P0 gates that rest on it can be audited rather than assumed.

Audit notes and boundaries are in `../AUDIT_REPORT_2026-09-24.md`.




- [x] **MFS H5 (intake could not be stopped) — FIXED 2026-09-29.** Two
  actuators on one joint, neither wired to state: a joint motor frozen on at
  creation-time speed, and a P-control it outvoted. Unified onto the joint
  motor, driven from `intake_active`/`intake_speed_rpm`/`intake_power` every
  tick (this also revived the dead momentary-reverse). Gated by the new
  `mfs_t_intake_stop`: OFF now 0.004 rad/s (was 62.532, i.e. never stopped),
  reverse now -59.495 (was +61.371, i.e. never reversed). MFS 10/10.

- [x] **MFS H6 (flywheel spun perpendicular to its own symmetry axis) —
  FIXED 2026-09-29.** Three disagreeing axes; the "tilt 35° about X" step was
  a no-op on the disc's symmetry axis because that is the axis X-rotation
  leaves fixed. Unified all three on `(0, cos35, sin35)`. Gated by the new
  `mfs_t_shooter_axis`: pre-fix `|disc·joint| = 0.0000` (perpendicular) and a
  90° launch; post-fix 1.0000 and 35.00°. MFS 11/11.

- [x] **MFS H7 (fired balls carried no spin, Magnus unreachable) — FIXED
  2026-09-29.** The launch transferred linear velocity only and never wrote
  `ball->angular_velocity`, so the Magnus branch's `spin_rate > 10.0` gate was
  unreachable — live code nothing could trigger. Added the contact spin
  transfer (`omega_ball = v_surface / r_ball`, same 80% factor as the linear
  term). Gated by the new `mfs_t_ball_spin`: \|omega\| was exactly 0.0, now
  742.1 rad/s (flywheel at 3957 rpm of a 4000 rpm target). MFS 12/12.
  *Approximation, not a friction solve — cannot express skid.*

- [x] **Tank turn: unexplained 5% heading change — RESOLVED, and a fabricated
  claim retracted (2026-09-29).** Two separate things came out of this.
  First, the cause: toggling the blocked-rotor gate in `motor_observe` with
  everything else fixed reproduces it exactly (gate on 2.3003/0.0603, gate off
  2.4167/0.0774). A pivot turn starts with the wheels nearly stationary, so the
  gate's "saturated AND not turning" condition is briefly true and the initial
  drive is trimmed — a 22% tighter turn. Which value is *correct* is unknown:
  there is no published tank-turn rate for this robot.
  Second, and more important: I had written that 2.3003 "happens to be the
  target the tank test documents". **That was fabricated** — the test gates only
  `heading >= 0.1` and `disp <= 0.3`, and the only occurrences of 2.3003 in the
  tree were in my own text. Retracted in `KNOWN_FAILURES.md`.
  Acting on it: those gates were 23x and 5x too loose to catch a 5% change.
  Now pinned to the measured baseline at 8% / 20% tolerance, explicitly
  labelled a regression baseline and NOT a specification.

- [x] **`ftc_hotload` failure diagnostics.** It printed only
  `[FAIL] ftc_hotload (failures=N)` with no reason, so a genuine failure
  reached CI with nothing pointing at what broke. Now the count also goes to
  stderr with a pointer to the per-check lines and the run log.

- [ ] **MFS motor chain, step 2 — re-measured 2026-09-29, needs a bigger
  landing unit.** Delivered-torque accounting (reflected rotor inertia,
  `I_total = I_axle + J_rotor*gear^2`) genuinely works now: physics strafe
  transmit 0.8739 -> 0.9727 m (+11%), and the stall endpoint is
  **byte-identical** in both phases (the original revert's stall regression does
  not reproduce for this form of the change). But encoder odometry goes
  1.1202 -> 2.1371 m, over-reporting by 2.2x, and the odometry test correctly
  goes red. The wheels slip more and the odometry model has no slip term.
  **Land it together with an odometry slip term and the lateral `VREF` retune,
  not before.** Not landed for exactly that reason. Full numbers in
  `KNOWN_FAILURES.md` -> `MOTOR-II-2026-09-29`.

- [x] **The suite never loaded the config (2026-09-29) — FIXED, and it was the
  root cause of the "unexplained" deep-overlap residual.** `g_cfg` is a
  zero-initialised global and `tests/mpe_suite_main.c` never called
  `mpe_config_init()`, so the MPE suite ran with `solver_iterations = 0`,
  `bias_factor = 0`, `penetration_slop = 0`, zero restitution and zero friction.
  The solver was not iterating. The game, headless, TUI and MFS all configure
  themselves correctly; the suite was the one path that did not. Fixing it
  resolved DEEP-2026-09-29 as a side effect. Full analysis in
  `KNOWN_FAILURES.md` -> `CONFIG-2026-09-29`.

- [x] **Test-harness upgrade (2026-09-29).** Four additions, all aimed at the
  failure modes that actually let defects through this audit:
  1. *Harness contract* — `mpe_test_begin()` now **initialises** `g_cfg`
     (it only used to save/restore, faithfully preserving inherited garbage)
     and `mpe_test_end()` **refuses to certify** a result produced under a
     degenerate config. A green line from a solver that was never switched on
     is now impossible.
  2. *Regime matrix* — the **entire** suite re-runs under five configurations
     (`default/light/heavy/brittle/sticky`: iterations 8–128, gravity ×0.25–×3,
     friction ×0.25–×4, restitution 0–0.95, sleep on/off). Wired into
     `test_runner.py`. A property that holds at one setting and not another is
     invisible to a single golden number — that is exactly how CONFIG-2026-09-29
     stayed green. It found a failure on its first run.
  3. *Metamorphic tests* (`tests/mpe_suite_d.c`, **6** registered) — rotation
     equivariance, solver boundedness plus a live-knob check, config
     reachability, sleep honesty, mouse-look axes, body-material ordering.
     **No golden numbers**: they assert relations any correct engine satisfies.
     **DESPOT-2026-10-03, two of these were not doing that.** `meta_rotation`
     could not fail (its failure branch printed `[XFAIL]` without incrementing
     `failures`), and `meta_convergence` was measuring a fixture that was
     BITWISE IDENTICAL at 1 through 128 solver iterations, so its gate passed
     trivially. Both fixed and both re-verified by deliberately breaking the
     engine. Note also that error is measurably **non-monotonic** in iteration
     count for a friction stack, so the old "monotonicity" wording was never
     true and the gate now asserts boundedness instead.
  4. *Withdrawn honestly* — `meta_sleep` was written, failed to converge, and
     was removed rather than shipped red or unjustifiably green.

- [x] **META-ROTATION-2026-09-29 — EXONERATED 2026-10-01, fixture was the bug.**
  The "stale contact" theory (has_contact stuck after 2.59 m separation) was
  a misread of per-BODY flags: the persisting contact was sphere-FLOOR (the
  rotated trajectory genuinely reached the floor), not sphere-sphere. Deeper:
  R^-1 Phi(R x) = Phi(x) needs an R-symmetric environment, and the y=0
  backstop (CCD sweep + depenetration shove + boundary clamp assume it even
  in "floorless" worlds) plus the box deny arbitrary-R probes — the rotated
  run truly interacts with the backstop while the unrotated one does not
  (measured: ~1e-7 agreement through the bounce, then a positional-only shove
  with no velocity change and no contact flag in one world only). Fixture now
  floorless with a yaw-only probe (maximal valid symmetry); engine holds
  5.4e-07 m / 1.5e-07 m/s over 150 ticks incl. a real bounce. XFAIL kept as a
  dormant tripwire. Phantom-floor audit (CCD/depenetration acting below y=0
  with the solver floor disabled) recorded as honest design, not changed.

- [x] **SLEEP-H1-2026-09-29 — EXONERATED 2026-10-01, expectation was the bug.**
  Reproduced EXACTLY under heavy (vertical pop from rest: v=0.0000 at spawn
  height, 0 travelled, is_sleeping FALSE at 0.5 s) — then watched it fall
  lawfully asleep at 0.83 s when the 0.5 s timer expired. Nothing frozen,
  nothing stuck: the ball had settled at ~0.33 s and the engine was
  mid-countdown. The withdrawn test demanded (moving XOR asleep), missing the
  legitimate third state (settled, timer pending). Locked with the committed
  `sleep_settle` gate (moves → settles → sleeps-iff-enabled, all regimes);
  direction-B companion (unsettled cube) also settles fine today.

- [x] **Mouse-look asymmetry ("right/down lock, left/up don't") — FIXED
  2026-09-29.** The sign convention was correct all along; the plumbing was not.
  With the relative pointer live the handler could still fall through to the
  absolute cursor path, which is direction-dependent exactly as reported:
  flicking toward an edge pins the cursor and stops absolute events (that
  direction works by accident), flicking back un-pins it and they resume and
  overwrite the relative signal. Also fixed: deltas were assigned rather than
  accumulated, so multi-event flicks lost magnitude. The convention is now a
  pure function (`ui_input/mouse_look.h`) asserted headlessly by
  `mpe_t_mouse_look_axes`, and per-direction receive counters
  (`mouse_lock_diagnostics()`) separate "compositor never sent it" from "we
  converted it wrong". See `KNOWN_FAILURES.md` -> `MOUSELOOK-2026-09-29`.

- [x] **MOUSE LOCK ACTUAL ROOT CAUSE (2026-09-29): `root_gtk.c` still did
  `g_setenv("GDK_BACKEND", "x11", TRUE)`.** The GTK4 engine never ran on native
  Wayland — it used XWayland — so every Wayland-side fix was dead code behind
  that one line, and the GTK3-era "force X11" workaround was never removed.
  Windowed failure = X11 warp cannot fire once the cursor has left the window.
  Fullscreen = window covers the screen so the cursor cannot leave. Three
  changes shipped together: backend no longer forced (GDK picks native
  Wayland or X11), `zwp_locked_pointer_v1` confinement added (the relative
  pointer supplies deltas but does **not** confine — that was the windowed
  failure), and globals bound once at startup instead of lazily from inside a
  GTK handler. See `KNOWN_FAILURES.md` -> `MOUSELOOK2-2026-09-29`.

- [x] **GAME RAN WITH ZERO-FRICTION OBJECTS (2026-09-29) — FIXED.**
  `root_gtk.c` built the entire default scene from `when_realised()` while
  `mpe_config_init()` ran 24 lines later in `app_activate()`. `g_cfg` is a
  plain global, so it was still all zero, and body materials are stamped from
  it at construction time and never retro-fitted: every default-scene object
  got `friction = 0` and `restitution = 0` permanently. That is the endless
  rolling/spinning, the bounce swinging between dead and violent, and the F5
  stack squirting. The tell was spatial, not numerical: **correct inside the F10
  region, wrong outside it** — runtime-spawned content is born after the config
  exists, default-scene content before it.
  Fixed structurally: `mpe_config_ensure_ready()` called from every
  `rigidbody_initialisation_*()` (the choke point) and from
  `scene_init_default()`, so caller order no longer matters. Gated by
  `mpe_t_body_materials_live`, which forces the unready precondition and fails
  4 assertions with the fix removed. See `KNOWN_FAILURES.md` ->
  `SCENEORDER-2026-09-29`.
