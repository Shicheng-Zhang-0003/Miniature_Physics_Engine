# Remaining Work

## Fixed in 2026-09-23 despot audit (verified: engine + mpe-tui + 27/30 headless green, same 3 pre-existing FAILs as clean HEAD: stack, driven_wheel, list4_cylinder_floor)

- [x] det_reduce_pi4 quadrant correction (+1/+3, was +2), single-count trig fallback, lazy sin/cos branch.
- [x] det_pow_retention chunking (no libm desync in-tick), CCD floor linear fallback + double quadratic, Poisson gate sign-enforced.
- [x] Sphere-cube degenerate normal (face normal, was -Y), cylinder-inside contact point on surface, gyro stability cap, broadphase linear clamp 10m + stratified double probe.
- [x] Revolute 6x6 in double with equilibration; quat sanitize + quat->mat double parity; lookAt re-orthogonalized.
- [x] Registry owns pair/module strings (no .so rodata dangle); register_broadphase/solver return int; capsule destructor + NULL/slop-per-world; loader clears primary stage pointers on unload.
- [x] scene_saving NULL/type-bounds; scene_load full-float + duplicate-ID veto, legacy empty-scene, custom identity preserved (100); config localtime_r + truncation consume + 0700 dirs; tui_dump NULL/cache; tui header/ delwin/auto-config; terminal capture/alias hardening; term type mismatch repaired (engine builds).
- [x] test_runner --include-paranoia + exact-match; makefile installcheck grep, help, clean, module target, .PHONY, install plugins + uninstall, check-deps-tui, native hard-fail; .gitignore mpe-tui; docs (joints persist all 5, counts 28+3, compat versions, freeze, version string, BE, Wayland-P1, native, V04 run-max, V03 F10/F11, deps).
- [x] Pre-existing FAILs confirmed on clean HEAD (not regressions): stack, driven_wheel, list4_cylinder_floor. fix_log contradiction corrected; run_all.sh logs to /tmp; verify.sh runs full suite + tui-smoke.

## High priority (still open)

- [ ] Repair the remaining paranoia tests and make their mathematical oracles valid.
- [ ] Diagnose and fix cylinder collision and sleep/depenetration failures with minimal reproducible cases.
- [x] Plugin load/attach/unload + stage-backend lifetime regression test
  (`loader_lifecycle` in Suite v2: real capsule .so, busy -2, purge, reload).
- [ ] Replace the duplicate GUI and headless physics pipelines with one canonical step path.

## Correctness and validation

- [ ] Add valid CCD, constraint, friction, restitution, and free-flight invariant tests with documented tolerances.
- [ ] Build and run every paranoia target, including energy/momentum, scene persistence, and spring-joint tests.
- [ ] Run the complete suite under AddressSanitizer and UndefinedBehaviorSanitizer.
- [ ] Add randomized/property-based physics tests and differential checks for simple analytic cases.

## Operational and release hygiene

- [x] Stale Makefile targets removed (`module:`, `ecosystem:`); MFS section
  delegates to ecosystem/mfs/Makefile (was duplicated and drifted).
- [x] Module stage detachment before unload: detach-everywhere +
  forget-pointers on every unregister path, proven by `loader_lifecycle`.
- [ ] Document numerical guarantees and unsupported CCD/rotational cases precisely.
- [x] Root README exists (`readme.md`); release gates updated to verified
  behavior (31/31 v2, MFS 11 gated + 5 info, lifetime rules).
- [x] Thread-safety boundaries: registry/MEI/MFS-internal locks, leaf-lock
  ordering, tick-boundary loader rule documented in headers (TSAN proof
  remains future work).

## Suite v2 (2026-09-23): testing suite reinvented in C from scratch

- [x] New harness `v15S/src/tests/mpe_test.h` (registry, file:line asserts,
  config save/restore, `mpe_floor_slab`/`mpe_floor_plane` Coulomb helpers).
- [x] All 30 tests reimplemented (`mpe_suite_a/b/c.c` + `mpe_suite_main.c`)
  as one binary `test_mpe_suite` (`make build_suite`, exact dispatch).
- [x] Root-caused v1's 3 FAILs as TEST bugs (missing frictional floor +
  wrong list4 axis); v2 ports pass 31/31 including the fixed setups.
- [x] v2 exposed + fixed an engine bug: uninitialized `cylinder_half_length`
  (sphere/cube inits) broke determinism across tests in one process.
- [x] Runner (`--suite`), `installcheck`, `run_all.sh` moved to v2 canonical.
- [ ] Migrate the 14 paranoia tests to v2 builders; tighten vacuous gates
  (paranoia_constraints energy growth, contact_solver apex detector, etc.).

## TUI ground-up validation (2026-09-23): 45/45 from raw dumps

Battery `/tmp/tui_validate.py` (kept outside the tree; rerun: `python3 /tmp/tui_validate.py` from `v15S/src`):
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

MPI core (engine suite v2 31/31 green, incl. new `loader_lifecycle`):
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
