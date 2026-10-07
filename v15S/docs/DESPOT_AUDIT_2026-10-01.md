# Despot Audit 2026-10-01 — Math, Programming, Operational

> SUPERSEDED for counts by DESPOT_AUDIT_2026-10-07.md (41/42 era -> 44/42;
> 234/234 -> 232+2 xfail). History below is preserved; live truth is in
> the 2026-10-07 audit + docs/VALIDATION.md.

Scope: full `v15S` tree (kernel, physics, render, scene, config, ui_input, tui,
plugins, ecosystem/mfs), `tools/test_runner.py`, validation scripts, all
user docs. Method: three independent audits (math/physics, programming,
operational) + baseline registry discovery (41 canonical / 30 legacy /
14 paranoia / 12 MFS). Scratch and logs under `temp/` only.

## A. Mathematical truth — fixed

1. **CCD discriminant FMA** (`physics/collision_ccd.c:74`): `fma(b,b,-4ac)`
   is hardware/libm, not IEEE-exact. Replaced with `b*b - 4*a*c` (double,
   `-ffp-contract=off`). TOI clamp/no-clamp bifurcation now bit-identical.
2. **Revolute motor double-drive** (`physics/revolute_joint.c:447,1018`):
   accumulator feedforward + 6x6 constraint row each clamped to full
   `max_torque` summed to 2x. Split single budget: 0.5x each. Sum respects
   `motor_max_torque`; one-tick contact convergence retained.
3. **Revolute limits atan2f** (`:480`): one-shot libm on limits-enable is cold
   path, not per-tick. Now marks `det_fallback_trig()` for honesty; per-tick
   angle stays exact mults/adds.
4. **Torque-free L epsilon** (`core/rigidbody.c:852`): exact `==0.0f` missed
   denormal torque. Gate `<1e-24f`. Comment corrected: L fixed-point (live,
   converges, 0.0028%) runs FIRST, falls through to implicit-midpoint gyro
   (correct under torque, explicit-degenerate when free). Residual 2.7%/2s is
   the FALLBACK path only. Second-order Euler remains the real future fix;
   `angmom` gate stays 3% ceiling.
5. **Verified non-findings (no change, documented)**: SAT 6+9 complete; SDF
   inside `pen=min_clear+r`, pos on surface; Poisson `e*compression` + Newton
   bound; friction `min()` + ellipse radial-scale; inertias (2/5, m/12,
   1/2mr²+m/12(3r²+l²), axle X); quat rotor det_sin/cos + normalize; qsort
   unique dedup deterministic; single slop source. Tangent swap: `adoptable`
   requires exact order (no swap → cold, conservative, never injected);
   `match_role` role==2 already negates. Split vs depenetration: sequential
   (pre-move split, post-move residual), same slop — header stands.

## B. Mathematical truth — ticketed future (honest limits)

- Translational-only CCD: `|w|*R` gates volumes but floor equation ignores
  `omega×r` lowest-point velocity; fast-rim dip tunnels one tick, solver
  reseats. Rigid-body sweep (rotation-aware TOI) is future work.
- Bounding-sphere CCD for boxes: `R_bound≈√3*half` fires early, air gap +
  creep. Tighter swept support (box/segment TOI) is future work.
- 1-tick-stale world inertia: `k` built from last tick `R·I⁻¹·Rᵀ`; error
  `O(|w|dt·I_aniso)`. FIXED 2026-10-01 (refresh R·I⁻¹·Rᵀ from current
  orientation at prepare; exact mults, suite green).
- Dead-reckoned hinge angle vs measured `q_rel` twist; prismatic already
  enforces on measured pos. 2026-10-01 verdict (measured, then scoped back):
  per-iteration `accumulated = limit` clamps REMOVED (they froze the books at
  the stop while truth sat 0.085 past it — enforcement blind forever after).
  A quaternion-delta measured replacement was proven better settled (0.04 vs
  0.085) but worse in transients with free-spin bias, so dead reckoning
  ships with live books; static rest-past-stop under starved solvers stays
  ticketed (positional re-seat loses to P2P bias — tried, reverted, see
  revolute_joint.c notes). In-envelope behavior exact (0.0000 violation).
- TSan 2026-10-01: canonical suite 41/41 green, ZERO warnings
  (single-threaded paths clean). NOTE: needs `setarch -R` (ASLR off) — GCC 13
  TSan vs glibc 2.39 shadow-mapping conflict, toolchain issue, recorded.
- Greedy 4-point support (face-clip max-fan, cylinder farthest-same-face):
  deterministic, suboptimal on long quads. Max-area subset is future.
- Dead knobs labelled in `config/mpe_config.h` (bias→split-only, thresh is
  speed, max_speed inform-only).

## C. Programming — fixed (P0/P1)

- `core/mpe_loader.c same_path`: basename fallback removed; exact or
  realpath-canonical only. Wrong-handle `dlclose` closed.
- `mpe_loader_path_at/name_at`: rotating snapshot copies under lock (4×
  PATH_MAX/128). No interior dangle across load/unload/dlclose.
- `core/mpe_registry.c`: tombstone reuse same-name-only (broadphase/solver).
  Cross-name ABA (purge wrong address) closed; table 8, refusal loud.
- `ecosystem/mpe_ecosystem.c attach`: ABA re-validation after unlock; on
  descriptor swap, orphan state detached, attach fails loudly.
- `core/physics_world.c grow_manifolds`: all four arrays NULL-checked.
  `cleanup`: wild `tick_module_count` clamped to [0,8].
- `ui_input/microvim.c paste`: malloc NULL-checked. `load`: over-long line
  + ferror detected, fails loudly. `save`: short-write/ferror checked,
  backup must succeed before truncate. `:w/:wq/:x` refuse to quit on failed
  save.
- `scene/scene_load.c`: NULL path guard; legacy joint count bounded to 4096
  before malloc/loop.
- `tui/tui_main.c`: `atol` → `tui_parse_count` (strtol, errno, 0..100000,
  fallback + stderr). `tui_dump.c`: `body_count` validated vs capacity.
- `motor.c`: non-finite bus voltage/omega fail-closed (12.0V / return);
  NAN pack-brick closed. `battery.c` NAN warning retained (visible bug).
- `plugins/mpe_capsule.c` both sampled paths: h/r finiteness + range guard
  before `(int)(h/r)` cast.

## D. Operational — fixed

- Canonical denominator 41 (39+2) everywhere (was 32/32, 33+2):
  `readme.md`, `RELEASE_GATES.md:109,240`, `RELEASE_POLICY.md`, `evolution.txt`,
  `validation/V03.py`, `install/.../linux_install_instructions.md`.
- MFS 12 gated everywhere (was 8): `readme.md:376`, `README_MFS.md`,
  `docs/TESTING.md`, `evolution.txt`.
- `tools/test_runner.py`: stale 40→41 comment; added `MIN_LEGACY=30`,
  `MIN_PARANOIA=14` floors + enforcement; header documents regimes,
  floors, summary contract, `--allow-skip/--list`.
- `makefile help`: installcheck-v1, TUI_OUT/OUTDIR, MPE_TEST_REGIME,
  allow-skip, regime cost, runner-vs-make default.
- `validation/V01.sh`: determinism-override note + V02 rebuild gate.
  `V03.py`: `--non-interactive` CI mode, log under `temp/`.
  `V04.sh`: `set -euo pipefail`, ROOT/TMPDIR/MPE_GAMEPAD_DEVICE, absolute SRC.
- `.gitignore`: header `v15S/ tools/ reference_materials/` (was stale
  `v15R3/ fixes/`).
- `config/mpe_config.h`: dead-knob truth labels (see §B).

## E. Verification

- `tools/test_runner.py --list`: 41 / 30 / 14 discovered.
- `make check-flags`, `make build_suite`, quick profile via
  `temp/qa_runs/` (see run logs). Full/ASan/UBSan per runner profile.
- Remaining outstanding (unchanged, honest): rotational CCD, single step
  path unification (passive bitwise-identical, driven 2.7% observer
  feedback), joint air-spin limit cycle, torque-free second-order scheme,
  seeded collision/constraint property + TSan, Wayland matrix (X11 verified).

All changes built under `temp/` scratch; no outside-tree writes.

## F. External-truth verification 2026-10-02 (no engine changes from it)

Independent battery (`temp/ext_truth.c`, separately-coded harness; oracles
hand-derived from closed forms, cross-checked by a second implementation in
Python, `temp/ext_oracle_check.py`). **22/22 green** in C, 20/20 oracle
agreement in Python. Full log: `temp/ext_truth.log`.

Re-run recipe (from `temp/`, engine at `../v15S/src`):
`CORE="<core list: core/physics_world.c core/rigidbody.c core/mpe_registry.c
core/mpe_loader.c core/det_math.c core/mpe_primary.c
physics/collision_narrowphase.c physics/collision_cache.c
physics/collision_solver.c physics/collision_ccd.c physics/collision_cylinder.c
physics/broadphase.c physics/constraint.c physics/revolute_joint.c
physics/depenetration.c physics/islands.c config/mpe_config.c
config/mpe_config_schema.c scene/boundary.c ecosystem/mpe_ecosystem.c
physics/spring_joint.c>"`;
`gcc -I../v15S/src $CORE stubs_ext.c ext_truth.c -lm -ldl -pthread -lepoxy
-o ext_truth && ./ext_truth && python3 ext_oracle_check.py`
(`stubs_ext.c` stubs `scene_resolve_object_by_id` + `main_camera_fov`, the
same set the spring test stubs.)

Findings (all dispositioned, none left open):
- Four first-run failures were HARNESS bugs (rolling-vs-sliding oracle,
  half-vs-full inertia sides, transient-vs-creep gating, exact-touch setup),
  corrected same-session; the engine was right in each case.
- Poisoned live config (SEVERE while present): `v15S/src/status/engine.cfg`
  + `.backup` held F11-torture values (gravity −17, drag 0.63, rolling 4.95,
  sleep OFF) from the pre-fix era, and the GUI loads `engine.cfg` on
  startup — live sessions started in torture state. Both files deleted
  (clean defaults regenerate; F11 rewrite prevents recurrence).
- Uninitialized joint pools (P2 hardening, proposed not applied):
  `physics_world_init` zeroes the joint count but not `is_active` flags;
  every in-tree caller pairs it with `constraint_pool_init`, so nothing live
  is affected, but out-of-tree init-alone callers get phantom joints (bit a
  harness here: 0.64 phantom lean). Proposed: clear pools inside init.
- Sleepless tall-tower lean (0.64 vs 0.004 with sleep): documented design —
  sleep is load-bearing for tall stacks; `sleep.enable=0` is truth mode.
- TSan note moved here from §B: canonical suite 41→42 cases 0 warnings
  (needs `setarch -R` for the GCC 13 TSan vs glibc 2.39 mapping conflict).
- Suite is 42/42 (40 physics + 2 diag) with `sleep_settle`; MFS 12/12 inner;
  TUI 9/9 finite; quick profile 56/56, 0 blocking.

---
# DESPOT AUDIT 2026-10-04 — torture-leak recurrence, harness revival, exact Coulomb

Requested as a full mathematical, programming and operational audit with
everything fixed, docs upgraded, commits batched and pushed. Standing
instruction executed first: check `engine.cfg` before touching physics math.

## 1. engine.cfg: convicted by diff, engine exonerated by proof

`status/engine.cfg` (2026-10-04 09:03) vs `.backup` (2026-10-03 22:14): all
79 keys differed; every physics-relevant value was F11 torture (list in the
run record: gravity −17, drag 0.635, rolling 4.946, sleep OFF with 25–430x
thresholds, restitution gate −0.15, static gate 0.19, broadphase cells
53/71/440m, spawner masses to 9288 kg, safety net OFF). Backup matched
compiled schema defaults + the scene-stamped friction floor — the sane
state. Restored by copy; proofs archived under `/tmp/opencode/`.

Before any physics change, every formula in the hot path was re-verified
against its textbook statement (Goldstein/Catto/Box2D/Bullet/Ericson/
Gottschalk/Mirtich/Press): sphere/box/cylinder inertia, exact viscous
free-flight closed form + Taylor branches, L-conservation fixed point,
gyro sign/chain, exponential rotor, Hamilton product, rotate formula,
axis-angle, quat↔mat3, mat3 inverse/adjugate + singularity scale law,
effective-mass triple products, Coulomb cone + stick/slip select, Poisson
+ Newton bound, split projection + caps, rolling/spin model + units,
revolute K/Jacobian/RHS/bias, Hooke + Courant numbers, CCD quadratics +
slab, cylinder SDF + support + segment-segment, SAT-15 + projection.
Verdict: NO live sign error, missing term, wrong constant, unit error, or
timestep dependence anywhere. The Tom-and-Jerry physics was configuration,
not mathematics. (The one real physics defect found — [FRICTION-THRESH]
selection, §3 — was found by measurement, not by this read.)

## 2. Programming/operational fixes (all measured, all gated)

Torture-leak closure (7 files): guarded saves, in-memory snapshot, exit
cancel-restore, flag-before-start ordering, stderr+event-log backup
reporting. Scene: legacy-reader validation, joint-truncation veto,
drop counting, NaN-save refusal, rc-2 propagation. Render: uniform checks,
utility fallback, non-finite skip, GL error drain. TUI: ferror rc, full
finite scan, pair-scan coverage. Runner: regime completion, MFS dupe check,
`--test` labelling, config-mutation contract. Tests: CLAMP-TAUTOLOGY closed
(net-OFF + fall-speed + ever-contact on cylinder_drop/floor_diag/ccd-floor),
MPE_SKIPPED −1, 33 regime re-inits removed, 13 premise pins,
meta_convergence (b) withdrawn with calm-top replacement.

## 3. [FRICTION-THRESH]: from OPEN to exact Coulomb (full trail in docs/VALIDATION.md)

Old pointer (normal shortfall) REFUTED by λ_n measurement (0.16342 vs
0.16344). True mechanism: memoryless per-iteration stick/slip selection +
kinetic-clamp saturation (mu_s=0.9/1.5 broke at ~5.9N; +0.035/tick ratchet
= (F−μ_k·N)·dt; slipth=0.001 read μ_k·N exactly). Fix: tick-start snapshot
in prepare (`snap_friction_mu`). Battery: 0.999–1.005 of μ_s·N across dt
(1/30–1/120), iterations (8–128), μ (0.6–0.9), geometry, push duration.
Adjudicated blast radius: meta 32-arm buckles (recorded, gate reformulated),
MFS tank 0.0530→0.0642 (re-baselined with chain, heading in band, strafe
identical), everything else green.

## 4. Record

Full profile 234 (232 pass, 0 fail, 2 xfail, 6 info, 0 blocking) from
`temp/qa_runs/20261004T162513Z-456158/`; canonical 42/42 in all five
regimes; probes uncommitted in gitignored `temp/`; engine.cfg restored and
guarded. Open items carried forward unchanged: warm-start absence
(iteration tax), F11 buckle transient, CCD rotation, angular multi-body,
manifold-point validation, MOTOR-III/F2.
