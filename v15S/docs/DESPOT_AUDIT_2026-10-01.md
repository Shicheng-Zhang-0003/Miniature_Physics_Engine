# Despot Audit 2026-10-01 — Math, Programming, Operational

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
  `O(|w|dt·I_aniso)`. Refresh-before-prepare is future work (cost vs gain).
- Dead-reckoned hinge angle vs measured `q_rel` twist; prismatic already
  enforces on measured pos. Revolute measured-angle enforcement is future.
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
