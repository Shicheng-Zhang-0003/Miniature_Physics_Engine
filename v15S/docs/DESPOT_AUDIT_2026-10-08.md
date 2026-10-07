# Despot Audit 2026-10-08 — Tandem-Helix: everything open

Solves or bounds every item left open on 2026-10-07 plus SIMD/threading.
Method: code read, temp probes, suite green throughout. Scratch under
`temp/` only.

## Solved

- Joint pools init inside `physics_world_init` (phantom-joint class closed).
- MSVC scope + auto `/fp:strict` when `CC=cl` (proof still needs Windows CI).
- SIMD `core/simd_math.h`: SSE2 add/sub/scale, scalar fallback, bitwise
  self-check (`broadphase_simd_selfcheck`, loud on mismatch). Scalar
  canonical; SIMD proven identical.
- Threading `MPE_THREADS=1-8` (default 1): parallel sanitize, disjoint
  writes, bitwise identical; pairs/solve serial for order. `mpe_parallel_`
  helpers in `physics_world.c`.
- `make tsan` added. This container aborts TSan on memory mapping
  (`FATAL: ThreadSanitizer: unexpected memory mapping`) — env, not code.
  TSan proof needs a capable host; locks/boundaries already documented.
- Rotational CCD: world max tip-speed margin (+capped 2 m/s) added to mover
  sweep (`g_ccd_obstacle_margin`); blade regime documented as unsupported.
- Deep containment: penetration >0.30 m moves lighter body only
  (`depenetration.c`); +Y inversion class closed.
- Buckling: ≤32 extra iterations (both step paths, cap 128) when bodies≥8
  and residual>slop. Mitigates (0.31@64→0.00@128 lever), does not solve
  chaos (≤0.69 m across F11 seeds remains possible).
- Air-spin: bearing `tau=-c·w`, `c=1e-6` (c·dt/I≈0.067) on airborne wheels
  only; grounded thrust unchanged. Diode+slew remain load-bearing.
- `angmom` sphere control added (isotropic L||w, same 0.5% gate).
- `v15S/docs/NUMERICAL_GUARANTEES.md`: exact budgets + unsupported list.
- `v15S/validation/property_sweep.py`: seeded Coulomb/Poisson/spring/
  pendulum/stop/TOI/inertia laws, 0 failures.
- Step paths: stages shared (dispatch/prepare/Poisson/split/depenetration/
  islands/cfg); frame drivers separate by design (GUI accumulator vs
  headless dt, passive bitwise proof). Legacy parity: buckling, free-flight,
  dt guards mirrored.

## Bounded, not solved (honest)

- Normal warm-start with stability proof (cold stays; full restore regresses
  F10 4/27 + f11 0.28 m; damped 0.1-0.2 seed unproven — needs proof, not a
  knob).
- Full Gottschalk distance numeric (overlap 400/400; depth spot-checked).
- Oblique multi-body angular-momentum oracle (single-body 0.5% + control).
- TSan on capable host; MSVC bitwise proof; multithreaded solve (serial by
  design for order).

## Verification

- `build_suite` clean (no new warnings).
- `test_mpe_suite --all` (CWD=`v15S/src`): 44/42 green, 0 blocking.
- `--profile quick`: 58/58 green, 2 info.
- `ext_oracle_check.py` 10/10 + `property_sweep.py` 0 failures.
- `make tsan` builds; runtime aborts in this container (recorded above).
