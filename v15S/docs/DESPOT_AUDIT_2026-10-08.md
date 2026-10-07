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

## Addendum 2026-10-07T15:21Z — continued

- Damped warm-start knob `solver.warm_start_damping` (80th tunable,
  default 0=cold byte-identical). Full restore (1.0) stays a measured
  regression; damped 0.1/0.2/0.5 experiment open to temp harnesses with
  f10 27/27 + f11 <0.05 + 42/42×5 gates. Torture pins 0 (proven envelope).
- `property_sweep.py` extended to 24 laws (SAT depth, reject, oblique
  exchange, 2-body momentum) — 0 failures.
- Island-parallel solve INVESTIGATED AND REJECTED: islands are disjoint,
  but the solver is Gauss-Seidel in global sorted order; island-parallel
  is Jacobi across islands and changes results (breaks bitwise twins).
  MT stays per-body phases only (order-independent, proven identical).
- TSan container abort recorded as env, not code.

## Addendum 2026-10-07T10:05Z — SIMD full (+ two process scars)

- SIMD is full: add/sub/scale/cross in `core/simd_math.h` (SSE2, MSVC
  intrin, scalar fallback, `MPE_SIMD_OFF` knob, `build_suite_scalar`
  target), wired through every physics hot path (~700 sites), runtime
  self-check over fixed + edge vectors with zero mismatches, both binaries
  44/42 green. Reductions stay scalar by proof (order-dependent rounding).
- Scar 1: first cross shuffle pairing was dot-like garbage; suite went
  22/42 red. The self-check stayed silent because the wiring script had
  renamed its scalar references too (simd-vs-simd always equal). Fixed the
  pairing, restored scalar references with a WIRE-EXEMPT marker, proved the
  op standalone (`temp/simd_proof.c`) before rebuilding.
- Scar 2: the red persisted after the fix — stale binary. Rebuild commands
  ran `make -C v15S/src` from inside `v15S/src` (nonexistent path, silent
  no-op behind the grep), so every "rebuild" re-ran the bad binary. Forced
  rebuild from the right directory went green. Lesson: verify the compiler
  actually ran (count the gcc lines), never trust a quiet make.
- Numbers: op micro-bench (`temp/simd_bench.c`, bounded, 20M x 6op) SSE
  490 ms vs scalar 235 ms (wrappers lose in synthetic loops); engine suite
  wired 2.87-2.93 s vs scalar 2.95-3.08 s (neutral, noise). SoA batching
  ticketed as the real frontier.

## Verification

- `build_suite` clean (no new warnings).
- `test_mpe_suite --all` (CWD=`v15S/src`): 44/42 green, 0 blocking.
- `--profile quick`: 58/58 green, 2 info.
- `ext_oracle_check.py` 10/10 + `property_sweep.py` 24 laws, 0 failures.
- `make tsan` builds; runtime aborts in this container (recorded above).
