# MPE Numerical Guarantees (precise, no wishes)

Canonical dt `1/60 s`. Determinism: fixed dt + fixed iteration order +
exact IEEE `+ - * / sqrt` + fixed-coefficient polynomials (`det_math.h`) +
`-ffp-contract=off` (GCC/Clang; MSVC needs `/fp:strict`, unproven).
`ENABLE_NATIVE=0` required for bitwise twins. `ENABLE_NATIVE=1` breaks it.

## What holds

- Free flight (contact-free, joint-free, `tick_v0` present): analytic
  `v(t),x(t)` Verlet-exact; otherwise symplectic Euler `v+=a·dt, x+=v_new·dt`.
  Degraded (NULL scratch) takes symplectic, never analytic (no double-apply).
- Rotation: exp-map rotor `q(dt)=[cos(|w|dt/2), w/|w|·sin]⊗q`, renormalized.
  Torque-free: L fixed-point (8 passes, falls through); converged 0.0028%/2 s
  (gate 0.5%), fallback ~2.7%/2 s. Sphere control in `angmom` settles the
  cube tension.
- Inertias exact: sphere `(2/5)mr²`, box `(m/12)(h²+d²)`, cylinder axial
  `½mr²` + transverse `(m/12)(3r²+L²)`, axle X.
- SAT-15 + S-H clip exact; slop admission `pen≥-slop→0` (Box2D linearSlop).
  Candidate 4-point max-area greedy is heuristic (keeps 4, measured).
- Sequential impulse exact per Catto; every manifold visited 2× per iteration
  (knob undercounts 2×). No warm start on any row (cold normal, cone-killed
  tangent); 96-128 iters for 10-high stack (12-30× Catto 4-8). Buckling guard
  adds ≤32 extra when `bodies≥8` and residual>slop (capped 128).
- Coulomb: isotropic disc exact; ellipse strict generalisation; rail exact
  zero-set. Combined `min(μa,μb)` (NOT Box2D `√`), up to 42% lower. Tick-start
  `snap_friction_mu`; breakaway 0.999-1.005 μs·N across dt/iters/μ.
- Poisson `e·C` + Newton bound, impact-refresh post-integration, paid once.
- Split impulse translational-only (deepest-only, mass-weighted, slop-gated,
  β-scaled, capped, no velocity touch). Per-contact angular tried twice,
  measured worse, reverted. Deep containment (>0.30 m) moves lighter only.
- CCD: q-form TOI + floor quadratic + slab sweep + symmetric two-phase +
  analytic chaining. Mover tip `|w|·R` + world obstacle margin
  `max|w|·R` (capped 2 m/s). Obstacle rotation beyond the margin (fast blade)
  unsupported: substep or bound tip speed externally.
- Rolling: Hertz-patch + spin, mass-weighted split is TUNE (band 4-14 m only).
- Sleep: three-gate wake (pair novelty + fast-other + deep); settled stacks
  sleep, slow pushers wake at any speed.
- Broadphase O(n) hash, swept AABB + tip-speed, 1M node cap + overflow
  telemetry (nonzero = do not trust the tick). SIMD SSE2 fast path is
  bitwise identical (self-checked); scalar canonical.
- Threading: `MPE_THREADS` (1-8, default 1) parallelises order-independent
  per-body sanitize only; pairs/solve single-threaded for identical order.
  Bitwise identical MT vs serial.
- Hash `physics_world_hash_state()` covers pos/vel/orient/ids ONLY (spin,
  mass, sleep, joints omitted; translation lockstep only). CRC32 table
  `pthread_once`.

## What does NOT hold (unsupported, gated or documented)

- Rotation-only tunneling beyond the obstacle margin (blade regime).
- Tall-stack buckle transient under extreme gravity (chaotic, ≤0.69 m;
  adaptive iterations mitigate, not solve).
- Normal/tangent warm-start impulse carry (dead by measurement; frame
  adoption live only).
- `max_linear/angular_speed` clamps (inform-only counters).
- Slop-band bounce (`pen∈[-slop,0)` friction-only, no bounce).
- Spring pure-Hooke unless `max_acceleration=10000` + `k<k_stable` + no warn.
- MSVC bitwise twins (needs `/fp:strict` proof + TSan).
- Multithreaded solve (pairs/solve serial by design; MT sanitize only).

## Budgets (measured, not wished)

- 10-cube default gravity: 0.1134@32 → 0.0145@64 → 0.0000@96/128.
- Gravity -17: 0.2084@64 → 0.3105@64 (buckle) → 0.0024@96 → 0.0000@128.
- Friction breakaway 0.999-1.005 μs·N (dt 1/30-1/120, iters 8-128, μ 0.6-0.9).
- Pendulum/spring periods ≤1.5%/0.8%; free-fall/projectile ≤5e-5; inertias
  ≤2e-5 abs; rolling 4-14 m; angmom ≤0.5% (converged 0.0028%).
