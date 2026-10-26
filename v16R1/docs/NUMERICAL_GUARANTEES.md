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
  cube tension. Gyro is EXPLICIT-midpoint (RK2-split: midpoint in external
  torque, Euler in gyro — not implicit despite the old comment); torque-free
  degenerates to explicit (the L path is what conserves). Gyro clamp
  `4·|w|/dt` binds for needles at high spin (non-conservative cut there).
  L-tolerance scales with `|L|`, not `|Δw|` (heavy/slow pass early).
- Inertias exact: sphere `(2/5)mr²`, box `(m/12)(h²+d²)`, cylinder axial
  `½mr²` + transverse `(m/12)(3r²+L²)`, axle X.
- SAT-15 + S-H clip exact; slop admission `pen≥-slop→0` (Box2D linearSlop).
  Candidate 4-point max-area greedy is heuristic (keeps 4, measured) AND
  degenerate at keep_count==1 (centroid==keep[0] → all areas 0 → clip-order
  pick; farthest-point reseed ticketed). Parallel-edge 2nd contact gated by
  `spread>0.05` (small boxes ≤0.1 stay single-point — scale-dependent tip).
- Sequential impulse exact per Catto; every manifold visited 2× per iteration
  (knob undercounts 2×). No warm start on any row (cold normal, cone-killed
  tangent); 96-128 iters for 10-high stack (12-30× Catto 4-8). Buckling guard
  adds ≤32 extra when `bodies≥8` and residual>slop (capped 128).
- Coulomb: isotropic disc exact; ellipse strict generalisation; rail exact
  zero-set. Combined `min(μa,μb)` (NOT Box2D `√`), up to 42% lower. Tick-start
  `snap_friction_mu`; breakaway 0.999-1.005 μs·N across dt/iters/μ.
  In-tick stick/slip transitions mis-solve by up to `(μs−μk)·Fn` (whole-tick
  freeze is the documented hack; per-phase re-snapshot ticketed).
- Poisson `e·C` + Newton bound, impact-refresh post-integration, paid once.
- Split impulse translational-only (deepest-only, mass-weighted, slop-gated,
  β-scaled, capped, no velocity touch). Per-contact angular tried twice,
  measured worse, reverted. Deep containment (>0.30 m) moves lighter only.
  Runtime caps (slop 0..0.05, beta 0..1, bias ≤10) are COUNTED
  (`mpe_clamp_solver_events`, bounded stderr) — a substitution you can see.
  Step `dt>0.1` clamp likewise counted (time dilation under lag, documented).
- CCD: q-form TOI + floor quadratic + slab sweep + symmetric two-phase +
  analytic chaining. Mover tip `|w|·R` + world obstacle margin
  `max|w|·R` (capped 2 m/s). Obstacle rotation beyond the margin (fast blade)
  unsupported: substep or bound tip speed externally. Overshoot bound
  `≤0.5·g·toi²<slop` holds at canonical dt only (0.25m at dt=0.1+g≈50).
- Rolling: the normal force uses the compression-only impulse snapshot taken
  before Poisson restitution (fixed 2026-10-20). Hertz-patch naming is avoided:
  spin patch and mass-weighted sharing are empirical tunes (4-14 m calibration
  band only); the 0.5mm `pen_eff` floor remains to decay resting spin.
- Cylinder contacts: slab shortcut falls back to the OBB path when a cylinder
  spans both slab faces, allowing the shallower exit; cylinder-cap support
  points are clamped into the common disc lens; parallel-edge contact sampling
  no longer has a 0.1 m size cutoff (fixed 2026-10-20).
- Joints: `joints.revolute_beta` is the SHARED anchor bias (revolute P2P +
  prismatic-perp + rope + distance); axis/fixed drift use fixed 0.1 by design
  (decoupled).
- Sleep: three-gate wake (pair novelty + fast-other + deep); settled stacks
  sleep, slow pushers wake at any speed. Post-correction wake uses the same
  `wake_depth_thresh` knob as pre-wake (single meaning, since 2026-10-13).
- Broadphase O(n) hash, swept AABB + tip-speed, 1M node cap + overflow
  telemetry (nonzero = do not trust the tick). SIMD: full lane-wise op set
  (add/sub/scale/cross, SSE2/MSVC-intrin/scalar-fallback) wired through all
  physics hot paths; bitwise identical by construction (lane-exact, no FMA;
  reductions stay scalar by proof) and by runtime self-check (fixed + edge
  vectors incl. Inf/NaN) with zero mismatches; both SSE and scalar-fallback
  binaries pass every gate (`build_suite_scalar`). Measured: op micro-bench
  wrappers 2.1x slower than scalar in a synthetic tight loop (round-trips),
  engine suite neutral (wired ~3% ahead, in noise). True 4-wide SoA batching
  is the remaining speedup frontier, not per-op wrapping.
- Threading: `MPE_THREADS` (1-8, default 1) parallelises per-body sanitize
  and integration, the two-phase CCD sweep, pair-local narrowphase detection,
  and solver sweeps across disjoint islands. Shared-body wake/contact
  bookkeeping and manifold preparation stay serial in pair order; the
  Gauss-Seidel solve stays serial within each island. Concurrent world steps
  share a process-wide pool whose dispatch phases are serialized. Determinism
  twins have been verified at 1/4/8 threads.
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
- MSVC bitwise twins (needs `/fp:strict` proof).
- 0.5mm rolling spin-patch floor is empirical; removing it previously stopped
  resting spin from decaying. Warm-start remains cold by default.
- Within-island parallel contact solving; current island sweeps preserve
  serial Gauss-Seidel order inside each island.

## Budgets (measured, not wished)

- 10-cube default gravity: 0.1134@32 → 0.0145@64 → 0.0000@96/128.
- Gravity -17: 0.2084@64 → 0.3105@64 (buckle) → 0.0024@96 → 0.0000@128.
- Friction breakaway 0.999-1.005 μs·N (dt 1/30-1/120, iters 8-128, μ 0.6-0.9).
- Pendulum/spring periods ≤1.5%/0.8%; free-fall/projectile ≤5e-5; inertias
  ≤2e-5 abs; rolling 4-14 m; angmom ≤0.5% (converged 0.0028%).
