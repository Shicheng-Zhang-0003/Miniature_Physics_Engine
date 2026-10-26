# MPE Physics — v15S

> Every equation, integrator, solver stage, and collision test in the engine,
> derived and stated exactly as implemented. This is the authoritative
> reference for the physics model; `docs/NUMERICAL_GUARANTEES.md` gives the
> measured budgets and what does / does not hold.

---

## 1. Timestep & Determinism

- Fixed canonical dt = `1/60 s` (0.016667). The GUI loop is fixed 60 Hz;
  headless callers must pin dt = 1/60.
- Non-canonical dt is stepped (no breakage) but loud: a one-time stderr
  warning. Variable-dt callers silently desync (damping `pow(drag, dt)`,
  CCD sweep × dt, rotor |ω|·dt/2).
- Fixed solver iteration order, exact IEEE `+ - * / sqrt`, fixed-coefficient
  polynomials (`det_math.h`), `-ffp-contract=off`. Bitwise-identical twins
  on GCC/Clang IEEE targets over 600 ticks (proven by `determinism` test).
- Fallback counters (`det_fallback_*_total()`) count every libm transcendental
  fallback so desync is diagnosable, never silent. Asserted zero in-contract.

---

## 2. Integration

### 2.1 Linear motion — semi-implicit (symplectic) Euler

```
v_new = v_old + a·dt
x_new = x_old + v_new·dt
```

where `a = F/m + g` (gravity applied as `gravity × mass` force).

### 2.2 Exact free flight (Verlet-exact)

For contact-free, joint-free bodies with `tick_v0` present, position
integration uses the analytic gravity+drag solution starting from `v_pre`
(start-of-tick velocity, before forces). This is Verlet-exact free flight:
a contact-free projectile follows an exact parabola.

```
v(0) = v_pre          (restored from tick_v0)
x(t) = x_pre + v_pre·t + ½·g·t²   (drag-free analytic)
```

Constrained bodies (contact or joint) use plain symplectic Euler — their
acceleration is canceled by contact impulses post-solve, so correcting with
pre-solve gravity would pump them out of slop.

**`tick_v0` contract:** `free_flight` is FALSE unless the `tick_v0` restore
actually ran. Degraded/low-mem ticks take the safe symplectic path (no
double-applied gravity).

### 2.3 Angular motion — quaternion integration

```
q(dt) = [cos(|ω|·dt/2), (ω/|ω|)·sin(|ω|·dt/2)] ⊗ q
```

Renormalized each tick (accumulated in double to avoid overflow-to-Inf for
large components). No gimbal lock.

### 2.4 Damping

Linear retention: `drag_clamped` clamped to `(0, 1.1]`, then
`linear_damping = det_pow_retention(drag_clamped, dt)` — deterministic
fixed-polynomial, never libm `pow`.

Angular: `scale = 1.0` means NO extra rotary damping (retention exactly 1.0
even when drag < 1 damps translation). Air barely damps rotation; coupling
them (old `pow(drag × scale)`) damped spin in vacuum even at drag = 1.

```
angular_damping = (ang_scale >= 1.0) ? 1.0 : det_pow_retention(drag × ang_scale, dt)
```

### 2.5 Torque-free angular momentum

Fixed-point solve: `ω = I(rotor(w, dt)·R)⁻¹·L` against the exactly-conserved
world-frame `L`. 8 passes, falls through. Converged drift 0.0028% over 2 s
(gate 0.5%); fallback ~2.7%/2 s.

Gyro uses **explicit-midpoint** (RK2-split: midpoint in external torque,
Euler in gyro — not implicit despite the old comment); torque-free degenerates
to explicit (the L path is what conserves). Gyro clamp `4·|ω|/dt` binds for
needles at high spin (non-conservative cut there).

---

## 3. Collision Pipeline

### 3.1 Broadphase — 3D spatial-hash grid

Objects mapped into hashed grid buckets; collision checks limited to local
neighborhoods for average O(N) scaling. Cell size adapts to object radii.
Node pools start small (4K) and double to a 1M cap with overflow telemetry
(nonzero overflow = do not trust the tick).

### 3.2 Narrowphase

| Pair | Method |
|---|---|
| Sphere–Sphere | Analytical distance test |
| Sphere–OBB | Closest-point projection |
| OBB–OBB | SAT-15 + Sutherland–Hodgman face clipping |
| Cylinder–* | Exact solid-cylinder SDF (caps, rim, inside) / segment-OBB / coaxial |
| Custom | Foreign shapes via pair registry (`mpe_register_pair_handler`) |

All narrowphase functions take the owning world's config; dispatch is
registry-first with builtin fallback (one shared implementation — no
duplicated per-path chains).

### 3.3 Contact manifolds

Multi-point Sutherland–Hodgman with support-first ordering for stable
stacking. Contact points stored in **body-local space** (rigid translation
preserves them exactly — persistent contact matches the same material point
next tick regardless of where the body moved).

Warm-start match: strict both-side material-point coincidence
(`contact_cache_match_role`). Both local positions must coincide within
`warm_start_match_dist_sq` (default 1 cm²). Side-A-only matching aliased
two B bodies sharing an A anchor within 5 cm (wrong impulse injection) —
fixed.

### 3.4 Three-gate wake

Any gate fires → wake:

1. **Pair novelty** (contact cache): either id pair has no entry from a
   prior tick. First touch wakes at ANY speed. Persistent resting pairs have
   entries, so settled stacks proceed to sleep instead of being kept awake
   forever (this pumped the F10 10-stack to 13 m/s via never-sleeping
   micro-motion — the F10 root cause, fixed).
2. **Fast-other**: original velocity gate (backstop for fast bodies that
   already held contacts, e.g. debris sliding along a stack before striking
   a sleeper).
3. **Deep overlap**: overlap beyond `wake_depth_thresh`, even vs statics.

---

## 4. Solver

### 4.1 Sequential impulse

Impulse-based sequential solver, 64 iterations by default (configurable
1–128). The 64 knob is really **128 manifold visits per iteration** — every
manifold is visited twice per iteration. Convergent (no energy), just
expensive.

Per contact, per visit:

**Normal (compression only — no restitution bias here; Poisson pass later):**

```
v_rel = (v_b + ω_b × r_b) − (v_a + ω_a × r_a)
v_n   = v_rel · n
λ_n   = −v_n · m_eff_n
m_eff_n = 1 / (inv_m_a + inv_m_b + ((I_a⁻¹(r_a × n)) × r_a + (I_b⁻¹(r_b × n)) × r_b) · n)
accum_n = clamp(accum_n + λ_n, 0, ∞)          // pure compression, never negative
Δλ_n = accum_n − accum_n_old
apply impulse Δλ_n · n to both bodies (linear + angular)
```

**Friction:**

```
rel_vel_t = rel_vel − n·v_n
tangent frame: slip direction if sliding, adopted/remembered direction if sticking
v_t1 = rel_vel · t1,  v_t2 = rel_vel · t2
λ_t1 = −v_t1 · m_eff_t1
λ_t2 = −v_t2 · m_eff_t2
Coulomb clamp on combined (λ_t1, λ_t2) vector:
  isotropic: disc of radius μ·Fn
  anisotropic: ellipse with semi-axes μ_along·Fn, μ_across·Fn (material frame)
apply clamped friction impulse
```

Stick/slip selection on **combined slip speed** vs `static_friction_thresh`
(default 0.02 m/s): below thresh the slip direction is micro-motion noise,
so a resting contact uses the remembered tangent direction or no frame at
all (normal-only this tick — the sweep cannot invent a hold direction from
noise). True sliding keeps the slip direction.

**Stick/slip μ selection** is snapshotted once per contact from pre-force
tick-start slip (`snap_friction_mu`, recorded in `collision_prepare_solver`)
instead of re-decided on per-iteration solver transient — measured breakaway
0.999–1.005 of μ_s·N across dt (1/30–1/120), iterations (8–128) and μ
(0.6–0.9), closing the old 13.9%-low-at-60 Hz `[FRICTION-THRESH]`.

**Combined μ = min(μ_a, μ_b)**, NOT Box2D `√(μ_a·μ_b)`. min ≤ sqrt, up to
42% lower grip (0.30 vs 0.52 for 0.9/0.3). Conservative and stable,
consistently applied.

### 4.2 Warm start — status

**No effective warm start on any row** — measured, not assumed.

- **Normal:** cold every tick. Restoring last tick's normal as the iterations'
  seed ejected the 10-stack at ~13 m/s across ablations, while the restored
  values stayed healthy (~0.5, bounded). The clamp stays.
- **Tangent:** the restored tangent was projected onto the Coulomb cone at
  application time using the **cold** normal as `fn` — so `fn` was exactly
  0.0 and `a3_anisotropic_coulomb_clamp()` returned both tangents as zero.
  Instrumented on a 4-cube stack at rest: restored t1 = 0.154651, fn =
  0.000000, after clamp t1 = 0.000000. The warm start was deleted by the
  very clamp added to protect it.
- **Tangent FRAME adoption** is live and helps; only the impulse seed is dead.

**Damped warm-start** (`solver.warm_start_damping`, default 0.0): nonzero
damping scales the cached normal impulse and uses it as the cone's `fn`,
restoring cross-tick carry. Cold (0.0) is the shipped default — stable
across all five regimes. Nonzero damping is experimental: faster convergence
at low iteration counts, unstable under extreme gravity/friction/restitution.
Full restore (1.0) is a measured regression (f10 drops from 27/27 asleep to
4/27; f11 overlap 0.2791 m vs 0.05 m bound).

**Consequence:** with no cross-tick information carry, the whole column is
re-propagated from the floor every tick, which is why this engine needs
**96–128 iterations for a 10-cube stack — 12–30× the published practitioner
budget** (Catto, Solver2D 2024: "typically 4 to 8 iterations"). That is the
real outstanding physics question, recorded in `docs/VALIDATION.md`.

Primary source (verified 2026-10-24): Erin Catto,
[Solver2D](https://box2d.org/posts/2024/02/solver2d), box2d.org, published
2024-02-05 — "iteration count: solvers generally use a fixed number of
iterations. The number is generally determined offline. In game physics
there are typically 4 to 8 iterations." The same post defines warm starting
("feeding the impulses/forces of previous time steps to the next time step")
and its reference PGS solver (Box2D-Lite, 2006) "uses warm starting and
Baumgarte Stabilization" — i.e. the 4–8 budget assumes a warm-started
solver. This engine's 96–128 is the measured COLD-solver cost of the same
PGS-class algorithm, so the comparison is directional, not like-for-like;
closing the gap means promoting the damped warm start above through
per-regime validation, not retuning the iteration ceiling.

### 4.3 Poisson restitution

One `e`-over-compression payment per fresh impact, then short relaxation so
friction sees post-bounce velocities. Paid once (`e·C` + Newton bound,
impact-refresh post-integration). No velocity Baumgarte — position-correction
energy cannot leak into the velocity state and be re-read by the Poisson
restitution gate (Catto, GDC 2014 p.52–53).

### 4.4 Split impulse

**Whole-body translation, not per-contact correction with lever arms** —
measured, not assumed. Catto GDC 2014 p.53 and Box2D 2.x apply it per contact
with an `r × n` angular term, and centre-only translation genuinely cannot
right a tilt. Implementing it per-contact (twice: independent pass, and
shared Gauss–Seidel with accumulated per-point impulse) made the 10-high
pile **4.2× worse** (0.0537 m → 0.2253 m overlap, ω 0 → 1.9217 rad/s) and
remained a net regression across the F11 seed sweep even after over-correction
was fixed (sum 2.42 → 2.95). Reverted; negative result recorded in source.

Split impulse is positional depenetration with **zero velocity change**:
the velocity solve above is compression-only, so contact impulses (and the
Coulomb clamp) stay honest. Deepest-only, mass-weighted, slop-gated,
β-scaled, capped at `max_separation_bias × dt`.

Runtime caps (slop 0..0.05 m, beta 0..1, bias velocity ≤ 10 m/s) are
COUNTED (`mpe_clamp_solver_events`, bounded stderr) — a substitution you
can see. Step `dt > 0.1` clamp likewise counted (time dilation under lag,
documented).

### 4.5 Rolling resistance

Hertz-patch + spin, mass-weighted split. N is post-Poisson (bouncers draw
from restitution impulse). 0.5 mm `pen_eff` floor manufactures overlap for
zero-depth spin decay. Band 4–14 m only (TUNE).

### 4.6 Buckling guard

Tall stacks (≥ 8 bodies) with residual deep overlap after budgeted iterations
get up to 32 extra iterations (capped 128 total). Cheap when settled
(worst ≤ slop skips), pays only during buckle transients. Measured lever:
10-cube at gravity −17 goes 0.31 m@64 → 0.00 m@128.

---

## 5. Constraints & Joints

### 5.1 Supported types

| Type | Solves | UI creation | Persists v200 |
|---|---|---|---|
| Spring | Hooke + damping + Courant guard | object menu link | ✅ |
| Revolute (+limits, motor, drift fix) | ✅ | scene/terminal | ✅ |
| Fixed / distance | ✅ | ❌ | ✅ |
| Prismatic (+limits, motor) | ✅ | ❌ | ✅ |
| Rope (pull-only) | ✅ | ❌ | ✅ |

All six live in the headless suite and `mpe-tui --scene demo`.

### 5.2 Revolute

Pre-step integrates angle/slide once per tick (never per iteration). Solve
inside the iteration loop for friction transfer through hinges. Axis-drift
correction (positional Baumgarte) once per tick after the loop — never
inside it (positional error is iteration-invariant; inner calls multiply the
correction by the iteration count = energy pump). Angle limits tracked once
per tick with velocity-level enforcement.

### 5.3 Motor

Torque → friction → translation (grounded, coupled). Driven-wheel moved into
the resolvable spin regime (0.080 N·m) with load-bearing gates: grounded
height, rolling coupling 0.20–1.15, spin cap. (π-rad/tick contact is
undefined for any discrete scheme — the old torque "passed" at y=488 lunacy.)

Jointed air-spin limit cycle: free-spinning jointed wheels oscillate (motor
86 rpm vs true wheel 799 rpm) via revolute-to-kinematic-chassis impulses vs
slew/governor/implicit-solve. Contained (diode+slew bound it; airborne
bypass tried and reverted after runaway to ±1900 rpm). Bearing damping
c = 1e-6 N·m·s in air only (c·dt/I ≈ 0.067 < 1, no runaway).

---

## 6. CCD — Continuous Collision Detection

Translation-only by design (spin never triggers floor CCD — a pure spinner
about Y has tangential motion only, no false clamps, no spin pump).

### 6.1 Floor plane (y = 0)

Exact quadratic CCD under constant gravity:

```
0.5·g·t² + v₀·t + y₀ = 0
```

where `g` = gravity (negative), `y₀` = lowest-point height (center − support
depth), `v₀` = lowest-point velocity `v_center + (ω × r_lowest)·ŷ`
(ROT-CCD: includes rotational lowest-point velocity so a tumbling box's
dipping corner clamps before tunneling).

Solved unconditionally (no v₀ sign gate) in double to avoid float
cancellation. Only `toi ∈ (0, dt)` clamps.

### 6.2 Volumes

Spheres, boxes (static AND dynamic via relative velocity in obstacle frame),
cylinders and custom shapes (conservative bounding spheres where exact sweep
geometry unavailable). q-form TOI + cancellation-resistant roots. Mover tip
speed `|ω|·R` + world obstacle margin `max|ω|·R` (capped 2 m/s).

Overshoot bound `≤ 0.5·g·toi² < slop` holds at canonical dt only
(0.25 m at dt = 0.1 + g ≈ 50).

**Rotation-only tunneling between arbitrary volumes is unsupported** —
substep or bound tip speed externally for fast blades / propellers.

---

## 7. Sleep

Three-gate wake:

1. **Pair novelty** via contact cache (fires at any speed).
2. **Fast-other** velocity gate (backstop).
3. **Deep overlap** (backstop, even vs statics).

Persistent resting contacts no longer veto sleep. Settled stacks proceed to
sleep and stay settled; slow pushers wake sleepers at any speed. Sleep is a
NON-PHYSICAL optimization — `sleep.enable = 0` runs sleepless validation
(solver alone must settle).

Sleeping bodies keep real mass/inertia (no staticize mutation); the solver
treats them as infinite mass via `effective_*` helpers, so observable state
is never corrupted mid-tick and the path is thread-safe. Velocities pinned
at zero.

---

## 8. Numerical Guarantees Summary

| Property | Guarantee |
|---|---|
| Free flight | Verlet-exact (contact-free, joint-free, tick_v0 present) |
| Rotation | exp-map rotor, renormalized, double accumulation |
| Inertias | sphere ⅖mr², box m/12(h²+d²), cylinder axial ½mr² + transverse m/12(3r²+L²) |
| Narrowphase | SAT-15 + S-H clip exact; slop admission pen ≥ −slop → 0 |
| Solver | sequential impulse exact per Catto; 2× visits per iteration |
| Warm start | cold by default; damped experimental |
| Split impulse | whole-body translation (per-contact angular measured worse) |
| Poisson | e·C + Newton bound, paid once |
| CCD | floor quadratic + volume sweep; rotation-only tunneling unsolved |
| Sleep | three-gate wake; sleep is non-physical optimization |
| Determinism | fixed dt + fixed order + exact IEEE + fixed polys + -ffp-contract=off |
| Bitwise twins | GCC/Clang only; MSVC needs /fp:strict (unproven) |

Measured budgets: 10-cube default gravity 0.1134@32 → 0.0145@64 → 0.0000@96/128;
friction breakaway 0.999–1.005 μs·N; pendulum/spring periods ≤1.5%/0.8%;
free-fall/projectile ≤5e-5; inertias ≤2e-5 abs; rolling 4–14 m; angmom ≤0.5%
(converged 0.0028%).
