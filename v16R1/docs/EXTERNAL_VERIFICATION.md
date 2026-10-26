# MPE — External Truth Verification

> Consolidated record of every physics equation and outcome checked against an
> authority that is **not this project**. Companion to `docs/VALIDATION.md`
> (the audit narrative) and `docs/NUMERICAL_GUARANTEES.md` (the measured
> budgets). Every number below was measured on the shipped engine, compared
> against a closed-form textbook reference or an independent from-scratch RK4
> oracle, and is reproducible from the canonical suite.

## Reference authorities (vendored in `reference_materials/`)

| Paper | What it covers in MPE |
|---|---|
| **Mirtich 1995/1996** — *Impulse-based Simulation of Rigid Bodies* | Sequential-impulse solver lineage; effective mass; Coulomb friction cone |
| **Baraff** — rigid-body dynamics | Contact & constraint formulation; inertia tensor transform `R·I⁻¹·Rᵀ` |
| **Anitescu** — fixed-timestep multibody dynamics | Fixed-timestep contact/friction complementarity |
| **Catto GDC 2005/2011/2013** — iterative dynamics, temporal coherence, soft constraints | Sequential impulse; β (ERP) bias `bias = (β/h)·C`; warm-start |
| **Gottschalk 1996** — OBBTree / SAT | 15-axis separating-axis theorem for OBB/OBB |
| **Coumans 2005** / **Schmidl 2004** — continuous collision detection | q-form TOI; swept-sphere quadratic; slab sweep |
| **Teschner 2003** — optimized spatial hashing | 3D spatial-hash broadphase |
| **CGPM 1901 / CODATA 2022** — `g_n = 9.80665 m/s²` (exact by convention) | Gravity constant; reported default is −9.81 (+0.0342% off) |

Constants were fetched, not remembered. `g_n` is exact **by convention**, not
an SI defining constant — the CGPM 1901 Declaration 2 adopted a value; it did
not define `g_n` in terms of `h`.

---

## 1. Mass properties — exact on all three primitives

Computed from closed-form inertia tensors and compared bit-for-bit against the
engine's `inverse_inertia_tensor_local`. All zero off-diagonal; `inv(I)·I = I`.

| shape | axis | closed-form law | measured error |
|---|---|---|---|
| sphere | x, y, z | `I = (2/5)·m·r²` | **0.00%** |
| sphere | — | `inv_mass = 1/m` | **0.00%** |
| sphere | — | `inv(I)·I = identity` | **0.00%** |
| box | x, y, z | `I = (m/12)(h² + d²)` | **0.00%** |
| box | — | half-extents preserved verbatim | **0.00%** |
| cylinder | axial (X) | `I = (1/2)·m·r²` | **0.00%** |
| cylinder | transverse (Y, Z) | `I = (m/12)(3r² + l²)`, axle = X | **0.00%** |
| cylinder | — | `half_extensions = (half_length, r, r)` | **0.00%** |

Volumes (`4/3πr³`, `8·hx·hy·hz`, `πr²l`) and implied densities are self-consistent
to **0.00%**. The transverse-cylinder axis previously had no coverage anywhere
(MFS flagged it as an open gap); it is now correct and gated.

**Verdict:** mass properties are exact textbook laws, verified against the
reference. No sign error, missing term, or unit error.

---

## 2. Rotation algebra — exact to float precision

`float32 eps = 1.1920929e-07`. Every deviation is at or below the rounding
budget for the format.

| property | worst deviation | budget |
|---|---|---|
| axis-angle quaternion is unit norm | 1.79e-07 | 4 × eps |
| rotation matrix orthonormal, `RᵀR = I` | 3.58e-07 | 8 × eps |
| rotation matrix `det = +1` (proper, not reflection) | 3.58e-07 | 8 × eps |
| quaternion rotation == classical Rodrigues angle-axis | 1.98e-07 | 4 × eps |
| quaternion composition `q(a₂)·q(a₁) = q(a₁+a₂)` | 1.19e-07 | 2 × eps |
| `cached_axes[i] == R · local_axis_i` | **0.00%** | exact |
| cached axes mutually orthogonal, unit length | ≤7.45e-09 | ≤2 × eps |

Orientation is integrated with an exp-map rotor
`q(dt) = [cos(|ω|dt/2), (ω/|ω|)·sin(|ω|dt/2)] ⊗ q`, renormalized each tick
with double accumulation. No gimbal lock.

**Verdict:** rotation algebra matches textbook Rodrigues/angle-axis exactly
within float32 rounding.

---

## 3. Integration — closed-form comparisons

| property | reference | error | note |
|---|---|---|---|
| semi-implicit Euler vertical fall `y_n = y₀ − g·dt²·n(n+1)/2` | closed form | 0.10% | viscous drag component |
| no horizontal drift in free fall | 0 | **0.00%** | |
| angular `α = τ/I`, `ω` after 1 s (world-frame inertia) | closed form | **0.00%** | |
| quaternion stays unit under integration | — | within budget | renormalized each tick |
| **angular momentum conserved** (sphere, isotropic inertia) | `L = Iω` | **0.00%** | sphere control; cube is `NOT_A_CHECK` (anisotropic top precesses) |
| energy conservation over a 19.5 m fall | `E₀` | 0.26% | drag accounts for the deficit |

**Torque-free angular momentum:** fixed-point solve `ω = I(rotor(w,dt)·R)⁻¹·L`
against the exactly-conserved world-frame `L`, 8 passes. Converged drift
**0.0028% over 2 s** (gate 0.5%); fallback path ~2.7%/2 s. The L-path is what
conserves; the gyro term is explicit-midpoint (RK2-split: midpoint in external
torque, Euler in gyro — **not** implicit despite the old comment).

**Verdict:** semi-implicit Euler + exp-map rotor + exact free-flight
(Verlet-exact for contact-free, joint-free bodies with `tick_v0`) match
closed-form mechanics within stated budgets.

---

## 4. Narrowphase — Gottschalk 1996 SAT, 400/400

OBB/OBB uses the exact 15-axis theorem: 3 + 3 face normals plus all 9 cross
products `A_i × B_j`, with
`overlap = proj_a + proj_b − |dot(t, n)|` and rejection on `overlap < −slop`.

Checked against an **independent dense reference** written from the theorem
statement (15 canonical axes + a 240-direction swept probe, its own radius
projection — not the engine's `project_obb`). Over **400 randomised yawed
configurations** the engine and the reference agree on overlap/no-overlap
**400 of 400**. Degenerate (parallel-edge) cross products are skipped, which is
correct — already covered by the face axes.

Contact *points* (not just the overlap decision) are covered by
`v15S/validation/property_sweep.py` laws (SAT symmetry, slop admission) plus
`reference_math` depth spot-checks. Full closest-point-pair vs Gottschalk
distance numeric agreement remains a declared differential future.

**Verdict:** SAT overlap decision is exact per Gottschalk 1996, 400/400.

---

## 5. Solver — Catto sequential impulse

### 5.1 Effective mass and normal impulse

Per contact:
```
v_rel = (v_b + ω_b × r_b) − (v_a + ω_a × r_a)
v_n   = v_rel · n
λ_n   = −v_n · m_eff_n
m_eff_n = 1 / (inv_m_a + inv_m_b + ((I_a⁻¹(r_a × n)) × r_a + (I_b⁻¹(r_b × n)) × r_b) · n)
accum_n = clamp(accum_n + λ_n, 0, ∞)   // pure compression, never negative
```
This is the Catto sequential-impulse normal solve, compression-only (restitution
is a separate Poisson pass). Effective mass matches `1/k_normal` from
`rigidbody_effective_inv_mass` + inertia transform.

### 5.2 Friction — Coulomb disc, min(μa, μb)

Tangent frame from slip direction (sliding) or remembered tangent (sticking);
below `static_friction_thresh` the slip direction is micro-motion noise and a
resting contact uses the remembered frame or normal-only. Combined slip speed
vs threshold selects stick/slip.

Coulomb clamp on the combined `(λ_t1, λ_t2)` vector:
- isotropic: disc of radius `μ·Fn`
- anisotropic: ellipse with semi-axes `μ_along·Fn`, `μ_across·Fn` (material frame)

**Combined μ = min(μa, μb)**, **NOT** Box2D's `√(μa·μb)`. min ≤ sqrt, up to
42% lower grip (0.30 vs 0.52 for 0.9/0.3). Conservative, stable, consistently
applied. Disclosed everywhere.

Tangent-frame adoption uses the material axis for anisotropic contacts
(`a3_anisotropic_tangent_frame`), so the elliptical cone is measured against
material axes — a slip-aligned basis would collapse it to a magnitude limit
that cannot express a rail.

### 5.3 β (ERP) bias — Catto GDC 2011, monotonic

`revolute_joint.c` implements `bias = (β/h)·C` — precisely Catto's "β feeds the
position error back to the velocity". Configured β = 0.3, inside Catto's [0,1].

| β | position error retained after 60 ticks |
|---|---|
| 0.0 | 2.4667 (velocity constraint only, exactly Catto) |
| 0.1 | 1.5667 |
| 0.3 | 0.7833 |
| 0.8 | 0.3567 |

Monotonically faster convergence with rising β, as the formulation requires.

### 5.4 [FRICTION-THRESH] — closed 2026-10-04

Before the fix, the static threshold was timestep-dependent (Coulomb's law is
rate-independent by definition). Bisected threshold for μ_s = 0.6, 1 kg block,
`N = m·g_n`:

| dt | old threshold F/(μ_s·N) | deviation |
|---|---|---|
| 1/240 | 0.99956 | 0.04% |
| 1/120 | 0.99751 | 0.25% |
| **1/60** | **0.86128** | **13.87%** |
| 1/30 | 0.75761 | 24.24% |

Root cause: memoryless per-iteration stick/slip selection re-evaluated on live
per-iteration slip (solver transient, not physics state). Fix:
`snap_friction_mu` recorded once per contact in `collision_prepare_solver`
(pre-force-integration, sees the tick's opening velocities) and reused by every
iteration.

| condition | threshold / μ_s·N | effective μ_s |
|---|---|---|
| dt = 1/60, iters 8 | 0.99485 | 0.5969 |
| dt = 1/60, iters 32/64/128 | 1.00038–1.00045 | 0.6002–0.6003 |
| dt = 1/120, iters 64 | 1.00547 | 0.6033 |
| dt = 1/30, iters 64 | 0.99920 | 0.5995 |
| μ_s = 0.9 (cone 8.83 N) | 0.99921 | 0.8993 |

Iteration-independent, timestep-independent, μ-scaling. **Verdict:** Coulomb
sliding branch worst **0.07%** (mean 0.05%) vs closed form
`(F − μ_k·N)/m` with `g = 9.80665`.

---

## 6. Coulomb sliding — 0.07% worst case

Once sliding, the block accelerates at exactly `(F − μ_k·N)/m`, `g = 9.80665`.
Measured after 0.5 s:

| F/(μ_s·N) | measured v | predicted | error |
|---|---|---|---|
| 1.10 | 1.27570 | 1.27486 | 0.07% |
| 1.30 | 1.86418 | 1.86326 | 0.05% |
| 1.60 | 2.74687 | 2.74586 | 0.04% |

**Verdict:** sliding branch matches closed-form Newtonian acceleration within
0.07%.

---

## 7. CCD — q-form TOI + floor quadratic

Floor plane (y=0), exact quadratic under constant gravity:
```
0.5·g·t² + v₀·t + y₀ = 0
```
where `v₀ = v_center + (ω × r_lowest)·ŷ` (rotational lowest-point velocity, so
a tumbling box's dipping corner clamps before tunneling). Solved unconditionally
in double to avoid float cancellation; only `toi ∈ (0, dt)` clamps.

Swept-sphere q-form TOI uses cancellation-resistant roots (`q = -0.5·(b +
copysign(root, b))`), plain double mults (no FMA, `-ffp-contract=off`) for
bit-identical cross-target results.

Symmetric two-phase clamp: Phase 1 computes all TOIs vs OLD positions; Phase 2
moves all simultaneously. Order-independent; prevents B tunneling through A
when A clamps vs B_old then moves.

No-tunnel verified at 144 m/s (`ccd_sweep` gate). Obstacle-spin margin = world
max tip speed `|ω|·R`, capped 2 m/s (2026-10-08).

**Rotation-only tunneling between arbitrary volumes is unsupported** — substep
or bound tip speed externally for fast blades/propellers. Declared limitation,
not a regression.

**Verdict:** CCD TOI matches q-form swept-sphere; floor quadratic includes
rotational lowest-point velocity.

---

## 8. Sleep — three-gate wake, non-physical optimization

Any gate fires → wake:
1. **Pair novelty** (contact cache): either id pair has no entry from a prior
   tick. First touch wakes at ANY speed.
2. **Fast-other**: original velocity gate (backstop).
3. **Deep overlap**: overlap beyond `wake_depth_thresh`, even vs statics.

Persistent resting contacts no longer veto sleep (this was the F10 root cause:
never-sleeping micro-motion pumped the 10-stack to 13 m/s). Settled stacks
proceed to sleep and stay settled; slow pushers wake sleepers at any speed.

Sleep is a **non-physical optimization** — `sleep.enable = 0` runs sleepless
validation (solver alone must settle). Sleeping bodies keep real mass/inertia
(no staticize mutation); the solver treats them as infinite mass via
`effective_*` helpers.

**Verdict:** three-gate wake matches the documented F10 fix; settled stacks
stay settled (run-max 0.0 at 60 s).

---

## 9. Determinism

Fixed canonical dt = 1/60 s. Fixed solver iteration order. Exact IEEE `+ - * /
sqrt`. Fixed-coefficient polynomials (`det_math.h`). `-ffp-contract=off`.
Fallback counters count every libm transcendental fallback so desync is
diagnosable, never silent. Asserted zero in-contract.

Bitwise-identical twins on GCC/Clang IEEE targets over 600 ticks (proven by
`determinism` test). MSVC needs `/fp:strict` — unproven, declared limitation.

**Verdict:** bitwise identical at 1/4/8 threads (multicore) and across
GCC/Clang targets; MSVC unproven.

---

## 10. Warm start — cold by default (measured)

No effective warm start on any row — measured, not assumed:
- **Normal:** cold every tick. Restoring last tick's normal ejected the
  10-stack at ~13 m/s across ablations.
- **Tangent:** restored tangent projected onto Coulomb cone using the **cold**
  normal as `fn` → `fn == 0` → both tangents zeroed. The warm start was
  deleted by the clamp meant to protect it.
- **Tangent FRAME adoption** is live and helps; only the impulse seed is dead.

**Damped warm-start** (`solver.warm_start_damping`, default 0.0): nonzero
damping scales the cached normal and uses it as cone `fn`, restoring cross-tick
carry. Cold (0.0) is the shipped default — stable across all five regimes.
Nonzero damping is experimental: faster convergence at low iteration counts,
unstable under extreme gravity/friction/restitution.

**Consequence:** no cross-tick information carry, so the stack is re-propagated
from the floor every tick — why 96–128 iterations are needed for a 10-cube
stack (12–30× the published 4–8 practitioner budget). Recorded as the real
outstanding physics question in `docs/VALIDATION.md`.

**Verdict:** cold start is stable everywhere; damped warm-start is experimental
and correctly defaulted off.

---

## 11. Split impulse — whole-body translation (measured, not assumed)

Whole-body translation, not per-contact correction with lever arms. Catto GDC
2014 p.53 and Box2D 2.x apply it per contact with an `r × n` angular term, and
centre-only translation genuinely cannot right a tilt.

Per-contact split impulse (twice: independent pass, and shared Gauss–Seidel
with accumulated per-point impulse) made the 10-high pile **4.2× worse**
(0.0537 m → 0.2253 m overlap, ω 0 → 1.9217 rad/s) and remained a net
regression across the F11 seed sweep even after over-correction was fixed
(sum 2.42 → 2.95). **Reverted; negative result recorded in source.**

Deepest-only, mass-weighted, slop-gated, β-scaled, capped at
`max_separation_bias × dt`. Runtime caps (slop 0..0.05 m, beta 0..1, bias ≤10
m/s) are COUNTED (`mpe_clamp_solver_events`, bounded stderr). Deep containment
(>0.30 m) moves lighter body only.

**Verdict:** whole-body split impulse is the measured-correct choice;
per-contact angular was tried twice and measured worse both times.

---

## 12. Multicore — island-based, bitwise identical to serial

`MPE_THREADS` env var (1–8, default 1). Persistent pthread thread pool
(created once lazily, sleeps on condition variable between ticks — no
create/destroy overhead per tick). Each awake island's manifolds dispatched
to a dedicated thread; islands share no bodies (connected components), so
writes are disjoint and within-island Gauss-Seidel order is preserved.

**Bitwise identical to serial at any thread count** (verified over 600-tick
determinism twins at 1/4/8 threads). Per-body integration and sanitize
parallelize. Joint solve (`constraint_solve_all`) stays serial (O(1024) scan,
measured negligible — joints only connect bodies within one island, so no
cross-island dependency).

**Verdict:** multicore is implemented and bitwise identical to serial.

---

## 13. Summary of truth status

| domain | vs external truth | status |
|---|---|---|
| Mass properties (sphere/box/cylinder) | closed-form tensors | **exact, 0.00%** |
| Rotation algebra | Rodrigues / angle-axis | **exact within float32 eps** |
| Integration | closed-form free fall / α=τ/I | **within 0.26%** |
| Angular momentum (sphere) | `L = Iω` conserved | **0.0028%/2s** |
| Narrowphase SAT | Gottschalk 1996 | **400/400** |
| Solver effective mass | Catto sequential impulse | **matches** |
| Friction Coulomb | min(μa,μb) disc | **disclosed, 42% ≤ sqrt** |
| [FRICTION-THRESH] timestep dependence | rate-independent Coulomb | **closed** |
| Sliding acceleration | `(F−μk·N)/m` | **0.07% worst** |
| CCD TOI | q-form swept sphere | **matches, no-tunnel @144m/s** |
| Floor CCD | quadratic + rotational lowest point | **implemented** |
| β (ERP) bias | Catto GDC 2011 | **monotonic, as predicted** |
| Split impulse | whole-body translation | **measured correct** |
| Warm start | cold default | **stable all regimes** |
| Sleep | three-gate wake | **F10 fixed, settled stacks** |
| Determinism | fixed dt + order + exact IEEE | **bitwise GCC/Clang** |
| Multicore | island-disjoint, same order | **bitwise == serial** |

### Declared open (bounded, not solved)

- Rotation-only tunneling between arbitrary volumes (blade regime).
- Full Gottschalk depth numeric agreement (overlap decision 400/400; depth spot-checked only).
- Oblique multi-body angular-momentum transfer oracle (single-body torque-free gated; two-body open).
- Normal/tangent warm-start impulse carry (dead by measurement; cold default).
- ThreadSanitizer run (container TSan aborts on mapping — env, not code).
- MSVC `/fp:strict` bitwise-twin proof.
- Multithreaded solve (pairs/solve serial by design; MT sanitize only).
- `WARNINGS_AS_ERRORS` CI enforcement.

Nothing in this list is a regression or a silent defect; each is a documented
frontier with its bounding measurement recorded.
