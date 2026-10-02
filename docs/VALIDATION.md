# MPE External Validation

How the engine's physics is checked against authority that is **not this
project**. Added 2026-10-02. Companion to `ecosystem/mfs/docs/VALIDATION.md`,
which applies the same discipline to the MFS drivetrain layer.

The rest of the suite checks the engine against itself: it re-derives what
the code says it should do, or compares two paths through the same model.
That cannot catch a shared misconception — if the inertia tensor, the
integrator and the expected value are all wrong the same way, the test is
green and the physics is fiction. This document and the `mass_properties`
gate exist to break that circularity.

## Reference authorities

Constants were fetched, not remembered:

| quantity | value | authority |
|---|---|---|
| standard acceleration of gravity `g_n` | **9.80665 m/s²** | CGPM 1901 (3rd GCWM); CODATA 2022. Exact by definition. |
| standard atmosphere | **101 325 Pa** | CODATA 2022. Exact. |
| air density (ISA, sea level, 15 °C) | **1.225 kg/m³** | International Standard Atmosphere |

Method references are the papers vendored in `reference_materials/`:

- **Mirtich (1995/1996)** — impulse-based rigid-body simulation; the
  sequential-impulse formulation this solver belongs to.
- **Baraff** — rigid-body dynamics, contact and constraint formulation.
- **Anitescu** — fixed-timestep multibody dynamics and friction.
- **Catto (GDC 2005/2011/2013)** — iterative dynamics, temporal coherence,
  soft constraints.
- **Gottschalk (1996)** — OBB intersection via the separating-axis theorem.
- **Coumans (2005)** / **Schmidl (2004)** — continuous collision detection.

Note `world.gravity` defaults to `-9.81`, which is **+0.0342%** off `g_n`.
Small, but it scales every normal load, and therefore every friction
budget. Reported rather than hidden.

## What was validated

**48 automated comparisons**, plus the `reference_math` gate, references computed independently — closed-form
textbook mechanics where it exists, from-scratch RK4 where it does not.
**48 passed, 0 failed.** Engine suite is now **44 total, 42 physics green, 0 blocking failures**.

### Mass properties — exact on all three primitives

| shape | axis | law | error |
|---|---|---|---|
| sphere | x, y, z | `I = (2/5)mr²` | **0.00%** |
| sphere | — | `inverse_mass = 1/m` | **0.00%** |
| sphere | — | `inv(I)·I = identity`, zero off-diagonal | **0.00%** |
| box | x, y, z | `I = (m/12)(h²+d²)` | **0.00%** |
| box | — | half-extents preserved verbatim | **0.00%** |
| cylinder | axial (X) | `I = (1/2)mr²` | **0.00%** |
| cylinder | **transverse (Y, Z)** | `I = (m/12)(3r²+l²)` | **0.00%** |
| cylinder | — | `half_extensions = (half_length, r, r)`, axle = X | **0.00%** |

Volumes (`4/3πr³`, `8·hx·hy·hz`, `πr²l`) and the implied densities are
self-consistent to **0.00%**.

The transverse cylinder axis had **no test coverage anywhere** — MFS had
already flagged it as an open gap. It is correct, and is now gated.

### Rotation algebra — exact to float precision

| property | worst deviation | budget |
|---|---|---|
| axis-angle quaternion is unit norm | 1.79e-07 | 4 × float32 eps |
| rotation matrix orthonormal, `RᵀR = I` | 3.58e-07 | 8 × eps |
| rotation matrix `det = +1` (proper rotation, not a reflection) | 3.58e-07 | 8 × eps |
| quaternion rotation == classical Rodrigues angle-axis | 1.98e-07 | 4 × eps |
| quaternion composition `q(a₂)·q(a₁) = q(a₁+a₂)` | 1.19e-07 | 2 × eps |
| `cached_axes[i] == R · local_axis_i` | **0.00%** | exact |
| cached axes mutually orthogonal, unit length | ≤7.45e-09 | ≤2 × eps |

`float32 eps = 1.1920929e-07`. Every deviation is at or below the rounding
budget for the format.

### Integration

| property | reference | error |
|---|---|---|
| semi-implicit Euler `v_n = −g·n·dt` | closed form | 0.50% (viscous drag, not integration error) |
| semi-implicit Euler `y_n = y₀ − g·dt²·n(n+1)/2` | closed form | 0.10% |
| no horizontal drift in free fall | 0 | **0.00%** |
| angular `α = τ/I`, `ω` after 1 s (world-frame inertia) | closed form | **0.00%** |
| quaternion stays unit under integration | 1 | within budget |
| **angular momentum conserved** (sphere, isotropic inertia) | `L = Iw` | **0.00%** |
| energy conservation over a 19.5 m fall | `E₀` | 0.26% |

## Findings

### Fixed: input clamps were silent, and are not conservative

The guards that keep mass and geometry finite are **load-bearing** — a NaN
radius produces a NaN inertia tensor and detonates the solver — and they
stay. What was wrong is that they changed the physics **without saying so**,
by margins large enough to matter:

| requested | served as | factor |
|---|---|---|
| mass `1e-9 kg` | `1e-4 kg` | **×100000 heavier** |
| mass `1e-5 kg` | `1e-4 kg` | ×10 heavier |
| mass `2e6 kg` | `1e6 kg` | ×0.5 |
| mass `1e9 kg` | `1e6 kg` | **×1000 lighter** |
| radius `150 m` | `100 m` | ×0.667 |
| radius `1e5 m` | `100 m` | **×1000 smaller** |
| cylinder half-length `1e5 m` | `100 m` | ×1000 |
| radius `−1`, mass `−2` | `0.5`, `1.0` — and the body stays **dynamic** | — |
| NaN radius / mass | `0.5` / `1.0` | — |

The ×100000 mass error is the dangerous direction: a caller that believes it
built a dust mote gets something 100000× heavier, and every downstream
force, impulse and friction budget is wrong by that factor with nothing to
notice. The negative-mass case is worse still — a corrupt scene file yields
a **phantom 1 kg dynamic collider** rather than a failure.

A clamp that rewrites the physics silently is indistinguishable, downstream,
from a clamp that did not happen.

**Fix.** Every clamp now increments a counter and emits a bounded stderr
diagnostic (`[mpe] INPUT CLAMPED: <kind> requested <x>, using <y> (xN). The
body is NOT the one you asked for.`), first 8 occurrences then every 1000th
so a mass sweep cannot flood the log. Counters are exposed as
`mpe_clamp_mass_events` / `mpe_clamp_radius_events` /
`mpe_clamp_half_length_events`, with `mpe_clamp_counters_reset()`, so a test
fixture can assert its own setup clamped nothing.

### Retracted during this audit

**"3.6% angular-momentum leak."** The first version of the probe used a
**cube** and compared `|I⁻¹ω|²` before and after. For an anisotropic body the
world inertia changes as the body rotates, so `I⁻¹ω` is not a conserved
vector even with zero torque and zero damping — a torque-free asymmetric top
genuinely precesses and `|ω|` genuinely changes. The probe was measuring the
body's **shape**, not the solver.

The valid form uses a **sphere**, whose world inertia is isotropic and
orientation-independent, so `L = Iw` is parallel to `ω`. With that rig the
ratio is **exactly 1.000000000** over 600 ticks. Both shapes are now in the
probe and the cube case is labelled `NOT_A_CHECK`.

## Gaps closed (2026-10-02, `reference_math` gate)

Three of the declared gaps are now checked against the vendored references
themselves, not against behaviour alone.

### Gottschalk 1996 — separating axis theorem: **400/400**

The narrow phase uses the **exact 15-axis theorem**: 3 + 3 face normals then
all 9 cross products `A_i × B_j`, with
`overlap = proj_a + proj_b − |dot(t, n)|` and rejection on
`overlap < −slop`.

Checked against an **independent dense reference** written from the theorem
statement (the 15 canonical axes plus a 240-direction swept probe, and its own
radius projection — not the engine's `project_obb`). Over 400 randomised
yawed configurations the engine and the reference agree on
overlap/no-overlap **400 of 400**. Degenerate (parallel-edge) cross products
are skipped, which is correct: they are already covered by the face axes.

### Catto GDC 2011 — β (ERP): **monotonic, as predicted**

`revolute_joint.c` implements `bias = (β/h)·C` — precisely Catto's
"β feeds the position error back to the velocity". Configured β = 0.3, inside
Catto's [0, 1].

Gated on the *semantics*, not the value, by releasing a revolute child from a
known 0.30 m anchor error with gravity off:

| β | position error retained after 60 ticks |
|---|---|
| 0.0 | **2.4667** (not corrected — velocity constraint only, exactly Catto) |
| 0.1 | 1.5667 |
| 0.3 | 0.7833 |
| 0.8 | 0.3567 |

Monotonically faster convergence with rising β, as the formulation requires.

### Coulomb — sliding branch: **0.06%**

Once sliding, the block accelerates at exactly `(F − μ_k·N)/m`, with
`g = 9.80665` (CODATA). Measured after 0.5 s:

| F/(μ_s·N) | measured v | predicted | error |
|---|---|---|---|
| 1.10 | 1.27570 | 1.27486 | 0.07% |
| 1.30 | 1.86418 | 1.86326 | 0.05% |
| 1.60 | 2.74687 | 2.74586 | 0.04% |

### [FRICTION-THRESH] Effective static threshold is timestep-dependent (OPEN)

**Coulomb's law is rate-independent by definition. This implementation's
static threshold is not.** Bisected threshold for μ_s = 0.6, a 1 kg block,
`N = m·g_n`:

| dt | threshold (F/(μ_s·N)) | effective μ_s | deviation |
|---|---|---|---|
| 1/240 | 0.99956 | 0.5997 | **0.04%** |
| 1/120 | 0.99751 | 0.5985 | 0.25% |
| **1/60 (default)** | **0.86128** | **0.5168** | **13.87%** |
| 1/30 | 0.75761 | 0.4546 | 24.24% |

The cone converges to the correct μ_s·N as `dt → 0` but is **13.9% low at the
default timestep**, and **24% low at 30 Hz**. A body that should hold under
0.87·μ_s·N instead creeps. The friction cone is therefore *not* the cone the
material describes, and the error grows as the timestep grows — the opposite
of a well-posed discrete approximation.

**Two hypotheses tested and REFUTED, recorded so they are not re-tried:**
- *The frictionless boundary box at y = 0 is diluting the contact.* The slab
  was moved to y = +1.0, entirely clear of the boundary, and the threshold was
  **identical to 4 decimal places** (0.86128 either way). Not the cause.
- *It is a normal-force shortfall.* Plausible but **not verified** — do not
  record it as the cause without measuring λ_n directly.

**Where to look.** `collision_solver.c` selects μ_s vs μ_k by a **velocity**
threshold (`slip_speed < static_friction_threshold`), and clamps the
accumulated tangential impulse to `μ · λ_n`. So the *static* capacity is
`μ_s · λ_n`, and any shortfall in the normal impulse directly lowers the
effective threshold. That is the mechanism to measure next; it is a pointer,
not a conclusion.

Consequence for users: at the default timestep a surface advertises
μ_s ≈ 0.52 when configured as 0.6. The **sliding** branch is unaffected
(0.06% accurate), so steady-state kinematics are sound; what is off is the
*breakaway* force — the peak force before motion starts.

## Declared coverage gaps

These are **not** validated by this pass and are stated rather than assumed:

- **Contact manifold generation** for non-trivial shape pairs beyond the
  existing cylinder/sphere, cylinder/cube, cylinder/cylinder and
  sphere/box cases. (The OBB/OBB decision *is* now gated against Gottschalk;
  the contact *points* it generates are not.)
- **The static-friction breakaway threshold** — see [FRICTION-THRESH] above:
  measured, characterised, and open.
- **Angular momentum in oblique and multi-body contacts** — linear momentum
  is gated, this is not.
- **The revolute constraint's axis-alignment** budget: β (ERP) semantics are
  now gated against Catto, but the axis-alignment rows carry no bias at all
  (velocity-only) and that choice is not gated against the paper.
- **Narrow-phase feature-pair distance functions** against Gottschalk's
  *distance* algorithm (the paper's closest-point computation): the
  separating-axis OVERLAP DECISION is now cross-checked 400/400, but the
  reported penetration DEPTH and the closest-point pair are not.
- **CCD** against Coumans/Schmidl: swept tests exist; agreement with the
  reference discriminant is not asserted numerically.

## Reproducing

```
make test_suite_all          # includes the mass_properties gate
```

The one-off cross-check harness lives in `temp/audit/` and is deliberately
untracked: `probe_engine.c` (engine measurements) and
`validate_engine.py` (independent references and the comparison). What is
tracked is the gate.