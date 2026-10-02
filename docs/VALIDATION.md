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

**48 automated comparisons**, references computed independently — closed-form
textbook mechanics where it exists, from-scratch RK4 where it does not.
**48 passed, 0 failed.**

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

## Declared coverage gaps

These are **not** validated by this pass and are stated rather than assumed:

- **Coulomb friction cone behaviour** over the full stick–slip transition;
  only the sliding-distance law is gated.
- **Contact manifold generation** for non-trivial shape pairs beyond the
  existing cylinder/sphere, cylinder/cube, cylinder/cylinder and
  sphere/box cases.
- **Angular momentum in oblique and multi-body contacts** — linear momentum
  is gated, this is not.
- **The revolute constraint's** position-correction and axis-alignment
  budgets against the Catto soft-constraint reference are exercised by
  behaviour tests, not compared against the paper's error bounds.
- **Narrow-phase feature-pair distance functions** against Gottschalk's
  separating-axis theory: the theory is vendored but no numerical
  cross-check against it is in place.
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