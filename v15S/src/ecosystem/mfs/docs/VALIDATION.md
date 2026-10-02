# MFS External Validation

How the model is checked against authority that is **not this project**, and
what that check found. Added 2026-10-02.

The rest of the suite re-derives what the code claims to do, or compares two
paths through the same model. That class of test cannot catch a shared
misconception: if the inertia tensor, the integrator and the expected value
are all wrong the same way, the test is green and the physics is fiction.
This document and the `external_truth` gate exist to break that circularity.

## External reference constants

Fetched, not remembered:

| quantity | value | authority |
|---|---|---|
| standard acceleration of gravity `g_n` | **9.80665 m/s²** | CGPM 1901 (3rd General Conference on Weights and Measures); reaffirmed **CODATA 2022**. Exact by definition. |
| standard atmosphere `atm` | **101 325 Pa** | CODATA 2022. Exact by definition. |
| air density (ISA, sea level, 15 °C) | **1.225 kg/m³** | International Standard Atmosphere table |
| drag coefficient, sphere | **0.47** | standard aerodynamics value |

**The engine does not use `g_n`.** `world.gravity` defaults to `-9.81`, which
is `+0.0342%` away from the exact standard value. That is small, but it is a
systematic bias that enters every normal load, so every friction budget and
every rolling-resistance term scales with it. It is now *measured and
reported* by the `external_truth` gate rather than left implicit in a
default.

## What was validated, and to what accuracy

76 automated comparisons in `temp/audit/validate_equations.py`, plus the
permanent `external_truth` gate. References are computed independently —
closed-form textbook algebra where it exists, from-scratch RK4 where it does
not. **Result: 76 passed, 0 failed.**

| equation family | law enforced | worst error |
|---|---|---|
| free fall | `y = y₀ − ½g t²` vs `g_n` | 0.16% |
| free fall + drag | `v(t) = −(g/c)(1 − e^{−ct})`, closed form **and** independent RK4 | **0.00%** |
| ballistic | no horizontal force ⇒ `x = v₀t` | 0.50% |
| rotational dynamics | `α = τ/I`, `ω = αt` | **0.00%** (exact) |
| inertia, cylinder | `I = ½mr²` about symmetry axis | **0.00%** (exact) |
| inertia, sphere | `I = ⅖mr²` | **0.00%** (exact) |
| rolling | no-slip `v + ωr = 0` | 0.17% residual slip |
| Coulomb sliding | `d = v₀²/(2μg_n)` | 5.01% |
| restitution | **definition** `v_out = e·v_in`, `e` = 0.2/0.4/0.6/0.8 | 15.7% (worst), typical 5–6% |
| energy | `E = ½mv² + ½Iω² + mgh` conserved over a 4.9 m drop | 0.31% |
| linear momentum | `v₁′ = (m₁−m₂)/(m₁+m₂)v₁`, `p` conserved at `e=1` | 0.67% |
| DC machine | `V = IR + K_e ω`, `τ = K_t I`, on the V–ω line | 0.15% |
| DC machine | stall endpoint = published spec torque | **0.00%** (exact) |
| DC machine | free speed = `V/(K_e·gear)`, scaling exactly 12.8/12.0 | **0.00%** (exact) |
| battery OCV | `OCV(soc)`, floor at 10 V | **0.00%** |
| battery sag | `V = OCV − I·R_int`, `R_int = 0.06 Ω` | **0.00%** |
| PTC fuse | `I²t` characteristic, `t = 20 A·s/(I − 20 A)` | 0.80% |
| battery capacity | 3.0 Ah pack emptied by 1 h at 3 A | **0.00%** |

Two results are worth calling out because they are exact rather than
merely close:

- **The engine's air drag is exactly a linear viscous drag with
  `c = −ln(drag)` per second.** The engine applies `v ← v·drag^dt` each
  step; integrating that as the ODE `dv/dt = −g − cv` with `c = −ln(0.99) =
  0.0100503 /s` predicts a 2-second free-fall velocity of **−19.4241 m/s**.
  The engine produces **−19.4241 m/s**. Agreement to five significant
  figures, and the closed form agrees with an independent RK4 to 1e-6.
- **The DC motor is a textbook brushed machine.** Its torque–speed
  characteristic is not the endpoint-linear line, and it *should not be*:
  `R` and `K_e` are derived from the 12.0 V spec sheet while the bus sits at
  12.8 V, so the operating line is `12.8/12.0 = 1.0667×` above spec at the
  same speed. Validated against `I = (V − K_eω)/R` pointwise (≤0.15% error),
  with the voltage scaling confirmed to 0.00% on both stall and free speed.

## Findings

### Fixed: gravity was hardcoded, and the fallback invented load

`drivetrain_mecanum_analytic()` and the rolling-resistance block both read:

```c
float g_mag = 9.81f;
if (cfg->world.gravity < 0.0f) { g_mag = -cfg->world.gravity; }
```

Normal load is `N = mg`, and both the analytic lateral cap (`f_max = μN`)
and rolling resistance scale with it. Two defects:

1. **A world with gravity disabled (`world.gravity == 0`, a supported
   configuration) still got `g_mag = 9.81`.** The mecanum lateral force was
   capped at `μ·m·9.81` and rolling resistance at `C_rr·m·9.81` — a
   friction budget invented out of nothing. A robot in free fall pushed
   against an imaginary floor.
2. `9.81` is not the standard value of gravity, and it was a *different*
   magic number from the engine's own `-9.81` default, so the two could not
   be reasoned about together.

Both sites now take `|gravity|` from the world config with **no fallback
constant at all**. A zero-gravity world now yields `N = 0` and therefore no
analytic lateral force — which is correct, because a free roller with no
normal load genuinely cannot push.

### Documented: the engine's drag is ~120× too weak to be real air

`world.drag = 0.99` gives `c = 0.01005 /s` and therefore a terminal speed of
**976 m/s**. Quadratic aerodynamics for the same 42 mm / 2.6 g ball
(`C_d = 0.47`, `ρ = 1.225`) gives **8.00 m/s** — the engine's own drag is
**122×** too weak to represent air.

This is not a bug: the config schema says so outright ("VISCOUS retention
base (truth: linear viscous c=−ln(drag), NOT quadratic aero)"). But it has a
concrete consequence that was nowhere written down: **the BioBuzz ball's
flight time and range are set almost entirely by the quadratic term
`mfs_module_1` adds on top** (`mfs_module_1_ball_physics_step`), not by the
engine. Anyone tuning ball flight from the engine alone will conclude drag
is negligible and be right — and will be wrong about the ball.

### Documented: the boundary box at y = 0 is perfectly plastic

The engine applies `boundary_apply_box_cfg(..., {-250,0,-250}, {250,500,250}, ...)`
to every non-static body, **regardless of `static_plane_enabled`**. When the
material plane is disabled this leaves a perfectly-plastic (e = 0,
frictionless) backstop at y = 0.

Validated by measurement: with **no** material floor, a ball dropped from 5 m
never rebounds (`e_eff = 0`) at any radius from 0.02 m to 0.64 m. With a
material slab present, restitution is correct at every one of those radii.

**Operational consequence:** any test that forgets to add a material floor is
not measuring `e = 0` — it is measuring the boundary, and will report a
bounce failure (or, worse, a pass) that has nothing to do with the
restitution it intended to exercise. The `external_truth` gate deliberately
places its slab with its top at exactly `y = +1.0`, clear of the boundary,
and says why.

### Measured: restitution carries a consistent +6% solver bias

Effective `v_out/v_in` for set `e` = 0.20 / 0.40 / 0.60 / 0.80 is
**0.216 / 0.430–0.463 / 0.631–0.641 / 0.826–0.845** — monotonic and within
~6% of nominal. This is the standard discrete-impulse artifact (restitution
applied against a penetration-recovery-biased velocity), not an error in the
restitution law.

### Not covered — declared gaps

- **Cylinder inertia about a transverse axis** (`m(3r² + L²)/12`) is not
  exercised by any test. Only the symmetry axis (`½mr²`) and the sphere
  (`⅖mr²`) are gated.
- **Angular momentum** in oblique and multi-body contacts is not validated.
  Linear momentum is (0.67%).
- **The analytic mecanum lateral force** is validated for *behaviour*
  (axis dominance, anti-symmetry — see `mfs_t_drive_directions`) but its
  magnitude is not validated against an independent roller-contact
  reference, because the model is an admitted continuum approximation and
  has no closed form to compare against.
- **The engine's constraint solver** (revolute position/axis bias, islands,
  depenetration) is out of MFS scope and is fenced in
  `KNOWN_FAILURES.md` under engine-side observations.

## Reproducing

```
./build_tests.sh                      # includes the external_truth gate
```

The one-off cross-check harness (probes + independent Python references)
lives under `temp/audit/` and is not part of the tracked tree:

```
temp/audit/probe_truth.c        # engine measurements, KEY/MEAS records
temp/audit/probe_rest.c         # restitution vs impact speed
temp/audit/validate_equations.py # independent references + comparison
```

Every tolerance in the permanent gate is set from what a discrete
Euler/impulse integrator can physically deliver at `dt = 1/60 s`, not from
whatever the code happens to print today.