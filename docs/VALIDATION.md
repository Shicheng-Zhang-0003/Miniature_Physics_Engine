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
| standard acceleration of gravity `g_n` | **9.80665 m/s²** | CGPM 1901, **Declaration 2** (3rd CGPM, CR 70); CODATA 2022 recommended value. **Exact by convention** — see note. |
| standard atmosphere | **101 325 Pa** | CODATA 2022, Table XXXIII. Exact (SI-defining). |
| air density (ISA, sea level, 15 °C) | **1.225 kg/m³** | **ISO 2533:1975 / U.S. Standard Atmosphere 1976** — *not* CODATA, which does not report it. |
| speed of sound (ISA, sea level, 15 °C) | 340.294 m/s | ISO 2533:1975 (tabulated; ≈340.294951 from R = 287.05287, γ = 1.4) |

**Precision note on `g_n`, verified against BIPM/CGPM and NIST records
(2026-10-03).** The value is right and unchanged: CODATA 2018, 2022 and the
2026 listing all give `9.806 65 (exact)`, and no revision is proposed at the
28th CGPM (Versailles, 13–15 Oct 2026). Two citation details were imprecise and
are corrected here:

- CGPM 1901 issued **Declaration 2** on the unit of mass and the conventional
  value of `g_n` (CR 70), not a "Resolution". It *adopted a value*; it did not
  define an SI unit in terms of `g_n`.
- Therefore "exact **by definition**" is loose. `g_n` is not an SI defining
  constant and is not derived from `h`; it is exact **by convention**, which is
  the stronger and more accurate statement (a conventional value carries zero
  uncertainty by construction). BIPM's own note records that the value was the
  reference for the now-obsolete unit *kilogram-force*.

There is **no "CGPM 2022 Resolution 1 redefinition of `g_n`"** — that resolution
is on metrology governance and none of the seven 2022 resolutions touch `g_n`.
This audit checked for it specifically so the claim could not reappear.

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

**Fix.** Every clamp increments a counter and emits a bounded stderr
diagnostic (`[mpe] INPUT CLAMPED: <kind> requested <x>, using <y> (xN). The
body is NOT the one you asked for.`), first 8 occurrences then every 1000th
so a mass sweep cannot flood the log. Counters are exposed as
`mpe_clamp_mass_events` / `mpe_clamp_radius_events` /
`mpe_clamp_half_length_events`, with `mpe_clamp_counters_reset()`, so a test
fixture can assert its own setup clamped nothing.

**DESPOT-2026-10-03 CORRECTION — "every clamp" WAS NOT TRUE, AND THE MISSING
PATH WAS THE IMPORTANT ONE.** The 2026-10-02 work instrumented thirteen sites
in the sphere and cylinder *initialisers*. It did not touch
`rigidbody_sanitize()`, which runs on **every body every tick** and is
reachable from the terminal editor and from any deserialised scene — and which
rewrote mass and geometry with **no counter and no diagnostic at all**. So the
claim above was false precisely where observability matters most, which is the
same "a clamp that changes the physics silently is indistinguishable from a
clamp that did not happen" failure the counters were introduced to prevent.

`sanitize()` is also **not policy-identical** to the initialisers, and the
documented single clamp table concealed it:

| input | initialiser | `rigidbody_sanitize()` |
|---|---|---|
| radius ≤ 0 | 0.5 | **0.01** |
| negative mass | 1.0 (body stays dynamic) | **0.0 if static, else 1.0** |
| mass > 1e6 | 1e6 | 1e6 |
| cylinder half_length ≤ 0 | 0.5 | **0.01** |

The negative-mass row is the consequential one: through `sanitize()` a corrupt
body can quietly become **static** (infinite mass, locked rotation) rather than
the dynamic 1 kg collider the initialiser would produce.

`sanitize()` now reports too — ten sites, labelled `sanitize <kind>` so the
diagnostic distinguishes the two paths — bringing the instrumented total from
13 to **23**. Two divergent clamp policies now both declare themselves instead
of one loud and one silent. The table above is the honest statement; the old
single-table presentation was not.

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

### Coulomb — sliding branch: **worst 0.07%** (mean 0.05%)

Once sliding, the block accelerates at exactly `(F − μ_k·N)/m`, with
`g = 9.80665` (CODATA). Measured after 0.5 s:

| F/(μ_s·N) | measured v | predicted | error |
|---|---|---|---|
| 1.10 | 1.27570 | 1.27486 | 0.07% |
| 1.30 | 1.86418 | 1.86326 | 0.05% |
| 1.60 | 2.74687 | 2.74586 | 0.04% |

### [FRICTION-THRESH] Effective static threshold was timestep-dependent (CLOSED 2026-10-04)

**Coulomb's law is rate-independent by definition. Before 2026-10-04, this
implementation's static threshold was not.** Bisected threshold for
μ_s = 0.6, a 1 kg block, `N = m·g_n` (old behaviour):

| dt | threshold (F/(μ_s·N)) | effective μ_s | deviation |
|---|---|---|---|
| 1/240 | 0.99956 | 0.5997 | **0.04%** |
| 1/120 | 0.99751 | 0.5985 | 0.25% |
| **1/60 (default)** | **0.86128** | **0.5168** | **13.87%** |
| 1/30 | 0.75761 | 0.4546 | 24.24% |

**Root cause, measured 2026-10-04 (the old pointer was wrong).** The
"measure λ_n next" pointer above is REFUTED: per-contact λ_n converges to
exactly `N·dt` (0.16342 vs 0.16344) at every iteration count. The normal
force was never short. The defect was the **memoryless per-iteration
stick/slip selection**: the μ_s-vs-μ_k choice re-evaluated on the LIVE
per-iteration slip, which is solver transient, not physics state. Under step
force loading the first iteration always sees the tick's own injected
`F·dt/m` (0.117 m/s at 7 N) above the 0.02 static gate, so kinetic is
selected, the accumulation saturates at the kinetic clamp, and
late-iteration static re-selection cannot retroactively add the missing
`(μ_s−μ_k)·F_n` — its correction signal `−vt·meff` is already ~0 by then.
Smoking guns (temp probes, uncommitted): μ_s = 0.9/1.5 broke at ~5.9 N
instead of 8.83/14.7 N; tick-end velocity ratcheted +0.035/tick = exactly
`(F − μ_k·N)·dt`; slipth = 0.001 read 4.91 N = μ_k·N to 3 decimals.

**Fix.** `contact_point_data.snap_friction_mu`, recorded once per contact in
`collision_prepare_solver` — which runs pre-force-integration, so it sees
the tick's opening (was-it-sticking) velocities — and reused by every
iteration of the tick (`collision_snapshot_friction_mu`;
`collision_solver.c`, `collision_mechanics.h`). A tick that starts at rest
solves the whole tick static and truly holds to μ_s·N; a tick that starts
sliding solves kinetic. No new fields persist anywhere (runtime scratch
only); direct resolve callers without a snapshot keep legacy behaviour.

**Battery after the fix** (force-accumulator bisection, 1 kg block,
`g = 9.80665`, 5 mm slip gate over 60 push ticks):

| condition | threshold / μ_s·N | effective μ_s |
|---|---|---|
| dt = 1/60, iters 8 | 0.99485 | 0.5969 |
| dt = 1/60, iters 32/64/128 | 1.00038–1.00045 | 0.6002–0.6003 |
| dt = 1/120, iters 64 | 1.00547 | 0.6033 |
| dt = 1/30, iters 64 | 0.99920 | 0.5995 |
| μ_s = 0.9 (cone 8.83 N) | 0.99921 | 0.8993 |
| small block / long push / tight gate / wide stick window | 0.997–1.000 | — |

Iteration-independent, timestep-independent, μ-scaling. The residual knob
honesty: `static_friction_thresh = 0.001` reads 0.972 — a 1 mm/s stick
window genuinely narrows the cone, which is what the knob means. Above-cone
loading still slips and accelerates (μ = 0.9, F = 9.5 N verified); the fix
grants grip, never immunity.

**Blast radius, adjudicated by the suite (not by assertion).** True static
grip changes emergent behaviour wherever the old kinetic-leaning selection
acted as pseudo-damping: the `meta_convergence` 32-iteration arm of the
μ = 0.9 8-stack now buckles (2.72 m vs 0.49; residual 5.5 m/s mid-collapse)
while 16 stands and 64 converges to 0.06 — a different valid trajectory of a
chaotic pile, recorded in-gate (arm-over-arm ratio withdrawn for the measured
reason, calm-top-arm gates kept). The MFS tank pivot translates 0.0530 →
0.0642 m (+21%, re-baselined with cause chain; heading +2.3%, in band;
mecanum strafe byte-identical). Full profile stays 232 pass + 2 xfail (total 234), all five
solver regimes 42/42.

## Declared coverage gaps

These are **not** validated by this pass and are stated rather than assumed:

- **Contact manifold generation** for non-trivial shape pairs beyond the
  existing cylinder/sphere, cylinder/cube, cylinder/cylinder and
  sphere/box cases. (The OBB/OBB decision *is* now gated against Gottschalk;
  the contact *points* it generates are not.)
- **The static-friction breakaway threshold** — see [FRICTION-THRESH] above:
  closed 2026-10-04 (0.999–1.005 of μ_s·N across dt, iterations, μ).
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
---

# Audit of 2026-10-03 — test-integrity and mechanism findings

A second pass, run after the reference material above was already in place.
Its theme is narrower and more uncomfortable: **not "is the physics right" but
"can the tests tell you when it stops being right."** Every claim below was
verified by running the shipped code, not by reading it.

## [META-ROTATION-UNGATED] A blocking gate that could not fail — FIXED

`meta_rotation` is registered as a **blocking physics** case and is the only
gate for rotation equivariance. Its failure branch printed `[XFAIL]` and
**never incremented `tp->failures`**. `tools/test_runner.py` records `[XFAIL]`
as `severity="info"`, i.e. non-blocking — so the case reported PASS on any
equivariance violation whatsoever.

Proven, not argued. A world-axis-dependent acceleration was injected into
`rb_integrate_velocity` (`acc.x += 0.05f * pos.x`, `acc.z += 0.035f * pos.z`) —
precisely the defect class the case exists to catch:

```
[info] rotation equivariance: max |dpos| = 1.759e-04 m
[XFAIL][META-ROTATION] rotation equivariance broken: max|dpos|=1.759e-04 m
  [PASS] meta_rotation (checks failed: 0)      <- PASS, exit status 0
```

The tripwire fired *correctly* and the test still reported green. A controlled
comparison in one directory, same flags:

| build | result | caught by |
|---|---|---|
| clean | 41/42 | — |
| bug injected | 38/42 | `rolling_decay`, `spring`, `f10_long_run` |

A rotation-equivariance break was caught by three unrelated tests and **not**
by the one named after it.

Fixed by making the failure branch increment `failures`. The clean engine
measures `max|dpos| = 5.440e-07`, so the existing `1e-4` gate has ~184×
headroom and did not need to move. Re-verified: clean build PASSES, injected
build **FAILS with exit status 1**.

## [SOLVER-NON-MONOTONIC] `meta_convergence` was measuring nothing — FIXED

Worse than non-gating: **vacuous**. The fixture was a head-on sphere-sphere
pair, whose single contact point has an exact effective mass and no coupling.
Measured on the real engine, that scene is **bitwise identical at 1, 2, 4, 8,
16, 32, 64 and 128 solver iterations**. Every arm reported error
`0.0000e+00` against the 256-iteration reference, so the gate
`err <= prev*1.35 + 1e-5` passed trivially — and would also have passed with
`solver_iterations` wired to nothing at all.

The suite's only "solver convergence" gate could not observe the solver. The
fixture comment claimed an earlier version "reported exactly 0.0000 error
because nothing collided" and that the fix made it "exercise real contact".
The collision *was* real; it was **trivial**.

Replaced with an 8-high cube stack, where load transfers through layers of
4-point manifolds. The knob is now plainly live — `max|dpos|` vs a
128-iteration reference:

| iterations | default | heavy |
|---|---|---|
| 1 | 3.73 | 3.04 |
| 4 | 3.88 | 4.08 |
| 16 | 0.400 | 3.59 |
| 32 | **0.490 ↑** | 4.88 |
| 64 | 0.0629 | 3.17 |

Two gates now, and the first is the one that matters:

- **anti-vacuity** — the 1-iteration arm must differ from the reference by
  more than 1 mm (measured 3.6 m). This is the gate that would have caught the
  old fixture, and it is the general defence against a test that measures a
  converged system.
- **no divergence** — bounded growth arm-over-arm.

A third gate, "strictly decreasing", was written, measured, and **deliberately
not kept**: the table shows error is *not* monotonic in iteration count for a
friction stack, even at default config. That is a real property of the
formulation (support-first ordering plus two visits per iteration interact with
which corner wins each sweep), not a defect. Asserting strict monotonicity
would have been asserting a wish — the exact failure this project already made
once with the withdrawn `mpe_t_meta_sleep`. The readme's claim "solver error
monotonic in iteration count" is therefore corrected: it is **not monotonic**,
and the gate says what is actually true.

## [FALLEN-UNFIRABLE] The "nothing fell through the world" gate could not fire — FIXED

`f10_long_run` and `f11_torture` both gate a `fallen` counter. With the
world-edge safety net installed, `boundary_apply_box_cfg` unconditionally
enforces all four sub-conditions (`obb_min_y ≥ −es`, `|x| ≤ 250+es`, …), so
the counter was structurally incapable of firing. The earlier note claimed the
volume invariant was "the stronger property, since it catches a boundary that
fails to apply at all" — but the boundary cannot fail to apply while it is
unconditionally installed, so that reasoning was wrong and the gate measured
the clamp, not the solver.

Fixed by adding `boundary.safety_net_enabled` (registry 78 → 79, default **1**,
so shipped behaviour is byte-identical) so the net can be switched off, and
`f10_long_run` now runs a **phase 2** on the same scene and same seed with the
net **off** and the real Coulomb slab in place. Now the contact solver alone
has to hold every body up:

```
[info] safety net OFF: fell=0 nan=0 lowest_centre_y=0.3500 (slab top y=0)
```

That is the property the gate always claimed, and it is now falsifiable — if
the solver tunnels or the slab leaks, it fires.

`f11_torture`'s counter was made **report-only, deliberately**: its scene is a
pile with no floor, so with the net on the counter is unfireable and with the
net off a rising count is the *correct* outcome. Neither variant is a test, so
its verdict stays the honest crash oracle it has always been described as.

## [CLAMP-TAUTOLOGY] Several rest-height assertions are satisfied by the clamp

With floor contact response fully disabled (`static_plane_enabled = false`,
the engine default at `physics_world.c:162`, and no floor body), the only thing
that can stop a falling body is the emergency clamp:

```
sphere r=0.05             support=0.0500  final y=0.050000  -> RESTS EXACTLY AT SUPPORT
cylinder r=0.05 (axle X)  support=0.0500  final y=0.050000  -> RESTS EXACTLY AT SUPPORT
```

`0.050000` is exactly the value `cylinder_drop` asserts
(`MPE_CHECK_NEAR(cyl_y, 0.05f, 0.02f)`) and `floor_collision_diag` asserts.
`ccd_sweep`'s floor case has no floor at all, so its "swept TOI: no
tunneling" verdict was likewise produced by the clamp. A build with the
contact solver stubbed to a no-op would pass these.

**Not changed here, and why.** Tightening them is a real task, not a doc edit:
the correct fix is for each to assert a property the clamp cannot satisfy
(approach velocity, contact-point count, absence of a boundary correction),
which requires per-case re-derivation of the expectation. The tests that
genuinely load the solver — `friction_stop`, `incline_accel`, `stack`,
`reference_math` §3, `static_hold`, `rolling_decay` — need a real tangential
force and are unaffected. Recorded rather than quietly re-baselined.

**CLOSED 2026-10-04.** The per-case re-derivation is done: `cylinder_drop`
and `floor_collision_diag` now bind a per-world net-OFF config
(`mpe_world_no_net`, `mpe_test.h`) so only contact manifolds can hold the
bodies, and gate on clamp-unsatisfiable properties — genuine free-fall
approach speed (cylinder_drop max_fall = 1.960 m/s vs ~1.98 predicted;
diag tracks it too) plus an ever-contacted record across the run (a
clamp-held body has neither). `ccd_sweep`'s floor case, which had no floor
body at all, gets a real Coulomb slab plus net-OFF plus the contact gate
(wall case already had a real wall). A no-op solver now fails all three;
net-OFF solver-held rest is measured (0.050–0.054 m) in all five regimes.

## [MFS-OBSERVER] Two misattributions of one number — FIXED

The MFS closed-loop stall endpoint read 3.1279 N·m against a 3.7265 N·m spec
and was attributed to "the observer → implicit-solve coupling", with a **25%
tolerance chosen to accommodate it**. Measured, observer armed identically in
both runs, copper temperature the only variable:

| | output torque | vs spec |
|---|---|---|
| thermal active | 3.12793 N·m | −16.062% |
| **temperature pinned 25 °C** | **3.72650 N·m** | **+0.000%** |

The observer contributes **exactly nothing**. The mechanism is the copper
model `r_eff = R·(1 + 0.00393·(T − 25))`, which reaches `r_eff/R = 1.19237` at
73.95 °C after 300 stall ticks, and `1/1.19237 = 0.8387` — the entire −16.1%.
So a 25% "observer robustness" band was encoding a thermal artefact, and a
genuine 25%-off observer regression would have been indistinguishable from a
warm motor.

Fixed by removing the confound instead of absorbing it: `stall_endpoint` now
gates the derated value against the `r_eff(T)` model **and** the 25 °C value
against spec at **2%** (12.5x tighter).

Related: **both free-speed gates never called `motor_observe()`**, so `τ_L ≡ 0`
and the disturbance observer was absent from the measurement — while the
shipped drivetrain arms it every tick (`robot.c:764`). Same rig:

| | free speed | vs no-load line |
|---|---|---|
| observer not armed (gates as written) | 237.8667 rpm | +0.0000% |
| observer armed, as `robot.c` | 92.0172 rpm | **−61.3156%** |

So the readme's "free speed exact at any bus voltage" was measured on a
configuration the engine never runs in. Resolved by **splitting the claim
rather than picking a winner**: free speed is independent of `R` and of load
*by construction* (`w_free = V/(kv·gear)`, `kv = V_nom/(w_free·gear)`, so `R`
cancels), which makes the open-loop check a sharp test of the electrical
model — so it is kept and now **labelled open-loop**. The observer-armed value
is a limit cycle, not a free speed; it is printed with its honest −61.3% and
gated on being finite and bounded. Pinning a number to a limit cycle would be
inventing a specification — the "fabricated tank target" failure this project
retracted on 2026-09-29. The defect stays tracked as `[MOTOR-III]`.

## [MFS-FIXTURE] Two MFS gates were red at pristine HEAD, both fixture bugs

- **`intake_stop`** could not pass. `intake_power` is *recomputed every tick*
  by `pre_step` from the gamepad edge state, and `mfs_module_1_intake_step()`
  is called *inside* `pre_step` — so writing the field from the test lands
  either before the overwrite or after the motor has already been commanded.
  Measured: the roller returned `+58.068 rad/s`, **bit-identical to the broken
  run**, which is how the ordering was confirmed rather than assumed. Also,
  `intake_active` was left `false` by the previous phase, and reverse is
  deliberately gated on an engaged intake. Fixed by driving the *consumer*
  directly — which is exactly the defect the H5 fix addressed ("written twice
  and read by nothing at all") — and re-enabling the intake. Now `-62.570
  rad/s`, the documented expected value.
- **`ftc_hotload`** failed **silently**: exit status 1 with *zero bytes* on
  stdout and stderr, because the `dlopen` failure path did
  `failures++` and returned with no diagnostic. Worse, the plugin path was
  CWD-relative and POSIX `mpe_pick_plugin` returns it verbatim with no
  existence check, so the same source, same build and same `.so` passed or
  failed purely on the working directory — and the runner invokes it from the
  repository root, so it never passed there. This is the same class as the
  engine runner defect already recorded ("a `206/206 … xfailed: 0` line could
  hide a known-red frontier with no trace"). Fixed: an explicit candidate list
  with `MPE_FTC_PLUGIN` override, the resolved path printed, every path tried
  named on failure, and `build_tests.sh` exporting the absolute path so the
  harness no longer depends on CWD.

**MFS is now 14/14**, verified from the repository root rather than only from
`ecosystem/mfs`.

## Verified NON-findings

Recorded because a wrong fix is worse than no fix, and because these were
reported as defects and did not survive checking.

- **`math3_inverse` Frobenius overflow — NOT a bug.** The claim was that
  `frob_sq * sqrt(frob_sq)` reaches `inf` and makes every finite determinant
  read as singular. `math3` elements are `float` and non-finite inputs are
  rejected on entry, so `‖M‖_F² ≤ 9·(3.4e38)² ≈ 1.04e78` and
  `‖M‖_F³ ≤ 1.06e117` — **provably inside double's 1.8e308**. The overflow
  needs double-precision matrix elements the type cannot hold. The existing
  double promotion is correct and sufficient; no change made.
- **OBB face-clip winding — NOT a bug.** Reported as a transposed-index
  self-intersecting bowtie in the Sutherland–Hodgman input polygon. The
  construction is `c − (u·eu + v·ev)`, not `c − u·eu + v·ev`, so the order is
  `(+eu,+ev) (+eu,−ev) (−eu,−ev) (−eu,+ev)` — a correct cyclic quad. Verified
  three ways: shoelace on the constructed vertices gives area 16.0 for a 4×4
  square; and on cases that genuinely force clipping (big box on a small
  pedestal, box overhanging a ledge, 4000 rotated asymmetric poses) every
  reported contact lands exactly on a true corner of the computed overlap
  patch, 0 non-finite.
- **MFS gamepad probing `/dev/input/js0` — NOT a defect.** `build_tests.sh`
  exports `MPE_GAMEPAD_DEVICE=disabled` unconditionally and `gamepad_init`
  consults it for a NULL path. The only `gamepad` line in a harness run is the
  compile check. The device-open message came from a manual invocation that
  bypassed the environment.
- **`g_n` and the ISA constants — NOT wrong.** `9.80665` is current and
  unchanged; `1.225 kg/m³` is correctly attributed to ISO 2533:1975, not
  CODATA (which does not report it). Only the "exact by definition" phrasing
  needed tightening, above.

## Standing lesson

Three of the four defects in this section were in **tests or harnesses**, not
in the physics. Two more were numbers attributed to the wrong mechanism. In
every case the *detection* machinery was the thing that was broken, and in
every case the honest fix was to **measure** rather than to adjust a tolerance
until the line went green.

---

# Second pass, 2026-10-03 — the cube-phasing bug, and a self-inflicted regression

The user reported, in the F11 torture view: *"the f5 cube stack cubes start
phasing into and folding into each other like they are hollow."* That is a
correct observation and it was caused **partly by this audit's own previous
commit**, so the record has to start there.

## [SPLIT-IMPULSE-REVERTED] My 205549 change caused it. Measured, then reverted.

The earlier commit replaced the whole-body positional translation with
per-contact split impulse and lever arms, because Catto GDC 2014 p.53 and
Box2D/Bullet genuinely do it that way and a centre-only translation cannot
right a tilt. The *premise* was right. The *implementation* was not: I applied
each contact's correction **independently**, so a 4-point manifold gave the
body 4× the intended translation plus four independent angular kicks at
r = 0.5 m. On the F10/F11 10-high pile:

| | before | after |
|---|---|---|
| worst pairwise overlap | 0.0537 m | **0.2253 m** (4.2× worse) |
| max &#124;ω&#124; of the stack | 0.0000 rad/s | **1.9217 rad/s** |

0.2253 m between 1.0 m cubes **is** the reported symptom.

I then implemented the *correct* form — shared Gauss–Seidel with a per-point
accumulated impulse clamped at ≥ 0, which is what actually makes the contacts
share the correction in Box2D. It fixed the over-correction (0.2253 → 0.0818 m,
ω back to 0.0001) and improved the isolated tilted box (0.354° → 0.312°). It was
**still a net regression** across the F11 seed sweep:

| seed | 1 | 2 | 3 | 6 | 7 | 9 | **sum** |
|---|---|---|---|---|---|---|---|
| original | 0.440 | 0.808 | 0.220 | 0.240 | 0.349 | 0.363 | **2.420** |
| shared + angular | 0.650 | 0.832 | 0.060 | 0.504 | 0.407 | 0.500 | **2.953** |
| shared, linear only | 0.766 | 0.642 | 0.832 | 0.728 | 0.648 | 0.417 | **4.033** |

Better on one seed, worse on five. In a buckling pile the angular positional
kick feeds energy into the next tick's velocity solve, and the column already
sits at its stability boundary. **Reverted**, and verified bitwise identical to
the pre-audit solver on all ten F11 seeds. The 0.042° gain on an isolated case
does not pay for that, and the negative result is recorded in the source so it
is not attempted a third time.

Kept from that commit: the wake threshold is no longer a hard-coded `0.01 m`
that contradicted the config schema's own measured note.

## [F11-BUCKLE-INTERPENETRATION] Three real defects, none in the contact solver

**1. The column spawned 10 mm interpenetrated.** `mpe_pile_bodies` placed ten
1.0 m cubes at 0.99 m pitch, so every cube began 1 % inside the one below it.
The shipped GUI scene has always been correct — `scene_spawn_long_run_validation`
(`scene_init.c:585`) uses pitch **1.002** with the comment *"2mm air gap, no
built-in overlap"*. The headless fixture was the only place still on 0.99, so
**the test and the product were running different scenes**. Now 1.002.

**2. The torture scene had no floor.** With nothing underneath, a 9.5 m column
free-falls until the bottom cube meets the world-edge safety net — perfectly
plastic *and* frictionless — and the remaining nine arrive at ≈5 m/s onto a
surface that cannot hold them. `scene_spawn_config_torture_test()` calls
`scene_spawn_long_run_validation()`, which calls `scene_ensure_friction_floor()`;
the reason is already written down at `scene_init.c:571-573`. That fix reached F10
and the GUI and never reached this case.

Measured effect, worst pairwise cube-cube overlap, summed over a 10-seed sweep:
**5.44 m → 1.88 m**. For the default seed alone: **0.841 m → 0.0016 m**.

**3. Nothing ever measured interpenetration.** `f10_long_run` and `f11_torture`
counted NaN and fallen bodies. A scene can be finite, uncorrupted, awake and
inside the world box while two cubes sit 60 % inside each other, with the suite
green — which is exactly what happened. Added `mpe_worst_cube_overlap()`, sampled
every 5th tick of the **whole run** rather than at the end, because the phasing
is a transient during the buckle that then partly recovers; sampling only the
final state would very likely have missed it.

## [A CORRECTION TO MY OWN DOCUMENTATION]

My first version of that gate claimed *"the corrected scene measures
0.0016–0.24 m across seeds"*. **That was wrong.** Measured properly, all pairs,
1500 ticks:

| seed | 0 | 1 | 2 | 3 | 6 | 7 |
|---|---|---|---|---|---|---|
| corrected | 0.0016 | 0.2405 | **0.6925** | **0.5638** | 0.0164 | 0.1337 |
| pre-fix | 0.8409 | 0.9815 | 0.9336 | **0.1028** | 0.0771 | 0.6869 |

The true range is 0.0016–**0.69** m, and note what the second row shows: seed 3
is *better* before the fix (0.103) than after (0.564). **A single global
threshold cannot separate the corrected fixture from the broken one** — the
spread *within* each group is as large as the gap *between* them, because the
torture outcome is chaotic. Any bound I had picked would have been fitted to a
story rather than to a distribution.

So the gate is deliberately scoped to what it can certify: the seed the case
actually runs (fixed seed `0xC0FFEE`). There it measures **0.0100 m** — which is
the penetration slop, i.e. resting contacts and nothing else — against **0.8409 m**
pre-fix. Bound set to **0.05 m**: 5× headroom over the shipped value, 16× under
the defect it exists to catch. Falsifiability demonstrated, not assumed:
tightening below the measured value turns the case red with exit status 1.

## Open: the buckling transient itself — DECLARED, NOT SOLVED

Under extreme randomized gravity the 10:1 column buckles (the project's own
measurement puts its stability boundary near −17.12) and cubes can transiently
overlap by up to ~0.69 m. What *is* established:

- The engine is **not** broadly broken: with default config, a 10-cube stack
  dropped 6 m onto the friction floor peaks at **0.0128 m** interpenetration —
  the slop. `max_separation_bias` changes that number by **0.000000 m**.
- **Solver iterations are the dominant lever for a stable column**, and sharply
  so. Gravity −17, otherwise default:

  | iterations | 64 | 96 | 128 |
  |---|---|---|---|
  | worst overlap | 0.3105 | **0.0024** | **0.0000** |

  127× then exact. `bias_factor` (0.1 → 0.5 → 1.0) changes nothing.
- Neither the correction budget (`max_separation_bias`, `depenetration.
  max_correction`, `correction_factor`) nor forcing 128 iterations fixes the
  F11 seeds: 128 helps seeds 1/2/6 and *hurts* 3 and 9.

So the residual is a genuine open item, not something quietly re-baselined. The
honest next measurement is the one the pointer already gives: instrument λ_n
directly during a buckle to establish whether the deep overlap is a normal-force
shortfall or a contact-pair detection failure, since no budget knob touches it.

---

# Third pass, 2026-10-04 — despot audit: engine.cfg recurrence, harness revival, exact Coulomb

## [TORTURE-LEAK-RECURRENCE] engine.cfg held full F11 values again — different mechanism, closed

The 2026-10-01 audit found a poisoned live config from the pre-fix era and
deleted the files. On 2026-10-04 `status/engine.cfg` again held all 79 F11
torture values (gravity −17, drag 0.635, rolling 4.95, sleep OFF, 9-ton
spawner masses, safety net OFF) — this time through a live mechanism, not
leftovers: the F11 restore runs only at normal run completion, while every
exit/save path (`root_gtk` ×2, menu-4, config-menu, `config save`, `sync`,
`poweroff`) wrote `g_cfg` unconditionally. Any mid-torture quit published
torture as the next boot's defaults — the user's Tom-and-Jerry physics
(frictionless-feeling floor, no sleep, artillery spawns) with a healthy
engine underneath. The engine math was exonerated first (every formula
re-verified against textbook statements, §1 of this pass's record — no live
sign error, missing term, wrong constant, unit error, or timestep dependence
in any formula), and only then was the config convicted by diffing all 79
keys against the backup.

Closed with defense in depth: `mpe_config_save_guarded()` refuses any write
to `engine.cfg` while torture is live (counted,
`mpe_config_torture_save_blocked_total`); `long_run_validation_snapshot_clean()`
takes an in-memory pre-torture snapshot (a file backup can go missing across
CWDs — memory cannot); `long_run_validation_cancel_restore()` runs on every
exit path before any save (memory → backup file → compiled defaults; torture
never survives); F11 sets flags before starting and snapshots before
randomizing. Plus diagnostics contributed along the way: config-loader
trailing-garbage rejection, `ferror` check on save, scene legacy-reader
finite/type validation, joint-truncation veto (was silent partial commit),
v200 dropped-entry counting, save-NaN refusal, `save_scene == 2` propagation,
render uniform-`-1` warnings + utility-shader fallback + non-finite body
skip + per-frame `glGetError` drain, TUI output-health (`ferror` → rc 2) and
full-state finite scan, runner regime completion (no early return), MFS
duplicate-name contract, `--test` diagnostic labelling, TUI pair-scan
coverage gating (non-stress scenes must scan fully), and a config-mutation
contract that snapshots `status/engine.cfg*` around every profile and fails
on unexpected mutation.

## [REGIME-REVIVAL] The regime matrix was vacuous for 33 tests — now live and green

`mpe_test_begin` applies the regime (`MPE_TEST_REGIME`), but 33 of 44 tests
called `mpe_config_init()` again on the next line, wiping it — every regime
ran default physics with different labels. The matrix could not observe the
knob it existed to turn (same class as the withdrawn vacuous convergence
fixture). The 33 re-inits are deleted; 13 tests with absolute oracles now pin
their reference conditions (gravity/iterations/materials/sleep, mirroring the
existing projectile pins) so each oracle measures its law, not the regime.
Found and dispositioned in the process: `MPE_SKIPPED` was 2, so any test with
exactly 2 failed checks reported SKIP instead of FAIL (light driven_wheel,
heavy incline_accel/list4, brittle f10 all vanished into SKIP; the C summary
undercounted blocking failures). The sentinel is now −1, unreachable by
counting. All five regimes now read 42/42 green with teeth: the revival
itself caught 20+ regime/original failures on the way there, each fixed by
pinning premise (not by loosening oracles) except meta_convergence (b), which
was withdrawn for the measured chaotic reason with calm-top-arm gates kept.

## Verification record, 2026-10-04

- Canonical suite 42/42 blocking (+2 diag) in **all five** regimes
  (default/light/heavy/brittle/sticky), ASan/UBSan included.
- Full profile: **234 checks, 232 passed, 0 failed, 2 xfailed, 6
  informational, 0 blocking failures** (`temp/qa_runs/20261004T162513Z-456158/`).
- External batteries unchanged and green (48 engine + 76 MFS + 22 ext-truth).
- `status/engine.cfg` restored from backup (compiled defaults + friction
  floor) and guarded; runner contract proves profiles no longer mutate it.
