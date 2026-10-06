# MFS Mathematical Models

Every equation the robots run on, with the file that implements it, the
constants that tune it, and what is deliberately NOT modeled. No physics
here is invented: motor endpoints come from `docs/FTC_SPECS.md`, and every
tuned constant below is labeled TUNED.

Notation: commands are -1..1, `dt` is the tick (1/60 s), torques in N·m
unless noted.

## 1. DC motor (`modules/ftc/submodules/motor.c`)

Quasi-static DC: no inductance `L·di/dt`, no PWM switching, no brush drop.

**Spec derivation** (`motor_from_spec`, motor.c:7-44; presets at 12.0 V):

```
Kt = stall_out / (gear · eff) / Istall     (motor-shaft, N·m/A)
R  = Vnom / Istall                          (ohms)
Kv = Vnom / (free_rad_s · gear)             (V per motor-shaft rad/s)
```

Preset `stall` is OUTPUT-shaft (post-gearbox, includes loss), so dividing by
`gear·eff` recovers motor-shaft Kt and simulated stall output equals the
published stall exactly; efficiency cancels and is kept as a build marker
(0.85 planetary / 0.80 spur). `Kt != Kv` deliberately: the ratio absorbs
gearbox friction + no-load current (two-endpoint fit, not textbook Kt==Ke).

**Per-tick update** (`motor_update`, motor.c:46-102):

```
Vapp    = Vbatt · cmd
backEMF = Kv · (w_wheel · gear)
r_eff   = R · (1 + 0.00393 · (T - 25))      (copper derating)
I       = clamp((Vapp - backEMF) / r_eff, ±Istall)
τ_motor = Kt · I
τ_out   = τ_motor · gear · eff
```

Exact stall (w=0 → I=Istall → τ=spec) and exact free (I=0 at
`w_free = V/(Kv·gear)`) by construction.

**Implicit-in-speed solve** (`motor_update_load`, motor.c:104-214) —
required, not optional: axle inertia `Iaxle = ½·m·r² ≈ 2.5e-4` with stall
~2.4 N·m moves ~160 rad/s/tick explicit (DESPOT-2026-09-28: was "~30",
recomputed 2.38·(1/60)/2.5e-4 ≈ 159 for 19.2:1, 250+ for 26.9:1 and up),
unconditionally unstable:

```
A = Kt·gear·eff / r_eff,   B = Kv·gear
w_end = (w + (A·V + τL)·dt / Iaxle) / (1 + A·B·dt / Iaxle)
```

`τL` is the disturbance-observer load (below). `w_end` is clamped to the
V-line no-load point `Vbatt/(Kv·gear)` at the CURRENT bus voltage
(DESPOT-2026-09-28: was `min(spec_free, V-line)` — at fresh-pack 12.8 V the
V-line sits 6.7% above spec, so min() pinned implicit 6.7% below explicit
and the "identical endpoints" claim was false off-nominal; spec survives
only as the Kv-degenerate fallback), one-sided (never below `|w_meas|`,
preserving regen braking) — the physically exact saturation of the V–w
line. A `torque_explicit` twin at measured speed feeds the observer
(`tau_exp_prev`; DESPOT-2026-09-28: the old "feedforward roller-thrust
sizing" consumer never existed — the analytic lateral is slip-driven).

**Disturbance observer** (robot.c:704-719):
`τL = Iaxle·(w - w_prev)/dt - τ_exp_prev`, clamped to **±2× output stall**
(NaN-guarded; reset on teleport via `motor_reset_observer`). The old ±100
clamp gave the observer 400,000 rad/s² of authority and fed a free-spin
limit cycle; 2× stall keeps full locked-rotor tracking while starving the
chaos loop. Holds full stall at lock; converges to ~0 when free.

**Thermal** (both paths heat with `r_eff`):
`dT = I²·r·dt·0.1 - (T-25)·0.01·dt`, floor 25 °C, ceiling 150 °C (magnet
limit). Coefficients are telemetry-grade magic — but NOT "negligible at
FTC currents" (DESPOT-2026-09-28: equilibrium is ΔT_eq = 10·I²·r, so a 2 A
cruise at r≈1.3 Ω settles +52 °C, +20% R; sustained stall is contained in
practice by the pack fuse browning out, not by this model).

**Not modeled:** inductance, PWM, brush drop, gearbox stiction,
load-dependent efficiency, per-motor PTC (fuse is pack-level only),
encoder quantization in torque paths (see §4 for odometry).

## 2. Battery (`modules/ftc/submodules/battery.c`, `.h`)

Fresh-pack assumption 12.8 V (spec nominal is 12.0 V — documented):

```
OCV(soc) = 12.8 - 4·(1-soc)²,  floor 10.0 V   (nominal curve fit, admitted)
Vterm    = OCV - Rint · I_signed,   Rint = 0.06 Ω (NiMH 10-cell + wiring)
```

- **Fuse:** 20 A PTC (`MPE_BATTERY_FUSE_A`), `heat += over·dt/20`
  (~20 A·s to trip, transients ride through), cools `dt/5` below 15 A
  (`MPE_BATTERY_FUSE_RESET_A`), cap 10. Tripped returns **1.2 V brownout,
  not 0** (resettable-fuse correct). NaN/Inf rejected.
- **Drain:** `soc -= I·dt/3600/3.0`, clamped [0,1]; regen credited at 50%,
  suppressed at full charge.
- **Pack integration** (robot.c:651-673): sag uses the **signed** current sum
  (opposing turn currents cancel — correct), fuse uses the **abs** sum
  (stall sums, regen doesn't cool — correct).
- **The fuse dominates high-current manoeuvres, and this is measurable.**
  A full-power mecanum pivot or a tank pivot-in-place puts all four motors near
  stall: 4 x 9.2 A = **36.8 A** against the **20 A** rating. The PTC
  integrates `heat += (36.8-20)*dt/20` = **+0.84 per second**, so it trips in
  **~1.2 s**, after which `battery_get_voltage()` returns the 1.2 V brownout
  and the drive nearly stops. Measured consequence for the same `rotate +1`
  command: **+2.29 rad/s** of yaw from a command-free settle on a fresh pack,
  **+0.46 rad/s** after 330 continuous ticks of driving on that same pack —
  a 5x spread caused entirely by pack state. Straight-line cross-talk is
  state dependent for the same reason (strafe leaks 28% of its lateral rate
  into forward and yaw when driven hard, 2.6% on a fresher pack).
  **Any assertion on drivetrain authority must be structural, not absolute**;
  see `mfs_t_drive_directions` in `docs/TESTING.md`. This is faithful FTC
  behaviour — real robots brown out the same way — and it is reported rather
  than smoothed over.

**Not modeled:** Peukert, temperature/age, transient RC; fuse constants are
engineering picks, not measured PTC curves.

## 3. Drivetrain (`modules/ftc/submodules/drivetrain.c`)

**Mixers.** Tank: even indices left, odd right. Mecanum IK with
max-normalize (refuses `wheel_count != 4` loudly — no geometry for other
counts):

```
FL = f+s-r,  FR = f-s+r,  BL = f-s-r,  BR = f+s+r
```

**Mixer signs verified by measurement, not by algebra (DESPOT-2026-10-02).**
The diagonal pairing above and the anti-diagonal pairing
(`FL=f+s+r, FR=f-s-r, BL=f-s+r, BR=f+s-r`) are BOTH zero-net-force with a
pure torque for this roller pattern, so closed-form reasoning cannot choose
between them — the sign is fixed only by convention, and the convention is
fixed by what the robot does. A/B over a steady-state window (ticks 180-240)
gives, for pure `rotate`:

| mixer | `rotate +1` | `rotate -1` | verdict |
|---|---|---|---|
| shipped (diagonal) | **+2.289 rad/s** | **-2.030 rad/s** | correct sign, anti-symmetric |
| anti-diagonal | -2.030 rad/s | +2.289 rad/s | same magnitude, INVERTED |

Shipped mixer kept. Measured steady-state response of the shipped mixer
(tile floor, 128 iterations, 26.9:1 preset, battery as reached by the rig):

| command | v_x (m/s) | v_z (m/s) | yaw (rad/s) |
|---|---|---|---|
| forward +1 | +0.243 | **+1.173** | -0.029 |
| reverse -1 | -0.240 | **-0.979** | -0.134 |
| strafe +X | **+0.869** | -0.244 | +0.245 |
| rotate +1 | -0.029 | -0.004 | **+0.388** |

Forward leaks 21% laterally and 2.5% into yaw; strafe leaks 28% into forward
and 28% into yaw (real mecanum cross-talk — the X roller pattern is not
symmetric under a pure lateral command, which is why teams re-zero heading
against the field mid-strafe). Pinned by `mfs_t_drive_directions`.

**Measuring direction requires a steady-state window.** Yaw *rate* builds
over roughly 2 s from rest, so a displacement average over the first few
seconds measures the spin-up ramp. A 3.0 s end-to-end average makes
`rotate +1` and `rotate -1` look 7.6x asymmetric (0.029 rad vs 0.221 rad)
when a steady-state window shows them near-perfectly anti-symmetric. This
false signal briefly presented as a rotate sign error; see
`docs/KNOWN_FAILURES.md`.

**No chassis-force cheat.** The old `sin45·Στ/r` lateral injection, 1.1×
breakaway margin, and chassis drag are deleted (verified absent by grep).
Lateral force comes only from engine contacts through the solver; MFS
computes **no friction cone itself** — traction below is slip-threshold
hysteresis (TUNED), not cone budgeting.

**Traction control** (robot.c:721-797): demand `w_expected` from rigid-body
chassis velocity at the wheel center projected on the rolling direction;
`slip = w - w_expected`. Cuts **overspeed only** (`over > 4.0 rad/s` →
scale 0.15; recovers +0.2/step when `over < 2.0`), under-speed keeps full
torque (anti-deadlock). Gates: observability (`|w_expected| < 0.5 rad/s` →
open loop; never regulate on unobservable error) and airborne
(`wheel_bottom <= 0.05 m`).

**Torque shaping** (robot.c:819-885, all TUNED unless noted):

- slew 0.6 N·m/tick (ESC current-slew stand-in; lets grip establish). Its
  memory `wheel_applied_torque` is the torque **actually delivered** on the
  previous tick — stored after the governor and idle brake, not before
  (DESPOT-2026-10-02: it used to hold the largest value the pipeline passed
  through, so a governor or brake clip was invisible to the limiter and the
  instant the wheel came back under the bound the full 0->stall step was
  restored in one tick, defeating the slew exactly when it mattered. Cost of
  the correction, measured and deterministic: tank pivot heading -18.5%,
  mecanum yaw authority -17%),
- governor diode at 1.155× the VOLTAGE-SCALED no-load point
  `Vterm/(kv·gear)` on **measured** speed (DESPOT-2026-09-28: was 1.155×
  spec-fixed — at 10 V sag the true no-load point is 0.83× spec, so the
  old diode permitted ~39% past true free; spec is now the Kv-degenerate
  fallback only). Zero outward push past bound, no reversing slam,
- idle brake: at `|cmd| < 0.05`, back-EMF braking torque clamped to
  `I·|w|/dt` (exact per-tick stop; kills the ±25 rad/s idle limit cycle),
- torque applied as a **couple** `+τ` hub / `-τ` chassis (reaction
  correctness; `MFS_NO_TORQUE_COUPLE` kill-switch).

**Analytic mecanum lateral** (`drivetrain_mecanum_analytic`,
MFS-STRAFE-A, default ON for mecanum only — tank untouched, `false`
restores the articulated 32-roller build for forensics). The 5-link
floor→roller→bearing→hub chain cannot converge in GS-128/512 (mass ratios
to 571:1, cone-projection vs bearing-stiffness fight — see KNOWN_FAILURES
for the solver math), so analytic mode builds NO roller bodies (6 bodies /
4 joints) and the roller geometry survives as the per-wheel axle direction:

```
a = normalize(chassis_R · roller_axle_local),  a.y = 0 (contact plane)
v_slip = (v_wheel + ω × r_c) · a,   r_c = (0,-r_run,0)
F = -a · f_max · clamp(v_slip / VREF, -1, 1)
f_max = MU · N_static_share,   MU = 0.7, VREF = 0.05 m/s
```

Honesty case (why physics, not the retired `sin45·Στ/r` cheat): per-WHEEL
force + `r_c × F` reaction torque (genuinely loads the motor) — never to
the chassis; Coulomb-capped (`|F| ≤ MU·N`, static-share normal, documented
~25% load-transfer error under hard accel); dissipative (`F ∝ −v_slip`,
zero at zero slip — the cheat pushed at full stall with no motion);
contact-gated at TRUE-CONTACT scale (patch bottom ≤ 1 cm, the slop scale —
DESPOT-2026-09-28: was 5 cm of hover force); single tangential model
(analytic hubs ship zero isotropic friction, engine supplies the normal
only — never double-counted); axle in the CHASSIS mount frame (hub-local
would sweep with spin); deterministic (`det_sin/cos` + arithmetic).

**Rolling resistance:** `Frr = Crr·N/nwheels`, `τ = Frr·r_eff` opposing spin
about the axle, only when `|cmd| < 0.05` and `|ω| > 0.01`. `Crr` comes from
world config (engine default 0.02); `N` includes chassis + wheels + rollers.
Velocity monitor counts `> 3 m/s` planar as `clamp_events` (telemetry only;
the old silent clamp is gone).

## 4. Odometry (`drivetrain.c:230-305`, `robot.h:59-69`)

Pure encoder forward kinematics, no fusion — honest:

```
v_fwd = mean(w)·R
v_lat = (FL-FR-BL+BR)/4·R
yaw   = ((FL-FR+BL-BR)/4·R) / 0.44          (mecanum; 0.44 = Lx+Lz, wheel track)
yaw   = ((wl-wr)·R) / 0.48                  (tank; 0.48 = 2×0.24 track)

SIGN CONVENTION — read before changing either line. The textbook mecanum
forward kinematics is v_lat = (-FL+FR+BL-BR)·R/4 and
yaw = (-FL+FR-BL+BR)·R/(4(Lx+Ly)). Both of the above are the NEGATION of that.
This is not a typo. The textbook assumes a right-handed frame (x fwd, y left,
z up); this engine's frame is X=right, Y=up, Z=forward, which is LEFT-handed
(X×Y = −Z, asserted by `mfs_t_odometry_yaw`). Under a handedness flip, polar
vectors (v, F, r) keep their sign and axial vectors (ω, τ) gain one minus sign —
so v_lat matches the textbook and yaw is its negation. The shipped code had
the textbook yaw combination unchanged, i.e. yaw was inverted on BOTH
drivetrains. Arm 0.44 is WHEEL_OFFSET_X (0.24) + WHEEL_OFFSET_Z (0.20); it is
the WHEEL track, not CHASSIS_HALF_X/Z (0.225), which are the chassis collision
box.
```

`R` = first `wheel_effective_radius > 1 mm`, else 0.05. Heading wrapped to
[-π,π] every tick (bounds libm error growth; `deterministic=false` still
honestly claimed).

- **Encoder quantization (real):** `wheel_radians` integrates the true hub
  angle, but odometry differentiates the **quantized** angle
  (`counts = base_ppr × gear_ratio` per output rev; base PPR from
  `motor_preset_base_encoder_ppr` is quadrature-DECODED counts per motor
  rev as the hub reports them: 28 goBILDA/HD-Hex/UP/NeveRest (=7 pulses
  ×4; AndyMark 1120 = 28×40), 24 TorqueNADO (=6 cycles ×4; 1440 = 24×60),
  4 Core Hex (4×72 = 288). DESPOT-2026-09-28: was 7/6/28 — 4× coarse on
  NeveRest/TorqueNADO, 7× fine on Core Hex). Quantization is
  round-half-away symmetric (was floor(): half-count bias on reversal).
  Creep speeds staircase / stick at zero like hardware.
- **`odom_slip` (reporting only, never fused):** 1 when encoder-implied
  motion disagrees with the true chassis beyond 0.25 m/s planar or
  0.35 rad/s yaw, else 0. Retained in the struct for ABI.

## 5. Robot assembly (`modules/ftc/submodules/robot.c`)

- Chassis box half (0.225, 0.075, 0.225) m, **8.0 kg** (~18 lb, inside the
  19.05 kg limit); wheels r = 0.05 m (100 mm class), m = 0.2 kg, offsets
  X ±0.24 / Z ±0.20 m; rest height 0.18 m (5 mm clearance);
  `WHEEL_PRELOAD` = 2 mm below touch inside the 10 mm slop (persistent
  contact without positional fight); mass ratio 8/0.2 = **40:1 → tests pin
  128 solver iterations** (engine default 64 cannot converge it).
- Revolute chassis→wheel on the X axle; hubs rubber 0.9/0.7; mecanum plate
  recessed to 30 mm; `wheel_effective_radius` = roller envelope (all levers
  use it, not the plate radius).
- Rollers: **8×1 per wheel** (was 8×2 — measured worse at 2× cost),
  r = 6 mm, L = 16 mm, 14.2 g steel-derived, X-pattern ±45° (generalizes
  past 4 wheels), free revolute bearings, isotropic rubber 0.9/0.7
  (roller-level anisotropic caps strangled rolling — directionality lives
  in the bearing constraint).
- **Roller spin is quasi-static** (stiff-DOF-slaved, labeled in code):
  time constant ~1 ms ≪ 16.7 ms tick, so bearing spin is prescribed to the
  massless-roller equilibrium from motor command each tick while contacts
  carry all force. See `docs/KNOWN_FAILURES.md` for the strafe frontier.
- Partial-spawn unwind via body watermark + exact-index joint removal;
  per-wheel/per-roller wake (sleep never swallows traction); idle hold
  gated below 0.25 m/s.

## 6. Game robot (`modules/module_1/mfs_module_1.c`, `.h`)

BioBuzz field (12×12 m floor slab + walls + 1.5w×2.5h goal), mecanum robot,
gamepad drive, intake + shooter + balls:

- Balls: r = 21 mm, m = 2.6 g, e = 0.65, μ 0.45/0.35; max 16, carry 3.
  Aero: drag `½·ρ·Cd·A·v²` (Cd 0.47 sphere); Magnus `½·ρ·A·S·ω·r·v`,
  S = 0.1, gate `spin > 10`. No spin decay (declared Module-2 scope).
- Intake: front-lower roller r25 mm/L200 mm/0.05 kg, P-gain 0.2 cap 0.5 N·m
  about the axle; pickup within `R+r+5 mm` compliance + 2 N pull + entrain
  `0.3·dt` surface velocity (magnitudes are picks).
- Shooter: flywheel r50 mm/0.3 kg on a 1.0×H pylon, joint tilted 35°,
  P-gain 0.05 cap 0.3 N·m, target `MFS_SHOOTER_TARGET_RPM` = 4000 rpm,
  ready at 95%, reaction `-τ` on chassis; fire: ball within 8 cm,
  `dv = 0.8·surf`, `F = m·dv/dt` (impulse-vs-force unit fix documented),
  80% transfer. No goal/score detection (`balls_fired` counts discharge —
  honest comment).

## 7. Gamepad (`modules/module_1/submodules/gamepad/`)

F310 XInput map, deadzone 0.15 rescaled, asymmetric js scaling fixed
(-32768/32767 sides land exactly on ∓1). Triggers handle both driver
conventions (0-rest and -1-rest; linear in transit, 0.05 press gate).
`MPE_GAMEPAD_DEVICE=disabled` skips hardware silently (headless). `Back`
resets the **full assembly** (chassis + wheels + rollers + flywheel +
intake by one delta, velocities zeroed, observers + odometry reset).
