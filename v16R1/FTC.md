# MPE FTC Robotics / Drivetrain — v15S

> FTC (First Tech Challenge) robotics stack: mecanum drive, drivetrain,
> motor model, disturbance observer, traction control, odometry, and the
> BioBuzz regulation league. Lives in `v16R1/src/ecosystem/mfs/`.

---

## 1. Overview

The FTC stack models mecanum wheels as **real rollers on real free bearings**
with real contacts against the floor. Lateral thrust is an **emergent
consequence of rigid-body dynamics plus Coulomb friction**, bounded by the
friction cone for free — no chassis-force is ever injected.

Analytic mode (`MFS_MECANUM_ANALYTIC_DEFAULT = 1`) builds **no roller
bodies** (hub contacts the floor directly; 6 bodies / 4 joints, trivially
converged) and computes the roller geometry analytically in
`drivetrain_mecanum_analytic()`. Articulated mode (8 rollers × 4 wheels on
free revolute bearings) is kept for forensics — it does not converge in this
engine's Gauss-Seidel solver (measured strafe 0.002–0.014 m vs >0.3 m
required).

---

## 2. Drive Commands

| Input | Meaning |
|---|---|
| Gamepad left stick Y | forward (+Y is tree forward = +Z) |
| Gamepad left stick X | strafe (robot's right = −X) |
| Gamepad right stick X | rotate |
| Gamepad START | toggle control |
| Gamepad BACK | cycle controlled robot |
| Gamepad LB+RB | e-stop |
| Gamepad LT (hold) | intake |
| Gamepad RT (press) | shoot (analog power) |
| Keyboard G/B | forward/back |
| Keyboard V/N | strafe left/right |
| Keyboard C/M | rotate left/right |
| Keyboard , (hold) | intake |
| Keyboard . (press) | shoot once |

The gamepad owns motion while connected; keyboard takes over when no pad is
present. Opposing pairs cancel (G+B, V+N, C+M); axes combine.

---

## 3. Drivetrain Model

### 3.1 Motor (`motor.c`)

DC motor electrical model:

```
Kt = stall_torque / (gear_ratio × efficiency)   (motor-shaft ideal)
R  = V_nominal / stall_current
Kv = V_nominal / (free_speed_rad_s × gear_ratio)
I  = clamp((V·command − Kv·ω_shaft) / R_eff, −I_stall, +I_stall)
τ_out = Kt·I × gear_ratio × efficiency
```

Copper thermal derating: winding resistance rises with modeled temperature
(`r_eff = R × (1 + 0.00393 × (T − 25))`). Equilibrium `dT_eq = 10·I²·r`;
150 °C magnet ceiling clamps the integrator.

Back-EMF braking is a damper — brings a coasting wheel to rest, never
reverses it. Brake clamp is HALF the stop-in-tick value (geometric decay,
no overshoot possible no matter what contact adds).

Free-speed governor (diode, not deadbeat): below free speed, full stall
torque for breakaway; only outward push past the voltage-scaled no-load
point `w_free(V) = V_term / (Kv × gear)` with 1.155× hysteresis is clipped.
No reversing slam, no limit cycle.

### 3.2 Disturbance Observer (`motor_observe`)

External load = measured net torque effect − last tick's explicit motor
torque. Clamped to ±2× output stall (not ±100 N·m — the old ±100 gave the
observer 400,000 rad/s² of authority over a 2.5e-4 axle, reading joint
constraint impulses as phantom load and causing a limit cycle: motor 86 rpm
vs true wheel 799 rpm). Same convergence point, stable path.

### 3.3 Traction Control

Compare wheel speed against the rolling speed the chassis motion demands at
this wheel (rigid-body velocity at the wheel center, projected on the rolling
direction). A wheel spinning far from demand is slipping: cut its torque so
kinetic friction re-captures it. Cut **only overspeed** (wheel outrunning
travel = burnout); under-speed keeps full torque so the wheel spins up to
rolling speed.

Hysteresis: cut to 0.15× when over > 4.0 rad/s; recover above 2.0 rad/s or
after 20 ticks (probe +0.2 every 20th tick). Genuine burnout re-slams next
tick; the trap escapes in ~5 probes.

### 3.4 Yaw Stability

- **Governor**: delivered motor torque scales down past 3.0 rad/s, reaching
  the traction-cut floor (0.15×) at 4.5. Applied equally to all wheels so
  mixer directionality/anti-symmetry is preserved.
- **Active damper**: pure yaw couple opposing chassis spin, deadbanded below
  3.0 rad/s, ramping to full by 4.5 (0.6 N·m·s at full ramp). Differential
  (mirrored per side) = zero net force, pure moment — planar drive is
  untouched. Tank peaks at 1.2 rad/s never reach the band.

Sustained rotate capped ≈ −4.4 rad/s (was −13.5 still creeping).

### 3.5 Rolling Resistance

`F_rr = C_rr · N`, applied as opposing torque about the wheel axis
(`F_rr × R_eff`), not a velocity-proportional force on the chassis.
`C_rr` from world config (`world.rolling_resistance_coeff`, default 0.02).
Free-rolling only: a driven wheel's net torque is the motor's business.

---

## 4. Odometry

**Pure encoder kinematics** — deliberately NOT fused with chassis velocity.
The old code compared encoder lateral estimate against chassis-derived
lateral velocity and, on disagreement, overwrote the estimate with chassis
motion (a dead-wheel stand-in) while raising `odom_slip`. That silently
injected ground truth into the "odometry" and made it agree with the physics
by construction.

Now that the mecanum lateral force is emergent from real roller contacts,
the encoders genuinely observe it, so no fusion is needed. Any remaining
encoder-vs-truth disagreement is real slip, reported as `odom_slip` rather
than papered over.

```
v_fwd  = mean(ω_i) × R
v_lat  = (FL − FR − BL + BR)/4 × R
yaw    = (−FL + FR − BL + BR)/4 × R / (Lx + Lz)
```

`R` = `wheel_effective_radius` (roller pitch radius + roller radius, not the
hub plate). Encoder quantization: integrate true angle, quantize to integer
counts (base_ppr × gear_ratio), differentiate the quantized angle —
low-speed motion staircases/sticks at zero exactly like hardware.

`odom_theta` wrapped to [−π, π] every tick (libm trig error grows with |θ|;
uses `det_sin`/`det_cos` for determinism).

**Historical note:** odometry yaw was sign-inverted (measured against two
independent yaw measures that cannot both be wrong the same way). Corrected
to `(wl − wr)`; moment arm is `(Lx + Lz)` — the wheel track, not the
chassis box (the chassis collision box half-extents have nothing to do with
where the wheels sit).

---

## 5. BioBuzz Regulation League

The full 2026-2027 field in SI — 144 in floor, 12 in walls, centre hive
frame (pivots 43.95 in) with 2 tipping hives × 2 cells, 4 flowers, zone
tapes, AprilTag IDs recorded — plus §10.3 staging (40 pollen + 16 nectar)
and Table 10-2 scoring (tips, cells, flower ownership/bottom, gardens,
leave/park predicates, RP). Tips are weight-census events (not a damper
solve — disclosed).

Terminal: `ftc biobuzz [build|spawn]`, `ftc biobuzz enter red|blue`,
`ftc biobuzz score`.

---

## 6. Known Issues

| Issue | Status |
|---|---|
| `tank` red (14/17 MFS) | Pre-existing CCD obstacle margin; per-pair fix is the candidate |
| `drive_directions`, `odometry_yaw` red | Same global margin root cause |
| Jointed air-spin limit cycle | Contained by bearing damping + diode+slew; documented frontier |
| MOTOR-III observer limit cycle | Documented; driven path is a limit cycle at −61.3% |
| Encoder odometry over-reports under slip | No slip term in model; land motor-chain step 2 together with slip term |

---

## 7. Verification

`build_tests.sh` — `mfs_suite --all` (30 gated) + build gates + ungated
diagnostics. **Status 2026-10-14: 29/30 green** (sole red `tank`,
pre-existing).

Run: `cd v16R1/src/ecosystem/mfs && ./build_tests.sh`
