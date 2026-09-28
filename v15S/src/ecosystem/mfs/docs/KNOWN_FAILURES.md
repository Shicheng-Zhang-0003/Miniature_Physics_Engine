# MFS Known Failures and Frontiers

Defects and limits that are measured, ticketed, and NOT hidden by gates.
Suite marks are XFAIL (loud + ticketed) or documented observations — never
silent passes. Measurements from the 2026-09-26 audit on this tree.

## [MFS-STRAFE-F1] Mecanum strafe did not transmit (FIXED 2026-09-28)

- **Was:** pure strafe developed ~0.01 m in 3 s vs 0.30 m gated
  (`mecanum`); the 5-link ground → roller → bearing → hub chain did not
  converge in GS-128 (roller-axis sweep 0.002–0.014 m; both roller-spin
  prescriptions ~0.01–0.07 m).
- **Fix (approach (a)):** analytic roller-kinematics lateral force at each
  wheel contact (`drivetrain_mecanum_analytic` in
  `modules/ftc/submodules/drivetrain.c`; model selected in `robot.c` via
  the `mecanum_analytic` flag, default ON for mecanum only, tank
  untouched). No roller bodies/joints are built in analytic mode (6
  bodies / 4 joints); the force is Coulomb-capped (|F| <= 0.7*N,
  static-share normal), dissipative (opposes measured axle-slip, zero at
  zero slip), contact-gated, applied at the wheel (plus r×F motor load) —
  never to the chassis, no sin45 torque term (grep-clean, verified).
  Analytic-mode hubs ship zero isotropic friction so engine contact
  supplies the normal only: a single tangential model, never
  double-counted past the cone.
- **Measured (128 iters, tile floor):** strafe +X 3.40 m / -X 3.66 m in
  3 s (gate 0.30 m); mecanum forward 2.52 m, reverse 3.09 m; diagonal
  (0.5, 0.5) → (1.19, 1.20) m; yaw in strafe <= 0.13 rad; iteration sweep
  64–512 flat at 3.40–3.64 m. Suite hard-gates F1 now.
- **Approaches measured and parked:** (b) wheel-level anisotropic rail
  ellipse (the `MECANUM_USE_RAIL_CONTACT` path): 0.001 m at 128 — the
  solver's static stick converges to lock; (c) iteration raise 64–512 on
  the articulated build: strafe wandered 0.03–0.17 m chaotically
  (non-monotonic), forward decayed 3.10→2.21 m — structural, not
  under-convergence. Engine 512-iteration ceiling tried for the sweep,
  then reverted (unneeded at 128 with the fix).
- **What was ruled out:** the retired chassis-force cheat (`sin45·Στ/r`,
  still gone — verified by grep); roller geometry (X-pattern ±45
  verified); governor starvation and P2P sign (fixed earlier).
- **Revert path:** `ftc_robot_set_mecanum_analytic_default(0)` before
  creation restores the articulated real-roller build (forensics).

## [MFS-STRAFE-F2] Odometry strafe tracking (FIXED 2026-09-28)

- **Transmit half FIXED (analytic lateral):** strafe physics reaches
  0.87 m in 1 s vs 0.10 m required; hard-gated.
- **Tracking half FIXED (voltage-scaled motor/governor bounds):** encoders
  report ~1.08 m vs 0.87 m physics (~25% over; needs <= 30%). Was 88% over
  (~1.53 m vs 0.81 m): the implicit clamp took min(spec, V-line) while the
  explicit observer twin ran unclamped, so at fresh-pack voltage the two
  paths disagreed 6.7% and the observer carried a phantom load into peel.
  Scaling BOTH bounds to the V-line no-load point `Vterm/(kv·gear)`
  (motor.c clamp + robot.c governor, spec kept as Kv-degenerate fallback)
  closed it — isolated by A/B (spec-fixed bounds reproduce 88%, V-line
  bounds give 25%, transmit 3.40 m both ways). `odom_slip` still flags real
  slip elsewhere. Deterministic (bit-identical across runs, -O2 and
  -O1+ASan, zero sanitizer errors).
- **Margin note:** 25% vs 30% allowed is thin. The suite's XFAIL branch
  stays as a fallback tripwire (fires only if tracking ever regresses past
  30%), not as the verdict. Phase-1 forward tracking (8.9%) stays
  hard-gated with room to spare.

## Solver mathematics: why GS could not converge the 5-link chain

- Kept for forensics (the articulated build is one flag away). Per
  roller, the lateral path is floor contact (Coulomb cone projection, a
  non-smooth inequality) → roller body (14 g, I ≈ 2.5e-7 kg·m²) →
  revolute bearing P2P (3-DOF position lock, Baumgarte-biased) + axis
  alignment (2-DOF) → hub (0.2 kg) → revolute → chassis (8 kg). Mass
  ratios up to 571:1 spread the effective-mass matrix spectrum; the
  friction-cone projection and the bearing stiffness fight over the same
  roller velocity each sweep (contacts solved, then joints overwrite,
  2 visits/iteration). Gauss-Seidel converges such coupled
  inequality/equality systems only when the coupling is weak; here the
  bearing is quasi-rigid next to a 14 g roller, so successive sweeps
  oscillate between sticking the contact and satisfying the joint —
  the fixed point exists (the true articulated solution strafes) but
  the contraction factor is ~1 at 571:1, hence 0.03–0.17 m of chaotic
  partial convergence at 64–512 iterations instead of monotone
  refinement. Forward survived because its force path projects mostly
  onto the P2P position lock (stiff, convergent), while lateral lives
  in the cone-projection/bearing-stiffness null fight. Reduced
  articulation (analytic: 6 bodies/4 joints, no bearing in the loop)
  removes the coupling rather than out-iterating it.

## Jointed air-spin limit cycle (contained frontier)

- **Symptom:** jointed free-spinning wheels oscillate instead of settling:
  measured motor-model 86 rpm vs true wheel 799 rpm, current slamming
  ±stall (revolute-to-kinematic-chassis impulses vs slew/governor/implicit
  solve).
- **Contained by:** governor diode (1.155× free, measured-speed gated) +
  0.6 N·m/tick slew. An airborne bypass of both was tried and reverted
  same-day after runaway to ±1900 rpm.
- **Suite status:** T6/T8 test the motor endpoint **isolated** (no
  joints/world): free speed lands at the voltage-scaled spec (237.9 rpm =
  223 × 12.8/12, 0.0% error), power cut coasts to rest with no reversal.
  The jointed-air plant is covered by T11 stability (finite, no NaN) only.

## Engine-side observations correctly fenced out (not MFS bugs)

- **~15 mm left-high static settle** that follows world X under 90°
  rotation: solver pair-order lock-in, not assembly geometry. Harmless to
  all gates.
- **Steady pure roll on slop contacts:** the engine rolling model holds
  velocity on slop-band contacts; T14 gates position + no-runaway rather
  than exact rest.
- **Free-spin-in-vacuum rigs** lift the whole robot (chassis + wheels +
  rollers) with observer resets; whole-assembly teleport without them
  winches wheels through pendulum chaos (historical false failures).

## Historical (fixed, kept for forensics)

- Preset table mixed NeveRest-class RPMs with invented torques under
  goBILDA names — replaced with spec-traced values (`docs/FTC_SPECS.md`).
- `MOTOR_REV_CORE_HEX` was 60 RPM / 1.40 N·m / 10 A vs published
  125 / 3.2 / 4.4 — replaced.
- Chassis-force strafe cheat ran 4.3× over the ceiling — deleted.
- Torque applied hub-only (chassis yawed for free) — now a couple.
- Plate-radius levers (observer/governor/odometry at 30 mm) — now the
  50 mm running envelope everywhere.
- Shooter impulse-vs-force 60× unit bug; intake/shooter id-vs-index bugs;
  roller-joint leak on partial spawn; gamepad double-init fd leak.
