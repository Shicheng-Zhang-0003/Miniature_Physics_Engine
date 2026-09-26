# MFS Known Failures and Frontiers

Defects and limits that are measured, ticketed, and NOT hidden by gates.
Suite marks are XFAIL (loud + ticketed) or documented observations — never
silent passes. Measurements from the 2026-09-26 audit on this tree.

## [MFS-STRAFE-F1/F2] Mecanum strafe does not transmit (structural)

- **Symptom:** pure strafe develops ~0.01 m in 3 s vs 0.30 m gated
  (`mecanum`); odometry strafe phase sees physics dx ≈ 0.005–0.012 m while
  encoders report ≈ 0.6–0.76 m (correct reporting of real slip).
- **Mechanism:** the 5-link ground → roller → bearing → hub constraint chain
  does not converge in the Gauss-Seidel solver at 128 iterations. Measured:
  roller-axis sweep 0.002–0.014 m; command-prescribed roller spin +0.01 m;
  measured-speed prescription −0.06 m (wrong sign, same dead magnitude —
  so it is a solver frontier, not a prescription-source question).
- **What was ruled out:** the retired chassis-force cheat (`sin45·Στ/r`,
  gone — verified by grep); roller geometry (X-pattern ±45 verified);
  governor starvation and P2P sign (both fixed, strafe still dead); the rail
  aggregate model (also measured dead, plus forward peel-out).
- **Fix needs:** solver-level work — reduced-coordinate articulation or a
  direct roller constraint. Not gating pressure.
- **Suite status:** both strafe gates XFAIL with measured values printed;
  forward / tank / hotload / module_1 / physics_truth all hard-gate.

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
