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

## [MOTOR-III] Disturbance observer feeds shaping back as external load (OPEN, root-caused further)

This is the blocker for [MOTOR-I]/[MOTOR-II], and fixing it revealed a second
problem. Recorded 2026-09-29.

- **Defect:** the observer computes
  `tau_L = I*(w - w_prev)/dt - tau_exp_prev`, and `tau_exp_prev` is the RAW
  unshaped `Kt*I*gear*eff` (`motor.c:224`). The torque actually delivered is
  then multiplied by the traction scale, slew-limited, governor-dioded and
  idle-brake-clamped. So `(delivered - raw)` is booked as "external load"
  every tick. With the traction cut at 0.15x that is 85% of motor torque
  reported as an external disturbance, which raises the predicted `w_end`,
  which raises back-EMF, which LOWERS delivered current — a positive
  feedback path from the shaping stages back into torque, bounded only by the
  +/-2x-stall clamp.
- **Attempted fix (verified directionally, NOT applied):** reference the
  torque actually banked into the axle (post-shaping). Measured effect on its
  own: mecanum strafe transmit 0.87 m -> **1.26 m** (+45%) and odometry
  distance error -> 11%. So the fix is directionally right and large.
- **WHY IT IS STILL NOT APPLIED — the new finding:** with the corrected
  reference, the motor STALL endpoint breaks. Measured T7 output torque
  **2.21 N·m against a 3.73 N·m spec (40.7% low)**, from
  `load_torque = -2.079`.
  Cause: a disturbance observer measures the gap between predicted and actual
  acceleration. At a BLOCKED rotor (wheel held at zero) it reads "I delivered
  2.2 N·m and nothing happened" and infers a braking load of -2.2, which
  lowers the next tick's prediction, which lowers the delivered torque. That
  is a ratchet, not convergence: a stalled motor can never hold stall torque
  against any observer that cannot distinguish "externally held" from
  "loaded".
- **To land it properly**, the observer needs to recognise a blocked rotor
  rather than treating it as load. A workable form: treat
  `|I*(w - w_pred)/dt|` exceeding a fraction of output stall as SATURATION —
  the wheel is being held, so freeze the load estimate at its last value
  (and, if it stays saturated, drive it to zero) instead of accumulating a
  braking term. Only then should the correct motor model from [MOTOR-II] be
  re-applied; doing it in the other order destabilises the stall endpoint, and
  doing both at once zeroes drive entirely (measured: strafe transmit 0.00 m).

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

### REL-PTR-2026-09-29 — Mouse lock broken on Wayland (and X11) since v1 playable — FIXED

*(Engine-level defect; recorded here because this file is the running ledger of
truthful known-failure accounting for the tree.)*

**Symptom.** Click to lock: the mouse appears to lock, but after moving far
enough it "exits the window" and never re-locks. A 360-degree camera spin is
impossible. Reported as long-standing; the old GTK3 build was worked around by
forcing X11.

**Root cause (two independent defects).**

1. *Lock was never implemented, only cosmetic.* `mouse_lock_enable()` set a
   `"none"` cursor and nothing else. Camera input was derived from the
   **absolute** cursor position: `dx = x - last_x`. That design only works if
   the pointer is continuously re-centred; otherwise it measures "how far from
   where I started", which saturates at the screen edge.

2. *The X11 rescue hatch was itself dead under GTK4.* `mouse_lock_reset_centre()`
   issued `XWarpPointer` after every motion event, keeping deltas bounded
   forever — that is why it ever worked. But it was guarded on
   `#ifdef GDK_WINDOWING_X11`, a **GTK3 macro that GTK4 does not define**, so
   under v15S the warp compiled out entirely. Mouse lock was therefore broken
   on **both** backends. "It works if I force X11" only ever described the
   pre-port GTK3 build.

**Why Wayland could never be patched the same way.** Wayland forbids client
pointer warping and GTK4 removed `gdk_seat_grab`, so the cursor cannot be
re-centred. It therefore travels to the physical screen edge, the compositor
clips motion there, and the camera stalls; once the pointer leaves the surface
GTK stops delivering motion, the view freezes, and `is_mouse_locked` stays
`true` with no events ever arriving — exactly the "exits the window and no
relock" report. Hiding a cursor is not a lock.

**Fix.** Real lock via the relative-pointer protocol GTK4 does not wrap,
`zwp_relative_pointer_manager_v1`. Its specification states that when "a
pointer motion caused the absolute pointer position to be clipped by for
example the edge of the monitor, the relative motion is unaffected by the
clipping" — precisely the failure above, and precisely the first-person-camera
requirement: the on-screen cursor never moves, so screen edges are irrelevant.

- `ui_input/wayland/relative-pointer-unstable-v1.xml` plus `wayland-scanner`
  generated client header and code.
- `ui_input/mouse_lock.c` binds the global from the `wl_registry`, attaches a
  `zwp_relative_pointer_v1` to the surface's `wl_pointer` (via
  `gdk_wayland_device_get_wl_pointer()`), and accumulates unclipped deltas.
- `ui_input/input_control.c` → `on_mouse_movements()` now calls
  `mouse_lock_take_relative_delta()` first and **ignores absolute coordinates
  entirely** when a relative delta is available. That is the load-bearing
  half: mixing the two sources is what kept the edge stall alive.
- X11 warp restored behind an explicit `-DMPE_GTK4_X11_WARP=1` instead of the
  GTK3-only macro.
- Fallback: without the protocol, lock degrades to the old hide-the-cursor
  behaviour and stays edge-limited rather than breaking.

**Verification.** The live compositor (`$WAYLAND_DISPLAY`) was probed and
**does advertise `zwp_relative_pointer_manager_v1`**, so the fix is active
here, not merely compiled. Engine builds warning-free; suite 34/34; full
profile 206/206. What CI cannot prove is a physical 360-degree mouse spin, as
no automated check can drive a real pointing device to a screen edge and
observe the compositor — **that one is for human hands**, and it takes five
seconds: launch, click to lock, spin the mouse hard in one direction, and
confirm the view keeps turning past where the cursor would have hit the edge.

### MOTOR-III-2026-09-29 — Disturbance observer cannot see a blocked rotor (PARTIAL: gate added, effect unproven)

**RESOLUTION UPDATE 2026-09-29 (later, same day).** The missing gated test
now exists: `mfs_t_stall_endpoint` in `tests/mfs_suite_a.c`, registered as a
blocking case (MFS is 9/9, was 8/8). It has two phases and the split between
them is the whole finding:

* **Phase 1, open loop** (motor alone, shaft fed exactly 0): `output_torque =
  3.6509 N.m` against the published `3.7265 N.m`, **-2.03%**. Current 9.013 A
  against the 9.2 A spec, back-EMF exactly 0. Gated at 10%. **The electrical
  model is honest.**
* **Phase 2, closed loop** (the disturbance observer feeding its estimate
  back into `motor_update_load`, which is the path that actually regressed):
  `output_torque = 0.7081 N.m`, **-81%**. `load_torque = -3.6983`, i.e. the
  estimator converged correctly to within 1% of `-stall`.

So the observer is *not* the broken part, and neither is the motor. The
breakage is in the **observer -> implicit-solve coupling**: `motor_update_load`
treats a co-rotating load as back-EMF. Feeding `tau_L ~= -stall` into

    w_end = (w + (A*V + tau_L) * dt / I) / (1 + A*B*dt / I)

pushes `w_end` up; back-EMF rises with it; current collapses; the transmitted
torque falls to a fraction of stall. The motor is told the load is absorbing
torque, so it correctly stops driving — and then reports that it is barely
driving.

**This disproves the blocked-rotor gate below as a fix**, which is worth
recording because the gate was written on the same intuition. The gate snaps
`load_torque` to `-stall_out`; `load_torque` is already within 1% of that.
Snapping the *input* of a broken coupling more precisely cannot fix the
coupling. The gate is retained (correct, inert, costs nothing) but is no
longer proposed as the remedy.

Phase 2 is reported as a loud `[XFAIL][MOTOR-III]`, not a silent pass and not
a blocking red, so the suite stays green while the fix is written. **The real
remaining work is therefore not the motor model or the observer: it is
deciding what a co-rotating external load should do to `w_end` in
`motor_update_load` so that a held shaft is a stable fixed point at full stall
torque rather than a runaway that starves its own current.**

**Status update, honest version.** A blocked-rotor gate was implemented in
`modules/ftc/submodules/robot.c` (inside the observer branch, gated on
`fabsf(tau_ref) >= 0.95f * stall_out` and `fabsf(wheel_speed) < 0.5f` rad/s,
snapping the external-load estimate to `-sign(tau) * stall_out`).

The reasoning is sound and is *not* a test hack: a motor at its torque limit
whose shaft is not turning is, by definition, transmitting its full stall
torque to the load, and that is the torque-speed endpoint of the same model the
solver already uses for its free-speed bound.

**But it made no measurable difference.** A/B on the gated MFS suite, same
build flags, both from the correct working directory:

| metric | without gate | with gate |
|---|---|---|
| tank turn heading (target 2.3003) | 2.4167 | 2.4167 |
| odometry distance error | 9.0% | 9.0% |
| strafe physics dx | 0.8739 | 0.8739 |
| strafe odometry dx | 1.1202 | 1.1202 |

Identical to four decimal places, so the gate **never fires** in the gated
suite — those cases do not hold a wheel at stall. The failure it targets
(2.21 N·m against a 3.73 N·m spec on the straight-push endpoint) is therefore
still unreproduced *by the shipped tests*, which is precisely why the number
was never improved. It is retained because it is correct, costs nothing, and
regresses nothing — but it is **not** presented as a fix, because a change
that cannot be measured to help has not demonstrated it helps.

**What is still actually blocking the motor chain.** The part that moved
strafe from 0.87 m to 1.26 m was referencing the *delivered* torque rather
than the commanded torque (accounting for reflected rotor inertia), not this
gate. That change is not in the tree: it broke the stall endpoint
(2.21 vs 3.73 N·m) and was reverted. The correct landing order is therefore
still: (1) make the stall endpoint measurable as a *gated* test, (2) land
delivered-torque accounting, (3) then retune lateral `VREF`. Step 1 does not
exist yet, and without it steps 2 and 3 cannot be verified rather than merely
believed. That is the actual next piece of work, and it is a test-authoring
task, not a physics task.

**Runner honesty fix (shipped with the above).** While adding this test,
`tools/test_runner.py` was found to **discard every MFS `[XFAIL]` marker**.
`parse_skip_xfail()` was only ever called on the `suite-v2` stage; the MFS
stage never called it, and the per-case markers live in `mfs_suite.run.log`
rather than the captured stdout, so they had no path to the summary at all. A
`206/206 ... xfailed: 0` line could therefore hide a known-red frontier with
no trace — the exact false-green shape this runner exists to prevent. The
pre-existing `MFS-STRAFE-F2` marker had been invisible this whole time. The
MFS stage now reads the run log and surfaces both. The full-profile line
changed from `206/206, xfailed: 0` to the truthful `206 passed, 2 xfailed`
(one MOTOR-III, one MFS-STRAFE-F2), with `0 blocking failures` unchanged.

**Process note (recorded because it nearly shipped as a false finding).**
While A/B-ing this, `mfs_suite --all` appeared to report `[FAIL] ftc_hotload`
while the QA runner reported `[PASS]`, which looked exactly like a false green
in `tools/test_runner.py`. It was not: `ftc_hotload` is sensitive to the
working directory, and the failing run had been launched from the project root
instead of `v15S/src/ecosystem/mfs`. Run from the correct cwd it passes
deterministically (3/3). The runner was correct; the ad-hoc invocation was
wrong. The genuine (much smaller) weakness is that `ftc_hotload` prints
`[FAIL] ftc_hotload (failures=N)` with no reason, so a genuine failure of that
case is hard to diagnose — worth improving, but it is a diagnostics gap, not a
masked failure.
