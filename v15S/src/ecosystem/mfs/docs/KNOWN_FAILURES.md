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

## [MFS-STRAFE-F2] Odometry strafe tracking (PARTIALLY FIXED — TRACKING IS STILL OPEN)

- **Transmit half FIXED (analytic lateral):** strafe physics reaches
  0.87 m in 1 s vs 0.10 m required; hard-gated. Confirmed still green.
- **Tracking half IS NOT FIXED. This entry said FIXED; it was not.**
  Re-measured 2026-10-03 by running the gate:

      Phase 2: strafe: physics dx=0.6318 odometry dx=0.9076
      [XFAIL][MFS-STRAFE-F2] strafe phys=0.6318 odom=0.9076 (tracking open)

  That is **+43.7%** (|0.9076 - 0.6318| / 0.6318), outside the 30% band. The
  entry previously claimed "encoders report ~1.08 m vs 0.87 m physics (~25%
  over; needs <= 30%)" — **neither number matches what the suite prints**, and
  the 25% claim sat inside the band while the actual 43.7% does not. So the
  write-up reported a passing margin for a check that is in fact red.

  What the earlier work actually achieved: the implicit clamp took
  min(spec, V-line) while the explicit observer twin ran unclamped, so at
  fresh-pack voltage the two paths disagreed 6.7% and the observer carried a
  phantom load into peel. Scaling BOTH bounds to the V-line no-load point
  `Vterm/(kv*gear)` (motor.c clamp + robot.c governor, spec kept as
  Kv-degenerate fallback) narrowed it substantially — from 88% over to
  something much smaller — but did not close it. Transmit is 2.2872 m
  (previously documented as 3.40 m, itself stale; the gate prints the truth).
- **Why it was not caught:** the XFAIL branch is surfaced by the runner as a
  non-blocking marker, so a red frontier is *visible* without being *blocking*.
  That is the correct design for a known-red frontier — but it means "green
  suite" and "no open defects" are different claims, and this entry conflated
  them. Corrected here.
- `odom_slip` still flags real slip elsewhere. Behaviour is deterministic
  (bit-identical across runs, -O2 and -O1+ASan, zero sanitizer errors).

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

## [STALL-THERMAL] The -16% stall softening was thermal, not the observer (RESOLVED 2026-10-03)

Recorded here because it was **twice** attributed to the wrong mechanism, and
because the wrong attribution had already hardened into a test tolerance.

The closed-loop locked-rotor endpoint read **3.1279 N·m** against a
**3.7265 N·m** spec (-16%) and was blamed on "the observer -> implicit-solve
coupling". The gate was then widened to **25%** to accommodate it. Measured
2026-10-03 with the observer armed identically in both runs and copper
temperature as the only variable:

| | output torque | vs spec |
|---|---|---|
| thermal active | 3.12793 N.m | **-16.062%** |
| temperature pinned at 25 C | **3.72650 N.m** | **+0.000%** |

**The observer contributes exactly nothing.** The mechanism is `motor.c`'s
copper model `r_eff = R*(1 + 0.00393*(T - 25))`, which reaches `r_eff/R =
1.19237` at 73.95 C after 300 stall ticks (5 s), and `1/1.19237 = 0.8387`,
i.e. the whole -16.1%. A motor held at 25 C delivers the full spec stall
torque through the observer without difficulty.

**Consequence, and why it mattered:** a 25% band chosen to absorb a thermal
artefact is indistinguishable from a warm motor, so the gate could not have
failed for the reason it exists -- a genuine 25%-off observer regression would
have sailed through. Fixed by removing the confound rather than absorbing it:
`mfs_t_stall_endpoint` now gates BOTH the derated value against the
`r_eff(T)` model (2%) and the 25 C value against spec at **2%**, 12.5x tighter
than the band it replaces. Header comment at `motor.h` corrected too.

**Do not conflate with [MOTOR-III] below.** That entry is a real and separate
defect (the observer books post-shaping losses as external load, which
destroys large-load torque). It is NOT what this 16% is. Two different things
were being called one thing for three days.

---

## [FREE-SPEED-DEAD-OBSERVER] Both free-speed gates measured a disabled observer (RESOLVED 2026-10-03)

The readme claimed the back-EMF model made "stall **and free speed** exact at
any bus voltage", and two gates appeared to back it: `mfs_t_motor_free_speed`
and the `external_truth` sub-7 free-speed check. **Neither called
`motor_observe()`.** With no call, `m.wprev_valid` stays 0, `tau_L` is
identically 0, and the disturbance observer is simply absent from the
measurement -- while the shipped drivetrain arms it every tick
(`robot.c:764`). The gates measured a configuration the engine never runs in.

Same rig, same build flags, 180 ticks:

| | free speed | vs the no-load line |
|---|---|---|
| observer **not** armed (gates as written) | 237.8667 rpm | **+0.0000%** |
| observer armed, exactly as `robot.c` | 92.0172 rpm | **-61.3156%** |

So the exact-0.00% figure was real but described a dead estimator, and the
driven wheel sits 61% off the no-load line.

**Resolved by splitting the claim, not by picking a winner.** Free speed is
independent of winding resistance and of load *by construction*:
`w_free = V/(kv*gear)` with `kv = V_nom/(w_free_spec*gear)`, so `R` cancels
exactly. That makes the open-loop check a sharp, estimator-independent test of
the electrical model and the preset constants -- so it is **kept, and now
labelled open-loop** in both gates. The observer-armed number is not a free
speed at all; it is the air-spin / [MOTOR-III] limit-cycle fixed point. It is
printed with its honest -61.3%, gated on being finite and bounded (not running
away), and tracked as a defect where it belongs. Pinning a tolerance to a
limit cycle would be inventing a specification, which is precisely the
fabricated-tank-target failure retracted on 2026-09-29.

---

## [MOTOR-III] Disturbance observer feeds shaping back as external load (OPEN, root-caused further)

This is the blocker for [MOTOR-I]/[MOTOR-II], and fixing it revealed a second
problem. Recorded 2026-09-29.

- **Defect:** the observer computes
  `tau_L = I*(w - w_prev)/dt - tau_exp_prev`, and `tau_exp_prev` is the RAW
  unshaped `Kt*I*gear*eff` (`motor.c:228`). The torque actually delivered is
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

## [RELEASE-STORM] Zero-command motor limit cycle + glide equilibrium (FIXED 2026-10-04)

Live-session report: wheels tilt/pivot off their shafts, and motor speeds
never revert to 0 on stick release. Measured headless (drive 180 ticks,
release, mecanum robot, tile floor):

- **Storm:** at zero command the implicit observer path planned torque
  against last tick's speed discrepancy; on light wheels with
  joint-hammered reads that is alternating ±stall — a 4-tick ±0.6
  slew-rail limit cycle with wheels ±70 rad/s forever. Fix: at
  |command| < 0.05 run the explicit regen path (passive damper, exact,
  memoryless), bypass slew (an exact-stop brake needs no ramp; slewing it
  caused a 4-tick bang-bang), halve the idle-brake clamp (exact-stop +
  contact in the same tick overshoots past zero and ping-pongs).
- **Glide:** with the storm gone, wheels held ±25 rad/s and the vehicle
  glided ~60 s. Analytic-mode hubs ship zero engine friction and the
  roller force is lateral-only, so nothing coupled wheel spin to chassis
  translation longitudinally (kill-tests: analytic OFF → full stop;
  real hub friction → full stop). Fix: at idle, analytic lateral parks
  and hubs get real 0.9/0.7 engine friction (one tangential model at a
  time, never double-counted past the cone). Driven operation is byte-for-
  byte the validated model (strafe 2.2872 m identical).
- **Tilt** was the joints telling the truth about the storm: 40:1 mass
  ratio needs the 128 iterations every MFS test pins (default 64 wobbled
  axles to 15°). Live spawn paths now auto-raise to 128, loudly.
  Post-fix: full stop <1 s after release, tilt ≤1.4° and flat.
- Locked by the new `release_settle` gate (drive ≥ 0.5 m, then chassis
  < 0.1 m/s, wheels < 2 rad/s, axles < 3°). MOTOR-III above is untouched:
  the observer still runs (and still limit-cycles) on the DRIVEN path;
  idle simply no longer asks it to plan torque.
- Idle switching is HYSTERETIC (engage <0.03, release >0.08, mode stored
  in the hub friction fields so motor path, analytic gate and friction
  switch together): a rescaled-deadzone stick riding a single 0.05
  threshold flapped all four every tick. Explicit path also publishes
  tau_exp_prev now (stale drive reference poisoned post-idle re-drive:
  measured strafe 10x dead until the handshake).

## [ABUSE-TILT] Sustained harsh mixed drive walks axles to flip (OPEN frontier, pre-existing)

Recorded 2026-10-04. Sibling of the fixed items above, NOT fixed: ~90+
ticks of sustained full mixed drive (0.5 fwd + 0.5 strafe + 0.3 rotate)
walks wheel axles 5° → 69° → 180° (frozen-in-tilt end state, mounts hold,
never NaN). Gated behaviors (single-mode bursts ≤180 ticks) stay ≤1.4°.

- Pre-existing: fully pre-change tree (pre-friction-fix engine + pre-change
  MFS) blows up identically (163.8° at the same tick), so no fix above
  caused it — but none cured it either.
- Six measured negative results (do not re-attempt without new evidence):
  analytic reaction projected to axle (same onset tick); axis-drift beta
  0.1 → 0.3 (same blowup); mount-frame motor axle (earlier: 175°);
  observer frozen (delayed only, 128°); proportional traction-cut engage
  (same blowup — AND moved tank 0.0642 → 0.0397 m and broke drive
  antisymmetry, so the slam is load-bearing: reverted); slew 0.6 → 0.2.
- What IS established: needs motor torque (zeroed-torque abuse stays
  0.2°/1800 ticks); per-tick wheel jumps hit ±85 rad/s (peel + slew-rail
  steps on 2.5e-4 inertia); the 0.05-command idle boundary is not the
  trigger (hysteresis changed nothing here); recovery is impossible once
  flipped (static rim-lock beats the velocity-level drift corrector —
  respawn is the recovery).
- Next honest measurement, not yet taken: per-wheel torque-budget cap at
  the contact cone (motor can never demand more than the ground can
  transmit) would remove peel overspeed structurally, but it retunes all
  drive authority and the suite above proves this loop is gate-calibrated
  and fragile — budget for a full re-baseline pass before attempting.

## [DESPOT-2026-10-02] Full mathematical / programming / operational audit

Recorded from a standalone run against this tree. Every number below was
measured, not estimated; the probe programs live in `temp/audit/`.

### Operational — the drift guard was blind, and the docs were wrong about git

- **The sync checker could never run from the tree it was advertised for.**
  `docs/SYNC_CONTRACT.md` states "a local copy also lives at this repo's root
  (`./sync_mfs_check.sh`) for standalone use." It could not work there. The
  path resolution was `ROOT="$HERE/.."` then `EMBED="$ROOT/v15S/src/ecosystem/mfs"`,
  which is right when the script sits at `<475>/tools/` but not when it sits at
  the 461 root: there `HERE/..` is the shared *parent* directory, so it
  searched for `…/projects/v15S/src/ecosystem/mfs` instead of
  `…/projects/475-MPE/v15S/src/ecosystem/mfs`. Every standalone run died with
  "embedded not found" and exit 1 — a status that reads like a misconfiguration,
  not like blindness.
- **The drift it existed to catch was present.** This tree was last committed
  28 Sep; its twin had been committed through 30 Sep. 11 files had drifted,
  including **4 gated tests that do not exist here** (`stall_endpoint`,
  `intake_stop`, `shooter_axis`, `ball_spin`), a `motor_observe()` refactor
  that moved the disturbance observer into `motor.c`, and a 971-line
  `KNOWN_FAILURES.md` against this tree's 122. The twin was verified to be a
  **strict superset** (0 lines of unique content here), so mirroring was
  lossless, and the suite went 8/8 → 12/12 on mirror.
- **The checker now distinguishes three outcomes, not one:** 0 = in sync,
  1 = drift found, **2 = could not locate a tree (the guard did not run)**.
  It also prints both resolved paths so a green result is attributable.
- **Build artifacts were committed.** `README_MFS.md` and
  `docs/ARCHITECTURE.md` both described `build/` and `plugins/` as
  "gitignored build output". There was no `.gitignore` in the repository at
  all, and 13 `.o`/`.so` files had been tracked since the initial import
  (`02fe0b2`). Added `.gitignore`; `git rm --cached` for all 13. Tracked file
  count 67 → 54.

### Mathematical — drivetrain directionality was untested, and one method was wrong

The suite gated **magnitudes on single axes** and never directions:
`mecanum` asserted `dx >= 0.30` after a strafe, `teleop`/`ftc_integration`
asserted `sqrt(x²+z²) >= 0.5` after a forward command. A mixer with `f` and `s`
transposed, a flipped rotate sign, or a drivetrain that answered `rotate -1`
harder than `rotate +1` would all have passed. Writing a 3-DOF probe to answer
an unrelated question exposed this, and the first conclusion drawn from it was
**wrong**:

- **A rotate sign "error" that was not one.** The probe reported, over a 3.0 s
  end-to-end average, `rotate +1 → +0.029 rad` and `rotate -1 → +0.221 rad` —
  a 7.6× asymmetry that reads exactly like an inverted rotate. Measured on a
  **steady-state window** instead (ticks 180–240, after 180 ticks of drive),
  the shipped mixer gives **+2.289 rad/s for `rotate +1` and −2.030 rad/s for
  `rotate -1`** — correct sign, correct anti-symmetry, ~131°/s. The
  anti-diagonal mixer was A/B tested and gives −2.030 / +2.289: same
  magnitudes, **inverted**. Both pairings are zero-net-force with a pure
  torque on paper, which is precisely why algebra cannot choose between them:
  the sign is fixed by convention and the convention is fixed by measurement.
  **The shipped mixer is correct and was left alone.** The cause of the false
  signal is recorded in `drivetrain.c`: yaw *rate* builds over ~2 s from rest,
  so a short-horizon displacement average measures the spin-up ramp, not the
  drivetrain.
- **`mfs_t_drive_directions` added** to close the gap. It pins axis dominance
  (forward→+Z, strafe→+X, rotate→yaw), cross-talk ceilings, and — the gate
  that would have caught the sign question — that `rotate` and its reverse
  produce **opposite yaw signs**. Suite 12 → 13.

### Mathematical — yaw authority is battery-state dominated, by more than 4×

The same `rotate +1` command measures **+2.29 rad/s** from a command-free
settle on a fresh pack and **+0.46 rad/s** after 330 continuous ticks of
driving on that same fresh pack. This is not model instability, it is the pack:

- A full-power pivot puts all four motors near stall: **4 × 9.2 A = 36.8 A**
  against a **20 A** pack PTC.
- The PTC integrates `heat += (36.8−20)·dt/20`, i.e. **+0.84/s**, so it trips
  in **≈1.2 s**.
- `battery_get_voltage()` then returns the **1.2 V brownout** and the drive
  nearly stops for the rest of the run.

This is faithful FTC behaviour (real robots brown out in exactly this
situation) but it means any yaw assertion must be **structural, not absolute**.
`mfs_t_drive_directions` is written accordingly: it gates which axis dominates
and whether the two directions are opposite, both of which are battery
independent, and deliberately does *not* gate absolute yaw rate. Measured
straight-line cross-talk is battery dependent too — a sustained strafe leaks
**28%** of its lateral rate into forward and into yaw (2.6% on a fresher pack),
which is why that ceiling is 0.35 and not 0.25.

### Programming

- **`wheel_applied_torque` did not hold the applied torque.** `robot.c` stored
  it (the slew limiter's "what did we deliver last tick" memory) *before* the
  free-speed governor and the idle brake, both of which cut torque. During a
  full-power pivot the governor zeroes delivered torque while the memory kept
  the full pre-shaping value, so the instant a wheel came back under the bound
  the slew restored full torque **in one tick** — reintroducing the exact
  0→stall step the slew exists to prevent, through the back door. The store now
  happens after every shaping stage, immediately before the accumulator write.
  Cost, measured and deterministic (bit-identical over three `-O2` runs and
  under `-O1+ASan`): tank pivot heading **2.3003 → 1.8741 rad (−18.5%)**,
  translation 0.0603 → 0.0530 m; mecanum yaw authority −17%. The
  `tank` regression baseline was re-measured with the causal chain recorded.
  A limiter is a rate limit on *delivered* torque, so its memory must be
  delivered torque; the alternative is a variable that does not mean its name.
- **Tick-start friction selection grips the pivot harder (2026-10-04).**
  Engine-side correction with an understood MFS consequence: stick/slip μ is
  now selected once per tick from pre-force slip instead of re-decided on
  solver transient, so rolling wheels keep the full μ_s cone through torque
  transients (engine breakaway 0.999–1.005 of μ_s·N). The pivot drives
  slightly harder: tank translation **0.0530 → 0.0642 m (+21%)**, heading
  1.8741 → 1.9179 (+2.3%, inside its 8% band); mecanum strafe dx=2.2872
  byte-identical (anisotropic path untouched). Baseline re-measured with the
  chain in `mfs_suite_a.c`; fixed point verified bit-identical over three
  `-O2` runs and under `-O1+ASan/UBSan. Structural gates unaffected.
- **Partial-spawn unwind left stale body indices.** The `fail:` path in
  `ftc_robot_create_with_drive` re-poisoned `wheel_joints` and `roller_joints`
  but left `wheel_bodies` holding whatever partial creation had written. After
  the pool rewind those indices point at whatever *else* owns those slots, or
  out of range if the pool shrank — a debug print or retry loop inspecting the
  half-built robot would read another robot's wheel as its own. Body indices,
  roller bodies and roller counts are now poisoned alongside the joint indices.
- **`MFS_ROBOT_MAX_CARRIED_BALLS` was never enforced.** It has been declared
  in `mfs_module_1.h` since it was written and described in the file's own
  DESPOT-FIX note as "the gameplay CARRY limit", and no code path read it: the
  intake drew in every ball it touched, up to the 16-ball storage bound. A
  limit that is written down and not enforced is worse than none, because a
  reader sizes gameplay from it. The intake now counts balls actually held
  against the throat — using the same `pickup_radius` the pickup test uses, so
  the two cannot disagree — and holds station at 3.
- **`MFS_ROBOT_MAX_BALLS` was decorative.** `attach` hardcoded
  `state->max_balls = 16` while the macro sat unreferenced: two numbers for one
  quantity, with the macro free to drift without changing behaviour. Both the
  array bound and the spawner now come from the constant, clamped to the array
  size.

### Sanitizer profile

`MFS_TEST_CFLAGS="-O1 -fno-omit-frame-pointer -fsanitize=address,undefined"`
`MFS_ASAN_HOTLOAD_ODR_SUPPRESS=1 ./build_tests.sh` → **13/13**, zero AddressSanitizer
errors, zero UBSan runtime errors, zero leaks (only the intentional hotload
ODR heuristic suppressed).

## [DESPOT-2026-10-02] External-truth validation: what held and what did not

Full table, constants and authorities in `docs/VALIDATION.md`. 76 automated
comparisons against references computed independently (closed-form textbook
algebra, or from-scratch RK4), **76 passed / 0 failed**, plus a permanent
`external_truth` gate. Constants were fetched, not remembered:
`g_n = 9.80665 m/s²` (CGPM 1901 / CODATA 2022, exact), `atm = 101325 Pa`,
`ρ = 1.225 kg/m³` (ISA sea level), `C_d = 0.47`.

**Validated exactly (0.00% error, five significant figures):**
- Rotational dynamics `α = τ/I`; cylinder inertia `½mr²`; sphere inertia `⅖mr²`.
- **The engine's air drag is exactly linear viscous with `c = −ln(drag)` per
  second.** Integrating `dv/dt = −g − cv` with `c = 0.0100503 /s` predicts a
  2 s free-fall velocity of −19.4241 m/s; the engine produces −19.4241 m/s,
  and the closed form agrees with an independent RK4 to 1e-6.
- DC motor: stall endpoint equals the published spec; free speed equals
  `V/(K_e·gear)` and scales by exactly 12.8/12.0; the whole V–ω line matches
  `I = (V − K_eω)/R` to ≤0.15%.
- Battery OCV curve, `V = OCV − I·R_int`, PTC brownout voltage, and the
  1 h / 3 A capacity drain — all 0.00%.

**Valid to expected discretisation error:** free fall 0.16%, Coulomb sliding
distance 5.0%, energy conservation 0.31% over a 4.9 m drop, elastic-collision
velocities and momentum 0.67%, rolling no-slip residual 0.17%, restitution
5–6% typical (15.7% worst), PTC `I²t` trip time 0.80%.

### [DRAG-122x] The engine's drag cannot represent real air (OPEN, documented)

`world.drag = 0.99` is `c = 0.0100503 /s`, a **terminal speed of 976 m/s**.
Quadratic aerodynamics for the same 42 mm / 2.6 g ball (`C_d = 0.47`,
`ρ = 1.225`) gives **8.00 m/s**. The engine's own drag is **122×** too weak.

Not a bug — the config schema says so outright ("VISCOUS retention base
(truth: linear viscous c=−ln(drag), NOT quadratic aero)"). Recorded because
the consequence was nowhere written down: **the BioBuzz ball's flight time
and range are set almost entirely by the quadratic term `mfs_module_1` adds on
top**, not by the engine. Tuning ball flight from the engine alone will
correctly conclude drag is negligible, and reach the wrong answer.

### [BOUNDARY-E0] The y=0 boundary box is perfectly plastic (OPEN, documented)

`boundary_apply_box_cfg(..., {-250,0,-250}, {250,500,250}, ...)` is applied to
every non-static body **regardless of `static_plane_enabled`**. With the
material plane disabled this leaves a frictionless, perfectly-plastic
(e = 0) backstop at y = 0.

Measured: with no material floor, a ball dropped from 5 m never rebounds
(`e_eff = 0`) at any radius from 0.02 m to 0.64 m. With a material slab
present, restitution is correct at every one of those radii (effective
0.216 / 0.430 / 0.639 / 0.843 for set 0.20 / 0.40 / 0.60 / 0.80, a
consistent +6% discrete-impulse bias).

**Operational consequence:** a test that forgets a material floor is not
measuring `e = 0` — it is measuring the boundary, and will report a bounce
failure that has nothing to do with the restitution it meant to exercise.
The `external_truth` gate places its slab with the top at exactly `y = +1.0`,
clear of the boundary, and says why.

### [GRAVITY-BIAS] Engine gravity is +0.0342% off `g_n` (OPEN, now measured)

`world.gravity` defaults to `-9.81`; the standard value is `9.80665`, exact.
The bias is small but systematic and enters every normal load, so it scales
every friction budget and rolling-resistance term. Now *reported* by the
`external_truth` gate rather than left implicit in a default. Correcting the
engine default is a 475-tree change and is out of MFS scope.

### Retracted during this audit — recorded because it nearly shipped

- **"Restitution is 51% wrong."** Measured by comparing a rebound APEX against
  `e²(h−r)+r` on a slab whose thickness was 2 m instead of 1 m, so the
  reference geometry was wrong and the ball never reached the intended
  surface (`ymin = 1.54` against a nominal `1.05`). Measured in the
  **defining velocity form** `v_out = e·v_in`, restitution is correct.
  Restitution must be gated in velocity, not apex: the apex form mixes in the
  bounce count, the trigger instant and drag.
- **"The rotate mixer signs are inverted."** Already retracted earlier the
  same day; see the `DESPOT-2026-10-02` section above.

### Declared coverage gaps

- Cylinder inertia about a **transverse** axis (`m(3r²+L²)/12`) is exercised
  by no test; only the symmetry axis and the sphere are gated.
- **Angular** momentum in oblique and multi-body contacts is unvalidated
  (linear momentum is, to 0.67%).
- The **analytic mecanum lateral force** is gated for behaviour (axis
  dominance, anti-symmetry) but its magnitude has no independent
  roller-contact reference, because it is an admitted continuum
  approximation with no closed form.

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
blocking case (MFS is 9/9, was 8/8). Two phases, and the split is the finding:

* **Phase 1, open loop** (motor alone, shaft fed exactly 0): `output_torque =
  3.6509 N.m` against the published `3.7265 N.m`, **-2.03%**. Current 9.013 A
  against the 9.2 A spec, back-EMF exactly 0. **The electrical model is
  honest.**
* **Phase 2, closed loop** (disturbance observer feeding its estimate back
  into `motor_update_load`): `output_torque = 3.1279 N.m`, **-16.06%**, with
  `load_torque = -3.1283`. Gated at 25%, so it passes.

**So the stall endpoint is now covered and is acceptable.** Engaging the
observer softens the locked-rotor torque by 16% — real, and far better than
the "~6x low" soft fixed point the `motor.h` header still warns about, which
is now out of date. It is *not* the 41%-low (2.21 vs 3.73 N.m) regression that
got the delivered-torque change reverted; that regression only appears once
delivered-torque accounting is applied, and there is now a test that will
catch it if that change is ever landed.

**Two corrections to earlier entries in this file, both from my own testing
errors, recorded because both nearly became false findings:**

1. An intermediate version of this test reported the closed loop at
   `0.7081 N.m` (**-81%**), and I wrote that up as a broken
   observer->implicit-solve coupling. **That was wrong: the bug was in my
   test.** `motor.c:158` gates the load term on `m->wprev_valid`, but
   `motor.c` NEVER SETS IT — only `robot.c:797` does. My isolated test never
   performed the handshake, so `tau_L` was silently 0 and I had measured
   "observer disabled", not "observer mis-coupled". The `-81%` figure, and
   any conclusion drawn from it, are retracted.
2. The earlier `ftc_hotload` "false green" was also my own error (wrong
   working directory), as recorded in the process note below.

**The one real finding that survived: the observer's on/off state was owned by
the caller.** `motor_update_load` read `m->wprev_valid` but never set it, so
any consumer other than `ftc/submodules/robot.c` — the plugin path, a future
submodule, the standalone build — got a silently dead disturbance observer
(`tau_L == 0`) with no warning and no way to detect it from the motor's own
state. **Fixed 2026-09-29:** the estimator moved into `motor_observe()` in
`motor.c`, so the module that READS the flag now also SETS it, and callers
cannot get it wrong. `robot.c` no longer touches `load_torque`/`w_prev`/
`wprev_valid` at all.

Two things about that refactor worth recording honestly:

* It was **not** behaviour-preserving on the first attempt. My first
  `motor_observe` cleared `wprev_valid` in the "no previous sample" branch,
  which meant the flag never became valid and the observer stayed dead for the
  whole run — **5 of 9 suite cases failed**. The pre-refactor code set the flag
  unconditionally, so the second tick could form a difference. Caught only
  because the suite is strong enough that a dead observer breaks half of it,
  which is a decent argument for having built it.
* Metrics after the fix: strafe (0.8739 / 1.1202), odometry error (9.0%), the
  stall endpoint (3.6509 open / 3.1279 closed) are all **byte-identical** to
  before the refactor, so it is a pure encapsulation. The one number that
  moved is the tank turn: heading 2.4167 -> **2.3003**, displacement 0.0774 ->
  0.0603.

* **CORRECTION (2026-09-29).** When first recording the above I wrote that
  "2.3003 happens to be the target the tank test documents". **That was false
  and I fabricated it.** The tank test gates only `heading >= 0.1` and
  `disp <= 0.3`; there is no published 2.3003 anywhere. The only occurrences of
  that number in the tree were in my own text. Do not cite it as a target.

  The real, established cause (measured by toggling the blocked-rotor gate in
  `motor_observe` with everything else fixed): **the gate is what moves the tank
  turn** — gate on gives 2.3003 / 0.0603, gate off gives 2.4167 / 0.0774. That
  is physically sensible: a turn-in-place starts with the wheels nearly
  stationary, so the gate's "saturated AND not turning" condition is briefly
  true, the observer reports full stall, and the initial drive is trimmed —
  producing a 22% tighter (less translational) turn.

  **Which value is correct is unknown.** There is no spec for tank-turn rate or
  for translation during a pivot, so neither figure can be called an
  improvement. The heading gate is far too loose (0.1 rad, against 2.3 rad
  achieved) to have caught a 5% heading change either way. The honest
  statement is: the gate measurably changes a manoeuvre, nothing currently
  knows whether it changes it correctly, and the tank test is not tight enough
  to notice.

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
| tank turn heading (NO published target; test gates only >= 0.1) | 2.4167 | 2.4167 |
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

### MFS-H5-2026-09-29 — Intake roller had two actuators and could not be stopped — FIXED

**Symptom.** The intake could not be switched off. The 600–1200 RPM speed
slider had no effect, and the momentary reverse (B while held) did nothing.

**Root cause.** Two actuators on one joint, neither wired to the intake state:

1. A revolute joint motor, enabled once in `mfs_module_1_intake_create()` with
   `intake_speed_rpm`, and **never touched again** — permanently on at the
   creation-time speed.
2. A P-control in `mfs_module_1_intake_step()` applying torque straight to the
   roller body. This one *did* correctly target 0 when `intake_active` was
   false — and was simply overruled by the joint motor still driving at full
   speed.

So the P-control was not wrong, it was outvoted. The speed slider never
reached the motor because the motor's target was frozen at creation, and
`intake_power` was written in two places and read by **nothing at all** —
verifiably dead code.

**Fix.** One actuator: the joint motor, re-driven from state every tick
(0 when inactive, `intake_speed_rpm` when active, negated when
`intake_power < 0`). The P-control is removed rather than kept alongside it —
two controllers on one joint was the defect, not redundancy, and the joint
motor is the correct one because it is solved inside the constraint system
(so it cannot fight the joint) and honours `motor_max_torque`. The motor is
kept `enabled` at target 0 on purpose: that is what actually brakes the roller
to rest rather than leaving it coasting.

**Verification — the test whose absence let this survive.** `mfs_t_intake_stop`
is registered and **gated** (MFS is 10/10, was 9/9). Measured, same binary,
only the fix toggled:

| phase | pre-fix | post-fix |
|---|---|---|
| intake ON | 62.848 rad/s | 62.848 rad/s |
| intake OFF | **62.532 rad/s** (does not stop) | **0.004 rad/s** |
| momentary reverse | **+61.371 rad/s** (never reverses) | **−59.495 rad/s** |

Both assertions fail on the pre-fix code and pass on the fixed code, so this
is a real regression guard and not a test written to agree with whatever the
code happened to do. Full profile 206/206, ASan/UBSan green.

### MFS-H6-2026-09-29 — Flywheel spun about an axis perpendicular to its own symmetry axis — FIXED

**Symptom.** The 35° shooter launch was unreachable by construction.

**Root cause — three spin axes, none of them agreeing.**

1. The engine defines a cylinder's symmetry axis as `cached_axes[0]`, i.e. the
   body's **local X**. The flywheel was created unrotated, so its symmetry axis
   started along the chassis X.
2. The "angle it up by 35 degrees" step rotated the body about **X** — the very
   axis that rotation leaves invariant. So the symmetry axis never moved at
   all. That step was a no-op on the quantity it was supposed to change.
3. The revolute joint was built about chassis `(0,1,0)`, and the spin-up torque
   is applied about `(0, cos35, sin35)`.

So a **thin disc was being driven to spin about an axis lying entirely in its
own plane**. That is a tumble, not a flywheel: the rim speed at the contact
patch is not the tangential speed a flywheel imparts, and the launch angle is
whatever the geometry says — measured, 90° (a flat horizontal disc), not 35°.

**Fix.** Make all three the same axis. Rim velocity is perpendicular to the
symmetry axis, so for a 35° launch the symmetry axis must be
`(0, cos35, sin35)` in chassis coordinates. The disc is oriented by a 90°
rotation about `(0, −sin35, cos35)`, which maps its local X onto that axis,
and the joint is built on the same axis.

**Verification — `mfs_t_shooter_axis`, registered and gated** (MFS 11/11).
It asserts the axes *agree* rather than asserting a spin rate, because the
geometric defect is the thing that is wrong and a rate symptom depends on how
long you wait. Same binary, only the fix toggled:

| measurement | pre-fix | post-fix |
|---|---|---|
| \|disc · joint\| | **0.0000** (perpendicular) | **1.0000** |
| \|disc · torque\| | **0.0000** | **1.0000** |
| launch angle above horizontal | **90.00°** | **35.00°** |

All three assertions fail pre-fix and pass post-fix. Full profile 206/206,
suite 34/34, ASan/UBSan green.

### MFS-H7-2026-09-29 — Fired balls carried no spin, so the Magnus model was unreachable — FIXED

**Symptom.** Balls left the shooter with zero spin. The Magnus lift in
`mfs_module_1_ball_physics_step()` was therefore dead code: it is gated on
`spin_rate > 10.0` rad/s, and nothing in the game could ever put a ball above
that. The only way a ball could spin at all was being clipped off-centre by the
contact solver, which is not a shooter.

**Root cause.** The launch block transferred **linear velocity only**. It
computed `surface_vel = omega_flywheel x r`, used its *direction* for the
launch force and its *magnitude* for the speed, and never wrote
`ball->angular_velocity` at all. The transfer model was half-implemented: the
surface velocity of a contact point implies a spin on the driven body, and that
consequence was dropped.

**Fix.** A ball driven by a flywheel is spun by contact friction until its own
contact point matches the flywheel surface. Using the same `omega x r = v`
convention the flywheel itself uses:

    omega_ball = v_surface / r_ball

so the smaller ball spins *faster* than the surface speed alone implies — which
is the point of a flywheel. A 50 mm flywheel at 4000 rpm (20.9 m/s surface)
hands a 24 mm ball roughly 870 rad/s. Applied with the same 80% transfer as
the linear term so the two stay self-consistent.

**Honest limitation.** This is a contact-model approximation, not a friction
solve. It does not resolve slip, so it cannot express "this ball skids instead
of rolling", and it will over-spin a ball that is barely touching. It is
strictly an improvement on the prior state, where the spin channel was never
written at all.

**Verification — `mfs_t_ball_spin`, registered and gated** (MFS 12/12):

| measurement | pre-fix | post-fix |
|---|---|---|
| fired ball \|v\| | 14.967 m/s | 14.967 m/s |
| fired ball \|omega\| | **0.0 rad/s** | **742.1 rad/s** |

The spin was *exactly* zero before, which is the signature of an unwritten
channel rather than a weak one. The flywheel in the same run reached 3957 rpm
against a 4000 rpm target, so the launch had real surface speed to transfer and
the fix is genuinely using it. Full profile 206/206, suite 34/34,
ASan/UBSan green.

### MOTOR-II-2026-09-29 — Delivered-torque accounting: real gain, but coupled to the odometry model — NOT LANDED

Re-attempted 2026-09-29 now that the gated stall test exists. **Result: it is
worth doing, and it cannot be done alone.** Not landed; this records the
measurement so the next attempt does not have to rediscover it.

**The change.** The observer compared `I_axle * alpha` against the motor's
*commanded* torque, but `tau_exp_prev` includes whatever went into accelerating
the rotor. That torque never reaches the wheel, so the difference was being
misread as external load. The fix reflects the rotor into the inertia the
estimator sees:

    I_total = I_axle + J_rotor * gear_ratio^2

**Measured, same binary, only this changed:**

| metric | baseline | with reflected rotor |
|---|---|---|
| stall open loop | 3.6509 N.m (-2.03%) | 3.6509 N.m (-2.03%) — **unchanged** |
| stall closed loop | 3.1279 N.m (-16.06%) | 3.1279 N.m (-16.06%) — **unchanged** |
| strafe, physics | 0.8739 m | **0.9727 m** (+11%) |
| strafe, odometry | 1.1202 m | **2.1371 m** (+91%) |
| odometry distance error | 9.0% | 8.8% |
| suite | 12/12 | **11/12** |

**Two things worth recording.**

1. *The stall gate did its job.* The reason this change was reverted in the
   first place was that it dragged the stall endpoint from 3.73 N.m to 2.21
   N.m with no test to catch it. This time the endpoint was **byte-identical**
   in both phases. The reflected-inertia correction is orthogonal to the stall
   fixed point — at a locked rotor `alpha == 0`, so the extra inertia term
   vanishes. The original regression must have come from a different (and now
   superseded) form of the change, not from this one.

2. *Physics improved, odometry broke.* Real chassis strafe transmit went up
   11%, which is the point. But the encoder-based odometry went up 91%, so it
   now over-reports by 2.2x against what the chassis actually does, and the
   odometry test correctly goes red. The wheels are slipping more (more torque
   available, same traction limit), and the odometry model has no slip term, so
   it counts rotation the chassis never converts into travel.

**Why not landed anyway.** A red test is a truthful signal that a change is
incomplete, and a half-landed change here would make the drivetrain *report*
worse than it *behaves* — the odometry is what a driver trusts. The correct
landing unit is delivered-torque **plus** a slip term in the odometry model,
then the lateral `VREF` retune on top. Doing it in the other order is what
produced the original revert.

**Open question for whoever picks this up.** `J_rotor = 3.0e-6 kg.m^2` at the
motor shaft is a representative FTC-class brushless figure and a MODELLING
CHOICE, not a spec value — no preset carries rotor-inertia data. The shape of
the correction is right; the magnitude is order-of-magnitude only. Sizing it
per-SKU would need vendor data that is not in the tree.

### CONFIG-2026-09-29 — THE BIG ONE: the MPE test suite never loaded the config

**`g_cfg` is a plain global, so the C runtime zero-initialises it. Nothing in
`tests/mpe_suite_main.c` ever called `mpe_config_init()`.** The whole MPE suite
had been validating physics against an all-zero configuration:

| field | suite was using | real default |
|---|---|---|
| `solver_iterations` | **0** | 64 |
| `solver.bias_factor` | **0** | 0.1 |
| `solver.penetration_slop` | **0** | 0.01 |
| `body_defaults.*_restitution` | **0** | 0.5 |
| `body_defaults.*_fric_s` | **0** | 0.3 |
| `world.gravity` | 0 | −9.81 |
| `timestep.max_substeps` | 0 | 5 |

So the sequential-impulse solver ran **zero iterations**, with **no Baumgarte
positional bias** and **no penetration slop**, on bodies with no restitution and
no grip. Every physics result this suite produced — including the revolute
matrix, the manifold-reduction change, the torque-free angular-momentum work,
the cylinder/sphere inside normal — was measured on a solver that was not
solving.

**Why nobody spotted it.** The shipped game is fine: `root_gtk.c` calls
`mpe_config_init()` (twice), as do `headless_main.c` and `tui/tui_main.c`. The
MFS harness calls it in `mfs_test_world()`. Some individual MPE test *files*
call it too. `mpe_suite_main.c` was the single path that did not, so the game
looked configured and the suite looked tested.

The clue was already in the tree, written by this same session's drag fix, in
`core/physics_world.c`:

> "...a perfectly reachable state, since a world initialised **before
> `mpe_config_init()` runs sees a zeroed g_cfg**..."

That comment was describing *this suite*, permanently. Nobody connected the two.

**It also resolves DEEP-2026-09-29.** The "unexplained 0.2–0.3 m residual" in
enclosed-sphere ejection was measured under the zeroed config. With the real
config loaded, all four sub-cases eject correctly and the residual disappears
(case 1 clears by 0.50 m, case 2 by 0.17 m, case 3 lands at 0.0100 m — which is
*exactly* the real `penetration_slop`, i.e. resolved). The mechanism I could not
establish did not need establishing: the solver simply had bias 0 and slop 0, so
it had no positional recovery to give.

**A second bug was hiding behind the first.** The residual assertion used
`pen = RS - min(axial_clear, radial_clear)`, which is only meaningful while the
sphere *centre* is inside the cylinder. Once ejected clear it reports a large
positive "overlap" for a sphere sitting well outside — so a fully-resolved
ejection (measured radial 2.44 against a radius-2.0 cylinder) was being
reported as a 0.94 m overlap. Replaced with the real signed distance to a solid
cylinder, which is negative inside, zero on the surface and positive outside,
so ejection and approach use one expression.

**Impact: 34/34 before and after, and that is the point.** The suite was green
either way, so no gate caught this. It was found by going looking for the class
of defect that had dominated this whole session — *a value that is set but never
wired to the thing that consumes it* — applied to the config instead of to a
joint motor or an intake flag.

**Lesson worth keeping.** `mpe_test_begin()` saves and restores `g_cfg` but
never *initialises* it. Any harness that saves-and-restores state should also
establish the state, or it will faithfully preserve whatever garbage it
inherited. MFS got this right by accident (`mfs_test_world()` calls
`mpe_config_init()`); the MPE harness never did.

### DEEP-2026-09-29 — Enclosed-sphere ejection: what is actually true, and what is still open

The long-standing "1 of 4 sub-cases unresolved" note was a **test fixture bug**
and has been retracted: the test spawned the cylinder at y=0 with half-length 1,
i.e. 1 m *inside* the world's y>=0 boundary, so the boundary position clamp
fought the contact normal every tick. With the cylinder clear of the floor, all
four sub-cases eject the sphere in the right direction. Two related test bugs
were fixed alongside it: the penetration formula was radial-only (wrong for a
sphere near a cap, where the minimum clearance is axial and the radial term is
~0, so it reported 2.0 m for a 0.7 m overlap), and the engine's own `pen` is
`min_clear + r_s` (conservative, not geometric overlap).

**What was measured, and what remains open.**

An isolated harness — cylinder clear of the boundary, no floor, gravity 0 —
resolves **all four sub-cases fully**, and the result is **identical at 8, 16,
32, 64 and 128 solver iterations**. So this is emphatically *not* an
iteration-count limitation, and the depenetration route itself is sound.

The canonical suite world still shows a **0.2–0.3 m residual** after the same
120 steps. So the residual is real for *that* world and the two differ. An
attempt to close the gap by pinning the config before world creation made things
strictly worse (8 checks failed instead of 0): `mpe_config_init()` resets
suite-wide defaults the test was implicitly relying on, so that hypothesis was
wrong and was reverted rather than left half-applied.

**RESOLVED — see CONFIG-2026-09-29.** The "unexplained residual" was this
suite running with `bias_factor = 0` and `penetration_slop = 0`: a solver with no
positional recovery at all. It is not a depenetration limitation and never was.
All four sub-cases now gate on full resolution against the engine's real
`penetration_slop`.

### META-ROTATION-2026-09-29 — EXONERATED 2026-10-01: the fixture was the bug, not sphere-sphere contact

Found by a new metamorphic test, MISDIAGNOSED twice, closed on the third probe.

**The test.** Rotate an entire initial condition by R, simulate, rotate the
result back by R; you must get the unrotated run's answer. This holds for any
correct engine regardless of what it computes, so it has no golden number to
be wrong in the same way as the code.

**Bisected** with a standalone probe, same build flags:

| probe | max abs position divergence |
|---|---|
| free flight, 1 sphere, no floor | **0.000000e+00** — exact |
| drop onto floor, 1 sphere | **1.49e-08 m** — float noise |
| head-on **sphere-sphere** pair, no floor | **1.195e+00 m** — BROKEN |

Integration is exactly equivariant and the floor contact path is equivariant to
float precision. The defect is specific to **sphere-sphere contact**.

**Mechanism, instrumented per tick.** The spheres do collide and do bounce apart
in both worlds (separation 0.563 m → 1.0 m → 3.2 m in the unrotated run, which
is correct). But `has_contact` reads **0** in the unrotated world and stays
**2** in the rotated world all the way to tick 150 — when the spheres are
**2.59 m apart** and cannot possibly be touching. The rotated run is still being
fed a contact, so impulses keep being applied to a separated pair. That is the
whole of the 0.98 m/s vs 0.82 m/s separation-rate difference measured.

**What to chase:** nothing — see 2026-10-01 verdict below. The tight 1e-4 m
gate stays as a dormant tripwire.

**Verdict 2026-10-01: ENGINE EXONERATED.** Per-tick reprobe killed the stale
theory: `has_contact` resets every tick in both step paths, and positions
agree to ~1e-7 THROUGH the bounce (ticks 0–9). Divergence starts at tick 10
as a positional-only shove (mm/tick, no velocity change, no contact flag) in
the rotated world only — the depenetration floor pass and CCD floor sweep
acting on a B-frame trajectory that crosses y=0, i.e. a phantom floor the
solver floor never ordered. But that is honest backstop design (boundary box
min-y=0 exists in both step paths), and it exposes the real error: the probe
demanded equivariance under an ARBITRARY rotation while the environment (y=0
backstop + box) is only yaw-symmetric. R^-1 Phi(R x) = Phi(x) is untestable
outside the environment's symmetry group. Fixture fixed (floorless, yaw-only
0.9 rad about +Y): **5.44e-07 m / 1.45e-07 m/s over 150 ticks with a real
bounce. PASS.** The XFAIL text is kept dormant; if it ever fires, suspect the
environment first, the dynamics second.

Reported as a loud `[XFAIL][META-ROTATION]`; the suite surfaces it on every run.

**Two false alarms I caused first, recorded so nobody repeats them:**

1. I assumed `vector4` was `{x,y,z,w}`. It is declared **scalar-first**:
   `typedef struct { float w, x, y, z; } vector4;` (math3d.h). So a
   positional conjugate initializer builds components rotated by one slot, not
   the conjugate of anything, and produced a 4 METRE "equivariance failure".
   The engine was exact — a 90° rotation about Z maps X→Y and Y→−X to full
   float precision. The test now uses named fields.
2. I rotated *every* body including the floor box, so the rotated run had a
   **tilted floor** — a different problem, not a test of equivariance. The
   rotation helper now takes the first dynamic index and rotates only that
   range; a mass-0 body is not necessarily flagged `static_state`, so filtering
   on that flag was also wrong.

Both were my test's fault. I checked the engine's quaternion math directly
before believing either, which is the only reason they did not become false
findings about the engine.

### SLEEP-H1-2026-09-29 — EXONERATED 2026-10-01: normal pre-sleep rest, wrong test expectation

Found while building the regime matrix, and deliberately left unresolved rather
than shipped as a test.

A ball launched at 3 m/s with sleep **enabled**, under the `heavy` regime
(gravity ×3), reported after 0.5 s: `|v| = 0.0000 m/s`, position back at
exactly its spawn height, distance travelled `0.0000 m`, and
`is_sleeping = FALSE`. So it was frozen without being asleep. The same test
passes in `default`, `light`, `brittle` and `sticky`.

This is a *different signal* from a sleep-threshold problem and it is not
explained. A related test (`mpe_t_meta_sleep`) was written, failed to converge
on its second arm (a cube dropped on the static plane still read 0.817 m/s after
10 simulated seconds, so "settles" was not a property the engine exhibited), and
was **withdrawn** rather than shipped red or green for a reason nobody could
state. `meta_sleep` is deliberately absent from the registry; the reasoning is
preserved as a comment in `tests/mpe_suite_d.c`.

For whoever picks this up: the first question is whether a non-sleeping body can
be frozen at all, or whether the regime is corrupting velocity some other way.

**Verdict 2026-10-01: ENGINE EXONERATED — no frozen body exists.** Reproduced
the exact signature under heavy (vertical 3 m/s pop from rest on a slab:
|v| = 0.0000 at spawn height, 0.0000 travelled, is_sleeping FALSE at 0.5 s)
with per-tick instruments: the ball settles at ~0.33 s, the sleep timer reads
0.20 against a 0.5 s duration at the 0.5 s sample, and it falls lawfully
asleep at ~0.83 s. The engine was mid-countdown, not stuck. The withdrawn
expectation (moving XOR asleep) missed the legitimate third state — settled
with timer pending — which is correct behavior, not a freeze. Locked with the
committed `sleep_settle` gate (moves → settles → sleeps-iff-enabled, all five
regimes); the direction-B companion (cube allegedly unsettled at 10 s) also
settles and sleeps normally today.

### MOUSELOOK-2026-09-29 — "Flick right/down locks, left/up does not" — FIXED (plumbing), and the convention was never wrong

User report after the relative-pointer fix landed: flicking the mouse or
trackpad **right or down** locked properly; **left and up** did not.

**The sign convention was correct.** Verified all four directions plus the four
diagonals: Wayland is +x right / +y DOWN, the camera consumes +x right / +y UP,
so x passes through and y is negated, and that is what the code does. The
asymmetry was not in the convention.

**What was actually wrong — three defects in `on_mouse_movements()`, all of
which lose motion in a directional way:**

1. **FALL-THROUGH (the likely primary cause).** When the relative pointer was
   live but no new relative event had been dispatched yet,
   `mouse_lock_take_relative_delta()` returned 0 and the handler fell straight
   into the ABSOLUTE path. With a live relative pointer the cursor is
   *unconstrained* — nothing stops it travelling — so `x - last_x` there is not
   "how far the hand moved", it is merely where the compositor left the arrow.
   That is direction-dependent in exactly the reported way: flicking toward an
   edge **pins** the cursor, which *stops* absolute events from contaminating
   the signal, so that direction works; flicking back **un-pins** it, absolute
   events resume, and they overwrite the relative signal. The working direction
   was working by accident.
   Fix: `mouse_lock_relative_active()` gates the absolute path off outright
   while a real relative lock is attached. Mixing the two sources is the entire
   hazard the relative pointer exists to remove.

2. **OVERWRITE INSTEAD OF ACCUMULATE.** Both paths did
   `mouse_delta_x = ...`. The camera consumes the delta once per frame, but GTK
   can deliver several motion events before then, so every event but the last
   was silently discarded — a fast flick lost most of its magnitude and a slow
   one lost all of it. Now `+=`.

3. `last_x/last_y = -1` after a relative event forced a re-anchor on the next
   absolute event — pointless once (1) disables that path, and one more way for
   the two sources to interleave.

**Testability, which is the real lesson.** The convention was inlined in a GTK
handler, so the one piece of mouse-look logic that can be wrong in a
*directional* way could only be checked with a live compositor and a physical
mouse — which is why this was reported by hand and not caught. It is now a pure
function, `mpe_mouse_relative_to_camera()`, in `ui_input/mouse_look.h`, with no
GTK and no engine dependency, asserted by the new `mpe_t_mouse_look_axes` from
the headless suite: all four directions, all four diagonals, magnitude
preservation, and the accumulation property from (2).

**Two of my own test bugs, recorded because the test caught both immediately:**

* The function's comment said it returned the magnitude while it returned
  `dx^2+dy^2`. The suite caught it on the first run. The contract now returns
  the true magnitude and says so.
* I asserted a flick of `(4,-2)` accumulated to `-10` in camera y, i.e. I had
  up/down backwards **in the test** while getting it right in the convention.
  Worth that it caught me: that confusion is the whole failure mode.

**Diagnostics added, for if it is still not right.** `mouse_lock_diagnostics()`
reports per-direction receive counts (`pos_x/neg_x/pos_y/neg_y`) and total
events, resettable per lock attempt. This separates the two possibilities that
look identical from the outside: *the compositor never delivered the event*
(counts asymmetric or zero) versus *we received it and went wrong* (counts
symmetric). That distinction is what this bug lacked and what made it
expensive.

### MOUSELOOK2-2026-09-29 — THE ACTUAL ROOT CAUSE: the GTK4 engine still hardcoded `GDK_BACKEND=x11`

Second user report after the first fix: **in a window nothing worked at all**
(mouse exits after a little movement); **fullscreen only partially worked**
(right/down fine, up/left not).

**Root cause, found in `root_gtk.c`:**

```c
g_setenv("GDK_BACKEND", "x11", TRUE);   /* <-- still there */
```

The GTK4 engine **never ran on native Wayland at all.** It went through
XWayland. The "force X11 because mouse lock is broken on Wayland" workaround
from the GTK3 era was carried across the port and never removed — and it
became the reason mouse lock could not be fixed, because **every Wayland-side
fix was dead code sitting behind it.** The relative-pointer protocol I
implemented, verified present on the compositor, was never being exercised.

This one line explains the entire reported behaviour precisely:

* **Windowed:** X11 re-centring only happens *on a motion event*, and motion
  events **stop once the cursor leaves the window**. So the warp never fires,
  the cursor leaves, and the camera freezes. Nothing is wrong with the warp; it
  cannot rescue a cursor that has already left. That is "the mouse exits after
  too much movement", verbatim.
* **Fullscreen:** the window covers the whole screen, so the cursor cannot leave
  and the warp keeps working. It degrades only at the screen edges, which is
  the residual up/left/right/down asymmetry.

**Fixes shipped together, because the first two were pointless without the
third:**

1. **Backend is no longer forced.** GDK chooses: native Wayland when
   `WAYLAND_DISPLAY` is set, X11 otherwise. Both paths are implemented, so
   neither is a fallback hack.
2. **`zwp_locked_pointer_v1` confinement added.** The relative pointer alone was
   never a lock — it supplies unbounded deltas but does **not** confine the
   cursor. That is exactly the windowed-mode failure: the cursor still leaves
   the window, and once outside, the compositor stops delivering. Locked-pointer
   is the confinement; relative-pointer is the input device. A first-person
   camera needs both.
3. **Globals bound once at startup** (`mouse_lock_init()`, called from
   `root_gtk.c`). The lazy bind needed a `wl_display_roundtrip()` from inside a
   GTK handler, which re-enters GDK's own event delivery — a classic
   works-first-click-then-behaves-differently hazard.

**Honest note on ordering.** I shipped the first Wayland fix, verified the
compositor advertises the protocol, and reported it as done. It could not have
worked, and neither could the second one, because a single `g_setenv` above
`gtk_init` made the entire Wayland path unreachable. I checked the compositor
and the protocol and never checked which backend the binary was actually
requesting. That is the same failure mode as the intake flag and the config:
**verify the thing is connected, not that the thing exists.**

`mouse_lock_confined()` and `mouse_lock_relative_active()` are reported
separately because they are different capabilities and can fail independently.

### SCENEORDER-2026-09-29 — THE GAME WAS RUNNING WITH ZERO-FRICTION, ZERO-RESTITUTION OBJECTS — FIXED

The user's framing was the key: *"if the physics math tests all show correct,
then something is wrong with configuration of objects and settings."* They were
right, and the decisive clue was not numerical at all.

**The observation that identified it:** physics was **correct inside the F10
validation region and wrong everywhere outside it**. A spatial boundary drawn
around a *creation-time* difference. Runtime-spawned content (F5, F8, F10, and
everything the player spawns) is created after the config exists; default-scene
content was created before it.

**Root cause — the config did not exist when the scene was built.**

`root_gtk.c`, first entry point:

```
line 39   when_realised()  ->  scene_init_default()   <- spawns the whole default scene
line 63   app_activate()   ->  mpe_config_init()      <- 24 lines too late
line 65                     ->  mpe_config_load("status/engine.cfg")
line 71                     ->  physics_world_init(primary)
```

`g_cfg` is a plain global in `mpe_config_schema.c`, so the C runtime
zero-initialises it. Body materials are stamped from it **at construction time**
and `physics_world_init()` does **not** retro-fit them onto existing bodies:

```c
/* core/rigidbody.c */
rigid_body->restitution     = g_cfg.body_defaults.cube_restitution;  /* 0.0, not 0.5 */
rigid_body->friction_static = g_cfg.body_defaults.cube_fric_s;      /* 0.0, not 0.4 */
```

Measured, same process, either side of `mpe_config_init()`:

| | cube restitution | cube friction |
|---|---|---|
| before init | **0.000** | **0.000** |
| after init | 0.500 | 0.400 |

So **every object in the default scene was permanently built with no friction
and no restitution.**

**That accounts for all three reports, and specifically:**

* *"objects rotating endlessly / rolling endlessly like a cartoon character"* —
  zero friction means no tangential traction AND no spin-down torque. Nothing
  decelerates a zero-friction body. This is the clearest of the three.
* *"bounce can go from no bounce to massive bounce in an instant"* — a world
  containing a mix of zero-material default objects and correctly-material
  runtime objects produces contacts whose behaviour depends on **which two
  bodies** happen to touch, so restitution swings between dead and violent.
* *"F5 cubes phasing through each other"* — F5 sets its own friction (0.8/0.7),
  but the stack rests on the static plane and friction combines with `min()`,
  so the effective stack friction was very low and the stack squirted.

**Fix — make the ordering unobservable rather than merely corrected:**

1. `mpe_config_ensure_ready()` (config/mpe_config.c): idempotent init, plus
   `mpe_config_is_ready()`.
2. Called at the **choke point every body passes through** —
   `rigidbody_initialisation_cube/cylinder/sphere`. Belt.
3. Called at the top of `scene_init_default()` — the function that was actually
   called out of order. Braces.
4. `root_gtk.c` startup now configures first, explicitly, so the log is honest.
5. `mpe_config_force_unready_for_test()`: a test hook to reproduce the
   precondition.

**Why the suite could never have caught this on its own, and what changed.**
Every test calls `mpe_test_begin()`, which initialises the config — so the
ordering was always correct in CI and only wrong in the game. The regression
test therefore reproduces the *precondition* rather than the happy path: it
forces the config unready, then builds a body and requires live materials. My
first attempt at that test was decorative (it passed with the fix removed,
because the test harness had already initialised the config); I stripped the
guards and confirmed it now fails:

```
WITHOUT the fix:  cube friction_static=0.000 friction_kinetic=0.000 restitution=0.000
                  [FAIL] cube.friction_static > 0.0f
                  [FAIL] cube.friction_kinetic > 0.0f
                  [FAIL] mpe_config_is_ready()
                  [FAIL] cyl.friction_static > 0.0f
WITH the fix:     friction_static=0.400 friction_kinetic=0.300 restitution=0.500
```

That is the same failure mode as the dead `intake_power` flag, the frozen joint
motor, the never-initialised test config, and the forced `GDK_BACKEND=x11`:
**something that exists, is correct in isolation, and is never actually wired
to the thing that consumes it.** Five instances now. The pattern worth taking
away is not about this codebase -- it is that "it exists" and "it is connected"
are different claims, and only one of them was ever being checked.

Verification: 39/39 green in the suite, and in all five regimes; engine builds
with zero warnings. The three visual symptoms are the user's to confirm.
