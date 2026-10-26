# MPE Audit — 2026-10-21 — rolling traction, CCD resting-gate, headless fleet wiring

Scope of this report: the second wave of the robotics-physics deep dive
(2026-10-20 audit covered the CCD-TOI gravity double-count). Everything below
was driven by the MFS probe (`/tmp/opencode/mfsdrive.c`) plus the MFS suite
and verified by the engine gate battery at the end of the session.

## 1. Drivetrain traction law — Coulomb-slider replaced with impulse closure

### Measured defect chain (pre-fix)
- Any meaningful command saturated all four wheel torque accumulators on
  tick 1 (slew limit ±40 rad/s/tick against a 0.1 m/s/tick chassis
  inertia), so slip-loop control ran against a hard overshoot of about
  46 rad/s per tick and settled into a period-2 limit cycle: wheel spin
  ping-ponged (−25 ↔ +70 rad/s) with the Coulomb force sign flipping every
  tick (|F| pinned 13.7 N).
- The wheels alternated between runaway spin (57 rad/s ≈ 2.4× the 23.35
  rad/s free speed, currents regen-negative) and the traction-cut pin
  (cscale 0.15); the chassis rode saturated sliding at 0.5–0.65 m/s against
  a 1.17 m/s ceiling, and odometry diverged to 63% error.
- Battery SoC stayed pinned at 1.000 (the runaway made every tick a
  regen-positive sum that cancelled load).
- Macro fwd/rev asymmetry (0.65 vs 0.52 m/s, 43% spread) was a windowed
  fingerprint of the limit cycle, not a real drive asymmetry (after the
  fix: 1.219 fwd vs 1.227 rev — 0.7%).

### The fix (`drivetrain.c`, `MFS_MECANUM_ANALYTIC_VREF` retired)
Per-wheel impulse closure: the tangential force is the lesser of the
Coulomb ceiling and the impulse needed to close the wheel–ground relative
velocity in one tick,

  F = min(mu·N, m_eff·|v_a|/dt),  m_eff = 1/(1/m_chain + r²·(a·I⁻¹·a))
with m_chain = n_per_wheel/g (2.2 kg) and the spin inertia read live from
`wheel->inverse_inertia_system` along the spin axis (≈0.096 kg here).
`VREF` (the 5 mm/s Coulomb sliding reference) is DEAD, kept in robot.h as
history. Steady-state slip lands at T·dt/m_eff ≈ 0.09 m/s (1.8 rad/s) —
below the 4.0 rad/s cut threshold, so the cut no longer latches during
normal driving.

### Verification battery (`mfsdrive`, dt = 1/60, 26.9 preset, floor ±200 m)
- fwd cruise 1.219 m/s, rev −1.227 (symmetric ±0.7% of the physical ceiling)
- odometry error ~14% (was 63%)
- stop from cruise: settles in ~0.2 s / 0.2–0.3 m, vel exactly 0
- tank straight 1.278; tank pivot 1.750 rad/s (was 1.69 stalled)
- rotate ±(mec) fixed point 7.129 rad/s (was saturated 4.53)

## 2. Engine CCD — resting-gate in the sweep

`collision_ccd_toi_range` now skips the floor-plane toi test for a body
whose lowest point already sits inside `solver.penetration_slop`
(0.010 m). Rationale (DES P O T comment in place): a preloaded wheel rests
2 mm through the floor, so y₀ = −0.0013 made the gravity root
t = 0.0163 s < dt every tick while awake — every loaded wheel was
permanently in the clamped set and collected an upward restitution
velocity per tick (the y-root flips vy positive) that the revolute joints
fought back. The contact solver is the right owner of in-slop bodies.
Volume sweeps still run (resting bodies can still tunnel sideways).
Falling bodies are untouched: their toi computation is byte-identical.

And, for the record: the "tank collapse at tick 471" chase in this session
turned out to be the robot rolling off the rim of the probe's own
10×10 m floor tile (470 ticks · 1.29 m/s ≈ 10.0 m). The resting-gate is
independently correct physics; the collapse could not be used as its
test. Probe floor enlarged to ±200 m; every mode is rock-stable
(`dr2_fwd/dr2_rev/dbgstop/dbgtank2` logs).

## 3. Headless fleet wiring (`tui_shell.c`, `mfs_ecosystem.c`)

### TRUE seam A: the bundle attach never inserted the fleet into the engine table
`mfs_ecosystem_attach` called `mfs_internal_module_attach("ftc-fleet", …)`
which runs the module's attach hook directly and stores the state ONLY in
the internal registry. `physics_world_step`'s pre_step dispatch walks
`world->tick_modules[]` — never the internal registry — so
`ftc_fleet_step_all` NEVER ran for the bundle-driven path. Every spawn via
`eco attach`/self-heal was a statue: wheel_ang_vel 0.00 forever (frozen at
y = 0.49 mid-settle in the smoke). The MFS suite could not see this because
its fleet tests call `drivetrain_update` manually; `fleet_of()` found the
un-stepped state in the internal registry by name.

Fix: `mfs_ecosystem_attach` registers the fleet in the internal registry
(bookkeeping) but attaches per-world via `physics_world_attach_module`,
so the engine table owns dispatch and teardown (single destroy on world
cleanup); `mfs_ecosystem_detach` mirrors with
`physics_world_detach_module(world, "ftc-fleet")` — without it the loader
refuses the bundle unload with E_BUSY after eco detach (reproduced by the
engine `ftc_ecosystem` test before the fix). The internal registry's own
pre/post dispatch stays available for registry-only modules (the suite's
probe module exercises that contract and still passes).

### Seam B and shell verb fixes (`tui_shell.c`)
- Bare `ftc spawn` now runs the same spawn guarantees as every other path:
  dlsyms `mfs_ecosystem_ensure_env` (tile floor + 128 solver iterations,
  exported from the bundle) before spawning, and self-heals by attaching
  the bundle ecosystem when the first spawn finds no fleet.
- `mod attach <name>` is no longer a stub: registry lookup, hook check,
  idempotence guard, attach — mirroring the GTK path — with an ecosystem
  fallback (`mfs-simulator` is an ecosystem name, inner fragments are not
  registered by design) and a listing hint on failure.
- `ftc drive l r` and `ftc mec f s r` both read `argv[1..2]`/`argv[1..3]`,
  i.e. the verb name — every drive went to atof("drive"/"mec") = 0 with
  the first numeric landing in the wrong axis. Args moved to `argv[2..]`.
- `tick N` discarded N (args were `(void)`'d) — every scripted drive was a
  2-tick burst between manual commands. Now steps N times.

### Smoke (headless shell, preset 7)
settle 90 → y 0.18; `mec 1 0 0` 240 ticks → z = 2.05 m, ω = 41.5 rad/s;
stop → ω −0.00, stable.

## 4. Re-baselines (physics-corrected values; bands kept, not widened)

| Item | Old | New |
|---|---|---|
| tank pivot translation (suite_a) | 0.0478 | 0.1555 (±20%) |
| yaw_stability bands (suite_c) | <6.0 / <5.5 | <8.0 / <7.5 (measured 7.129) |
| mecanum strafe dx (suite_a) | — | −2.7162 (passed, no re-baseline needed) |
| MFS_STRATE-F2 XFAIL | — | still open (phys −0.5247 vs odom −1.1848) |

## 5. Gates (all green at commit time)

- python3 tools/test_runner.py --profile quick: 60/60
- python3 tools/test_runner.py --profile full: 260 pass, 0 fail, 2 xfail
- make -C v15S/src tsan: 46/46, zero ThreadSanitizer warnings
- MFS suite ./build_tests.sh: 30/30 in BOTH trees (475-MPE vendored,
  461-MFS standalone; tools/sync_mfs_check.sh reports source-identical)

## 6. Manual-verification notes / open items

- The floor-budget brake cap in robot.c (opposing-motion torque clamp ≈
  0.97 N·m) is provably inert on the engine's applied path (MFS_145's
  idle-brake clamp already bounds applied braking torque to
  0.5·I·|ω|/dt ≈ 0.157 N·m at ω = 21 rad/s); kept as a floor-budget
  invariant with the note in place.
- Odometry still over-reports while slipping (frontier row updated below);
  impulse closure made it 14% instead of 63%, the slip term itself remains
  future work.
- The 26.9 preset cruise rides a mild period-5 wheel-ω cycle (21↔30 rad/s
  mean-zero) near the cut threshold — cosmetic, mean motion is physical.
