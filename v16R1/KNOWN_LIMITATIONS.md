# MPE Known Limitations — v16R1

> The honest, complete list of what MPE does **not** do, what is unsupported,
> and what is a documented frontier. Every item here is measured, not
> assumed. Nothing in this list is a hidden bug; all are known, tracked, and
> labeled.

---

## 1. Physics — Unsupported / Documented Frontiers

### Contact-model tuning
The rolling spin patch uses a 0.5 mm minimum effective penetration so resting
yaw continues to decay at zero-depth/slop contacts. Removing the floor stopped
that decay in measured scenes. Rolling resistance now uses the compression-only
normal impulse, but spin-patch size and the mass-weighted two-body split remain
empirical calibrations, not a Hertz-contact derivation.

### Rest-gap closure gates
The split-stage rest-gap branch (flat-cap float fix, 2026-10-20) closes an
all-legs-gap static-pair manifold toward mean touch at 0.5 mm/tick and stops
when the nearest leg is within 0.5 mm of the plane, so a flat-resting body
parks up to ~0.5 mm above touch instead of inside the whole 10 mm admission
band. Class asymmetry is accepted: a tilted rest whose legs straddle the
plane (some legs over, some legs gapped) is mean-touched once, not
edge-crept, so its uphill legs stay inside the admission band at rest.
Both are measured trade-offs, not hidden bugs.

### CCD-TOI gravity window (flat constant-velocity post-impulse flight)
A sweep-clamped body's velocity is advanced by the CCD to the exact touch
speed `v_toi` (gravity and drag over `[0,toi]` banked) and its position is
teleported to the hit. The remaining `[toi,dt]` window is the post-impulse
flight, which integrates at **constant velocity — gravity inside it is
deliberately not applied**: the velocity integrate keeps the full dt for
forces/torques/damping (motors, springs, L-conservation rotor, quat
normalization) but the gravity push is skipped for clamped bodies
(DESPOT-2026-10-20 CCD-TOI GRAVITY; the GUI driver shares this canonical
step). Consequence: a
clamped tick's normal impulse is `m·g·toi`, not `m·g·dt`; a constantly
re-clamped rolling contact receives slightly less support than a stack
resting in-place, and locked-rotor drive regimes validate at their own
pre-baselined numbers (MFS pivot translation 0.0478, fwd/rev antisymmetry
band 0.45 — see docs/AUDIT_REPORT_2026-10-20.md, fourth finding).

### Warm start
**Cold by default.** No effective warm start on any row — measured, not
assumed. The normal is cold every tick (restoring last tick's normal ejected
the 10-stack at ~13 m/s). The tangent impulse seed was deleted by the very
Coulomb clamp added to protect it (`fn = 0` → both tangents zeroed). Only
the tangent **frame** adoption is live.

Damped warm-start (`solver.warm_start_damping`, default 0.0) is experimental:
faster convergence at low iteration counts, unstable under extreme
gravity/friction/restitution. F11 pins it to 0 for that reason. Full restore
(1.0) is a measured regression (f10 drops from 27/27 asleep to 4/27; f11
overlap 0.2791 m vs 0.05 m bound).

**Consequence:** the engine needs **96–128 iterations for a 10-cube stack —
12–30× the published practitioner budget** (Catto, Solver2D 2024: "typically
4 to 8 iterations"). That is the real outstanding physics question.
Primary source verified 2026-10-24: Erin Catto,
[Solver2D](https://box2d.org/posts/2024/02/solver2d) (box2d.org,
2024-02-05), §"Important concepts relevant to all PGS solvers" — quote
verbatim. The same post defines warm starting and documents that its PGS
reference (Box2D-Lite) "uses warm starting and Baumgarte Stabilization":
the 4–8 budget assumes warm starting; these are cold-solver costs.

### Split impulse per-contact with lever arms
Tried twice (independent per-contact pass; shared Gauss–Seidel with
accumulated per-point impulse). Both measured worse (4.2× worse overlap on
the 10-high pile; net regression across the F11 seed sweep). Whole-body
translation stays. The genuine remedy for deep overlap is solver iteration
count (0.31 m at 64 → 0.0024 m at 96 → exactly 0 at 128 for the same scene).

### CCD — rotation-only tunneling
Translation-only for volume-object pairs. Floor plane CCD includes the lowest
point's rotational velocity (`v_center + w × r_lowest`), so tumbling boxes'
dipping corners clamp. But **rotation-only tunneling between arbitrary
volumes is unsolved** — fast blades / propellers need substepping or external
tip-speed bounding. Obstacle rotation beyond the margin (`max|ω|·R`, capped
2 m/s) is unsupported.

### Multithreaded solve
Per-body work, CCD, pair-local collision detection, and solver sweeps across
independent islands parallelize via `MPE_THREADS`. Shared-body contact
preparation and the joint solve (`constraint_solve_all`) stay serial. The
Gauss-Seidel order within each island is preserved; within-island row
parallelism remains future work.

---

## 2. Platform / Toolchain

- **MSVC bitwise twins:** unproven. GCC/Linux passes the full suite under
  ThreadSanitizer with `setarch x86_64 -R`; MSVC still needs its own
  `/fp:strict` build and determinism comparison. Without the launcher, this
  container's TSan runtime can fail while mapping shadow memory.
- **Windows/macOS:** unsupported as release targets (Windows build exists
  via MSYS2/MinGW but is not CI-proven).
- **Big-endian hosts:** unsupported — little-endian only; big-endian fails
  at compile time (`scene_saving.c #error`).
- **Wayland:** mouse lock is Wayland-safe by design (no X11-only lock) but
  P1-unverified until a Weston/Sway matrix is logged. X11 path verified.
- **32-bit:** untested; `size_t` / pointer arithmetic assumptions may bite.
- **MicroVim path jail:** non-listed files are confined to direct children of
  `status/`; the fixed known-file list still includes project-relative paths.
  Symlink and concurrent path-replacement hardening is not a sandbox boundary.

---

## 3. Engine Scope

- **GUI vs headless frame drivers:** the GUI still accumulates wall time
  (with an anti-spiral clamp) while headless callers pass dt directly, but
  both now call the same `physics_world_step()`. Bitwise identical on passive
  towers; driven robots differ ~2.7% via motor-observer feedback (expected,
  not a path bug). The former duplicate GUI stage sequence was removed
  2026-10-23.
- **Scene format:** v200 saves bodies (stable IDs, sleep state, damping)
  plus springs + revolute/fixed/distance/prismatic/rope joints, with CRC32
  integrity footer. Files v130/v140/v150/v151/v152/v153 load via the legacy
  reader.
- **Joint UI/persistence scope:** solver supports spring/revolute/fixed/
  prismatic/distance/rope; menus create springs (+revolutes in TUI scenes);
  v200 persists springs + all five constraint types. In-engine creation UI
  and scene persistence for fixed/distance/prismatic/rope is post-release
  work.
- **Per-object config persistence:** beyond `nice_value`, not in scene files.
  The saved `nice` domain is the engine's [0, 100] sanitize invariant — the
  three engine paths (`nice`, `renice`, `sed nice`) now clamp there, so a
  saved value can no longer veto the file on reload (2026-10-20 fix).
- **Capsule plugin scope:** example-module tier. Normal conventions are A→B
  with engine-parity inside-solid contacts (2026-10-20 fix); outside-cube
  sampling keeps the per-axis clamped walk segment fidelity the module
  shipped with; exact segment/OBB distance inside the engine core uses the
  engine-side ternary sweep (documented in DESPOT_AUDIT_2026-10-19.md).

---

## 4. FTC Robotics — Documented Frontiers

| Issue | Detail |
|---|---|
| MFS CCD obstacle-margin regression | Resolved by per-pair margins in the 2026-10-19 audit; the prior red tank/drive/odometry rows are historical, not current open failures |
| Jointed air-spin limit cycle | Free-spinning jointed wheels oscillate (motor 86 rpm vs true 799 rpm); contained by bearing damping (c=1e-6) + diode+slew |
| MOTOR-III observer limit cycle | Observer-armed driven path is a documented limit cycle at −61.3%; open-loop no-load line is exact |
| Odometry over-reports slipping contacts (peel) | Impulse-closure traction (2026-10-21 audit) removed the VREF slip-lock and cut odometry error from 63% to ~14%, but the odometry model still has no slip term; the suite tracks the remaining peel under MFS-STRAFE-F2 (strafe phys −0.5247 vs odom −1.1848) |
| Cruise wheel-ω micro-cycle | 26.9 preset full-command cruise rides a small period-5 wheel-ω cycle (21↔30 rad/s, mean-zero, below the 4.0 rad/s cut threshold); mean motion is physical — cosmetic only |
| Mecanum strafe magnitude | Solver frontier (5-link ground→roller→bearing→hub chain doesn't converge in GS-128); analytic mode is the workaround |
| Full-throttle fast-preset long cruise can topple | Fleetless probe (hand-run drivetrain_update, preset 2/5_2, seg=1.81, cmd=1.0): 28.5 s clean to ≈5.6 m/s, then vertical seesaw — solver-pinned wheels (vy exactly 0) alternate quadrants with peers driven −2..−3.6 m/s into the floor (−2 cm burial within ~5 ticks), all-wheel cscale 0.35 slam same tick, +0.5..+1.3 m/s upward vy launches, pitch +14°, tumble near tick 1770–2000. Boundary: cmd 0.90 (v≈5.2) stays clean 1920 ticks; preset 26_9 is clean at cmd 1.0 (47.36 m/40 s, y 0.182). Exonerated by instrument: CCD resting-clamps (only legitimately airborne bodies), depenetration (position-only), restitution (wheel+floor e=0, pair rule fmin). Live suspects: solver-level coupling at the slip-cut churn boundary (cruise slip 3.4 rad/s vs 4.0 cut) — never reproduced in the 26_9 suite preset; TUI/graphical sessions with high free-ω presets (5_2, 50_9, 71_2, 139) should branch below full or use 26_9-class presets for long cruises |
| Long cruise yaw drift (same regime) | Full-throttle longitudinal cruises curve slightly (probe preset 7 71_2: x=+2.87 m/2400 t, curvature ≈0.0012/m; preset 9 139: +3.07) while 26_9 is straight (curvature <0.0001); same near-free-speed slip-churn regime as the topple row above — not seen at cmd ≤0.9 |
| Hive-aim entering band is narrow (13% of aim space) | Only elevation ≈55–75° at power ≈0.62–0.82 enters a tilted up-cell mouth, and the band shifts with pose/gap (gaps 0.9–3.4 m measured). Eye-aiming at the stick-centred default (50°) always clips the mouth's underside — this is why `ftc_aim_params_solve` exists: it marches the entry arcs and returns a known-entering pair (grids elev 45..80 / power 0.40..0.90, lowest-elev row first), so it can answer "no arc" rather than guess; fail cache and success cache both keyed to pose (muzzle 0.35 m / yaw 0.09 rad / mouth move 0.08 m) |
| Closest-up-cell pick is geometric, not camera-facing | `ftc_aim_params_solve` picks the nearest up-cell mouth (2D range), which between the two hives is not always the mouth the player faces; the −1 failure hint names the LOS azimuth off the nose so re-aiming is mechanical. The solve is advisory only: it publishes nothing on its own (gamepad publishes only while charging with the stick centred; the shell `ftc aim` verb is read-only) |

---

## 5. S2 Bridge (MGB)

- **Phase 0:** kinematic display only — gravity and contact impulses do not
  move the staged dish; `k=0` bonds render without applying force.
- **Phase 1:** moves existing kinematic bodies per refresh tick
  (`cfg->every > 0`); bonds remain `k = 0` (display-only).
- **Physical coupling:** positions flow S2 → MPE, never back. Out of scope
  forever by design.

---

## 6. Performance

- No SIMD autovectorization reliance; scalar core with optional SSE2 SIMD
  wrappers (bitwise identical by construction). ~1,136-object practical
  wall.
- Single-threaded solve (see above).
- Float32 world: playable volume bounded (±250 m); float resolution
  (~0.03 mm at corners) sits 300× below contact slop. No origin rebasing
  inside the boundary box.

---

## 7. What Is NOT a Limitation (Deliberate Design)

These are sometimes filed as bugs but are intentional:

- **No velocity clamps / restitution caps:** game-only damping (`nice_value`,
  angular scale) labeled and defaulted off/vacuum. Under extremes, perpetual
  fall/creep can be the true outcome — F11 is a robustness verdict
  (no NaN, nothing fallen), speeds reported, never gated. The report tracks
  the 10-cube column and prints `TOPPLED` with its max drift, so a fall is
  visible even though it does not flip the corruption verdict.
- **Sleep is non-physical:** `sleep.enable = 0` runs sleepless validation.
  Tall-tower equilibrium depends on the sleep optimizer: the 10:1 column
  drifts apart under solver micro-jitter without it, so F11 pins sleep ON.
- **F11 "nothing fallen" gate is belt-and-braces:** the world-edge safety net
  is pinned ON during torture and clamps every body unconditionally, so
  "nothing fell through the world" is still effectively unfireable. The real
  property (contact solver holds bodies up with the net switched off) is
  gated in `f10_long_run` phase 2.
- **`max_linear_speed` / `max_angular_speed`:** inform-only counters,
  never scale velocities (CCD owns fast bodies).
- **Spring pure-Hooke:** only exact when `max_acceleration=10000` +
  `k < k_stable` + no Courant warn; otherwise softened for stability.
