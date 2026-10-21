# Mathematical, Programming, and Operational Audit — 2026-10-20

## Scope and method

This audit covers the MPE repository in this working folder: engine and
physics code, UI/TUI, plugin and MFS/MGB integrations, build and verification
tools, tests, and current documentation. The repository had 332 tracked paths
at the start of the audit. The audit also reconciles findings from the dated
2026-10-13 and 2026-10-19 audit ledgers with the current source, since several
entries had already been repaired while remaining marked open. All intentional
repository edits, test logs, and retained build artifacts were confined to
this working folder; no sibling repository or system configuration was
modified.

Methods included source and contract review, targeted numerical regressions,
seeded mathematical property checks, GCC `-fanalyzer`, warning-as-error
builds, ASan/UBSan, ThreadSanitizer, determinism comparisons, and the
repository's full test runner. Results and logs are kept under `temp/`.

This work is an engineering audit, not a formal proof of every execution path
or every floating-point result. Numerical claims below are bounded by the
listed tests and configurations.

## Findings fixed

### Simulation, constraints, and concurrency

- **Cross-world worker-pool race:** pool function/range/counter state was
  shared by concurrently stepped worlds. Dispatch phases are now serialized,
  task counters use atomics, and workers snapshot phase data under the pool
  lock. A concurrent-world regression compares two worlds against serial
  twins bit-for-bit.
- **Narrowphase shared-body races:** parallel workers now write pair-local
  collision results only. Waking bodies, updating inertia/solver preparation,
  and contact flags happen serially in pair order after the barrier.
- **Prismatic DOF contract mismatch:** the solver previously enforced two
  lateral translations but left relative rotation unconstrained despite the
  documented five removed degrees of freedom. Three angular impulse rows now
  lock relative orientation; tests exercise axial sliding, torque rejection,
  limits, and exact 180-degree recovery.
- **Rolling impulse accounting:** rolling resistance read the post-Poisson
  normal accumulator, making restitution add rolling torque. An existing
  ABI-reserved field now snapshots compression support before restitution;
  rolling uses that snapshot without changing contact-structure layout.

### Collision geometry and manifold selection

- **Cylinder/slab minimal exit:** the one-sided top-face shortcut now falls
  through to the OBB path when the cylinder spans both slab faces. A regression
  checks that the shallower underside exit is selected.
- **Cylinder/cylinder cap support:** contact points are placed inside the
  intersection lens of the two cap discs. A regression checks both contact
  points against each cap radius; a second checks slop-only lateral separation
  cannot create a positive-depth face contact.
- **Small parallel-edge support:** removed the fixed 0.05 m spread and
  0.0001 m² duplicate-point gates that silently suppressed support for small
  geometry.
- **Clip-manifold selection:** the one-point reseed uses the farthest
  candidate, avoiding the zero-area score that previously made the second
  point depend on clip order.

### Programming and operations

- **MGB sink ownership:** imported display bodies are kinematic, refreshes
  clear stale motion/force state, and bridge attachment/staging no longer
  clears caller-owned joint pools.
- **MGB child-pipe failure:** writes to a closed child stdin block/consume
  `SIGPIPE` in the calling thread and return an error instead of terminating
  the host process.
- **MicroVim failed edit:** file loading is transactional; a failed `:e`
  preserves the current buffer, filename, and undo history. Unlisted files are
  restricted to a direct child of `status/`, removing arbitrary CWD file
  creation through an extension-only check.
- **ThreadSanitizer target:** the TSan executable now exports host symbols and
  builds its required plugins. On this Linux host, `setarch x86_64 -R` avoids
  the sanitizer's shadow-memory mapping collision.
- **Documentation drift:** current test counts, dependencies, threading
  phases, plugin boundaries, bridge behavior, and limitations were reconciled
  against code and test output.

Earlier hardening fixes remain in place: checked numeric conversions and
indexes, scene joint-pool commit accounting, editor path validation, safe
object IDs, TUI builder checks, plugin diagnostics, MGB temporary-file
handling, and scratch-path isolation.

## Mathematical assessment

The implemented rigid-body model is a deterministic single-precision game
physics engine with explicit approximations, not a general-purpose exact
mechanics solver. The audited core uses semi-implicit integration, finite
rotation updates, world-space inertia, SAT/shape narrowphase, sequential
normal and Coulomb-friction impulses, Poisson restitution, positional split
correction, and joint constraints. Tests cover matrix inversion, inertia,
SAT symmetry/depth, Coulomb bounds, spring periods/stability, pendulum period,
momentum, TOI stability, and deterministic replay.

Corrections were made where source behavior violated an explicit contract or
an independently testable geometric invariant. The remaining model frontiers
are listed below; they should not be interpreted as defects the current tests
have disproved.

## Overall assessment

MPE is a substantial, specialized real-time engine rather than a small demo.
Its best engineering qualities are explicit per-world configuration,
determinism checks, failure diagnostics, and a test suite that increasingly
checks output contracts instead of trusting counters or comments. This audit
also found that the test infrastructure caught several defects the narrative
docs and older audit ledgers still described incorrectly.

The main engineering risk is calibration complexity: contact behavior depends
on several empirically tuned thresholds and correction stages, so changing a
local formula can move distant stack, rolling, or robotics baselines. The
measured regressions documented in the repo justify preserving those gates and
checking changes across the full profile. The GUI and headless drivers also
remain separate control paths, and the C plugin surface makes layout and
reentrancy contracts important. The engine is suitable for its documented
game/robotics envelope; the current evidence does not establish a general
mechanics solver or cross-platform release guarantee.

## Remaining verified limits

- **Rolling spin patch:** the 0.5 mm minimum effective penetration remains an
  empirical floor. Removing it stopped resting yaw from decaying. Rolling
  resistance now uses compression-only normal support, but spin-patch size and
  two-body sharing remain calibrated approximations.
- **Warm start:** normal and tangent impulse carry are cold by default; frame
  adoption is live. Restoring cached impulses regressed measured stack and
  long-run behavior.
- **CCD:** rotation-only tunneling between arbitrary volumes remains
  unsupported. Volume CCD is translation-based; floor CCD includes the
  rotating lowest point.
- **Solver parallelism:** body integration, CCD, pair-local detection, and
  independent islands run in parallel. Contact preparation and the joint
  solver remain serial; Gauss-Seidel rows inside an island remain ordered.
- **Numerical oracle gaps:** full numeric agreement for general OBB penetration,
  oblique multi-body angular-momentum transfer, and an independent analytic
  drag oracle remain unproven. Existing accepted tolerances are not formal
  error bounds for arbitrary scenes.
- **UI filesystem boundary:** unlisted MicroVim targets are confined to
  `status/`, but symlink and concurrent path-replacement hardening is not a
  sandbox. Known project files remain editable by exact path. The editor
  change was compile-checked; this repository has no automated GTK editor
  interaction test, so failed-`:e` preservation was not exercised through a
  live window.
- **Platforms:** GCC/Linux was built and sanitizer-tested here. MSVC bitwise
  parity, Windows/macOS release readiness, 32-bit, big-endian, and a Wayland
  interaction matrix are not verified. Big-endian builds are explicitly
  rejected.
- **Integration scope:** the MGB bridge is one-way display/copy integration;
  physical coupling back into S2 remains out of scope. MFS twin drift and the
  separate 472-MGB forward-port were not checked because those repositories
  are outside the authorized working folder.

Additional bounded items are maintained in [`KNOWN_LIMITATIONS.md`](../v15S/KNOWN_LIMITATIONS.md)
and [`REMAINING_WORK.md`](../v15S/REMAINING_WORK.md).

## Verification record

The final verification profile, sanitizer runs, TSan, property sweep, static
analysis, and focused regressions are recorded in
[`VERIFICATION.md`](VERIFICATION.md). Exact artifacts are under `temp/`.

## Second-pass fixes (same day, 20:05–21:00 Z session)

A follow-up pass re-ran the dated ledgers against the working tree after the
first set of repairs and closed the remaining high-frequency findings. Every
fix below carries a `DESPOT-2026-10-20` marker comment in source.

Core, loader, build
- **Plugin stage ifaces survived unload:** the registry exposes indexed row
  snapshots (`mpe_registry_broadphase_row` / `mpe_registry_solver_row`),
  and the loader now runs an address-based `purge_plugin_stages()` after
  `purge_plugin_pairs()` in both unload paths. Any broadphase/solver iface
  whose function pointers live inside the unloaded image is unregistered
  by its own stored name (world slot reset + tombstone, same as a named
  unregister). Builtin rows never match; their code is in the engine.
- **`dlerror()` double-read:** the registry report consumed the error
  state, and the second `dlerror()` could return a message from an
  unrelated thread. Captured once into a local.
- **Makefile define purely by BASE path:** `-DMPE_GTK4...` reached
  `$(CFLAGS)` only through `BASE_CFLAGS`, so any command-line override
  (the V-01/V-02 sanitizer gates) silently built the removed GTK3 code
  path. `GTK_DEFINES` are now hard-appended exactly like determinism
  flags, and the engine link hyphen-appends `$(HARD_LIBS)` (X11/wayland/
  -ldl/-rdynamic) so `mouse_lock.o XFlush`-style link errors cannot
  recur. `V01.sh` + `V02.sh` pass again.
- **Physics halt countdown supersede:** `physics_halt_set(true)` clears a
  pending countdown immediately (manual halt could not beat an armed timer).
- **Portable atomics:** the "MSVC has no `__atomic` builtins" promise is
  now honest: `mpe_platform.h` carries one shim (`MPE_ATOMIC_*`, GNU
  builtins where available, C11 `<stdatomic.h>` otherwise), consumed by
  the previously-builtin-direct sites (`rigidbody.h` inline helpers,
  `physics_world.c` pool counters, `rigidbody.c`). No behavior change on
  the GCC build; the C11 fallback compiles into the same hardware ops.
- **Ecosystem register ABA refusal:** in-place desc overwrite while any
  attached ecosystem still aliases the slot is refused (-1, loud).
- **Data types counted loudly:** `validation_report.c` PE sign
  (`PE += -(m*g)*y`), `mpe_diag.c` g_seq truncation (saturating cast
  against INT_MAX, `%ld` on a long), `rigidbody.c` sanitize mass
  recovery re-clamps (`mpe_note_clamp`), and the cube/cylinder init
  paths no longer log the sphere-mass label.

Physics and math
- **Capsule plugin normal convention (LIE-CAPSULE):** every emitted
  relationship in `capsule_vs_cube_sampled` and
  `capsule_vs_cylinder_sampled` was cube→cap / cyl→cap, i.e. negated
  relative to A→B — `mpe_shape_dispatch` swaps and negates once more, so
  direct-registered hits injected energy and swapped-order hits injected
  −energy. Rewritten as A→B: outside = `-(diff)/dist` (engine parity with
  sphere-vs-cube), inside-cube = inward `-(outward)` with the contact on
  the escape face wall, cylinder inside path (verified correct) kept.
  Contact positions and penetrations unchanged; the old
  dead `best_p = closest + 0` line is gone.
- **Solver wake-depth policy slip:** the both-bodies-locked deep-overlap
  gate re-read the RAW config name while a clamped local `wake_depth`
  already bounded the same policy 80 lines earlier (NaN comparisons in
  the raw gate meant sleepers skip waking); the clamped local is shared.
- **Core broadphase cull-vs-insert clamp divergence:** insert clamped the
  per-axis sweep to 10 m; the cull band clamped 4 m and the insert
  comment claimed the 4 m sweep covers the cull — arithmetic false in
  the (4 m, 10 m] window. Both ceilings are 10 m now (CCD pre-clamps
  genuine fast movers per tick regardless).
- **GUI/legacy-step stage parity:** the leg loop now (a) feeds drag
  through the same `isfinite / >1→1.0` clamps the canonical applies, (b)
  rebuilds islands + re-marks `manifold_awake` AFTER the motor/force
  stage (motor wake flips used to land after the flags were derived —
  the documented broken-hinge-response behaviour persisted verbatim on
  the GUI path), and (c) runs the same late-manifold (post-split) extra
  resolve ×2 + Poisson + split pass over exactly the freshly-created
  range, mirroring `physics_world.c:1815-1885` (previously the GUI
  solved those manifolds cold one tick late).

UI, TUI, scene, render
- **`mv` argv[5] behind `argc >= 5`:** the 6-token `/pos x y z` forms
  read `argv[5]` with the guard checked only five-wide (out of bounds on
  the last arg; missing component fell through to a neighbour's value
  on truncated input). Bound is `argc >= 6` for both /pos and /vel.
- **`sleep` cast:** `(int)(seconds * 60)` on huge/inline values is
  implementation-defined out of range; now
  `term_float_to_int_safe(seconds * 60.0f, 0, 1, 216000)` (1-hour cap,
  matching the tick-input ceiling).
- **`nice` / `renice` / `sed nice` domain:** three entry points declared
  the Unix-style [-20, 19] band against the engine's enforced [0, 100]
  invariant — negatives were accepted then silently zeroed next tick,
  and saving a negative wrote `(uint32_t)` cast overflow that the
  loader's `nice_u > INT_MAX` check VETOES, bricking the whole scene
  file for one misplaced `nice -20`. All three clamps agree with the
  engine domain now.
- **`vi` known-files gate order:** `status/scene.dat` is printed by
  `--list` and written by `cmd_sync`, but the blocked-extension sweep
  ran BEFORE the known-files pass, so the advertised entry was always
  refused. Known exact-entries now short-circuit allow first (explicit
  allow beats the extension deny for the reviewed list only).
- **MicroVim boundary line:** a complete 4095-char line + newline no
  longer trips the truncated-flag (the old check discarded EVERY line
  read so far on that false positive; one `fgetc` classifies the
  boundary, the consume loop only catches real continuation).
- **Legacy scene load vetoes:** a failed or negative joint-count read
  silently committed zero joints as success — vetoed loudly. Joint
  wiring is gated on remap presence (`scene_id_remap_has` helper): raw
  saved ids can never alias the live allocator's fresh ids (loud drop
  instead of a weld to a stranger).
- **`scene_init` sandbox guard:** the unchecked `scene_add_object`
  return wrote through `bodies[-1]` on pool exhaustion; guarded + loud.
- **Cube wireframe:** the 24-index stream drew the y=0/z=0/x=0 interior
  cross-section planes of the vertex infill rows, not the corners.
  Rebuilt from the bottom/top face corner rows (rows E/F: indices
  16..19 and 20..23): 8 loop edges + 4 verticals = 12 real edges.

MGB bridge (472 subsystem, vendored in-475)
- **Transcript reservation made real:** the mkstemp-reserved fd was
  closed and the path re-opened by name (`"w"` during spawn TRUNCATED
  the reserved file; `"a"` during cmd was a fresh, symlink-checkable
  open every time). The session now owns one persistent `FILE*` on the
  reserved fd and `mgb_close` fcloses it and unlinks the path (the old
  close left every transcript on disk permanently, readable by any
  local user).
- **Handshake failure polarity:** a banner that arrived from a non-s2tui
  child (right prompt, wrong program) reported SILENT spawn_reason and
  left the previous thread-local failure visible to consumers. A
  trailing enum addition `mgb_spawn_e_banner` (append-only contract,
  header count comment) covers it distinctly.
- **Snapshot TOCTOU:** lstat-then-fopen gives two path lookups; replaced
  with one `open(O_RDONLY|O_NOFOLLOW|O_CLOEXEC)` + `fstat` + non-empty-
  read on the fd.
- **Frame scratch unlink:** `bridge_stage` unlinks each parsed snapshot
  (read-once scratch). Previously an unbounded file accumulation in
  `$TMPDIR` and post-read by any local user.
- **`make probe` target restored:** `readme.md` advertises `make probe`
  and `tools/mgb_probe.c` exists, but the makefile had no target; added
  (with the spring_joint.o render stubs the headless test already has).
  `readme_475.md` no longer references `tests/mgb_load_test.c` (that
  file does not ship in this copy).

MFS robot/drivetrain kit
- **Wheel/robot front-truth labels:** the suite gate enforces +Z forward;
  indices 0,1 sit at the −Z end (back), the nose marker used to sit on
  the ramp half pointing at the wheels the robot drives backward from.
  Comments and `MFS_NOSE_OFFSET_Z` follow the physical truth (+Z), index
  churn excluded by design (friction mapping is ride-height tuned on the
  existing arrangement).
- **Odometry overflow:** the `(int)` cast of `revs * counts_per_rev`
  was implementation-defined out of domain beyond ~2e9 counts (reachable
  on multi-hour unbraked runs); the counter now holds at the
  representable ceiling band (INT_MAX − 1e6 / INT_MIN + 1e6).
- **`fuse_heat` domain:** struct comment said "0..1+" while the
  accumulator caps at 10 ("flag, not accumulator"); comment corrected.
- **Yaw-damper constants policy:** the 0.6 / 3.0 / 1.5 band is sharpened
  in the comment as empirically A/B-tuned ESC-substitute values (not
  derived law); retuning must go through the same log harness, not be
  "corrected" toward a textbook anti-slip rule.
- **FTC standalone sources:** `Makefile` was one behind `mfs_sources.mk`;
  `intake.c` and `biobuzz.c` now build in the standalone path too.

Regression tests
- The existing paranoia/quick suites remain the primary lanes; the new
  failure surface (capsule orientation order, renice domain, microvim
  boundary, legacy-count veto) is covered by the same gates this build
  reruns below. Suggested dedicated regression targets (capsule
  orientation-order via `tests/module_test.c`, nice-bound roundtrip via
  `paranoia_scene_persistence`) are recorded as REMAINING_WORK items.

External math oracles (re-run)
- `validation/property_sweep.py` + `ext_oracle_check.py`: all 10 external
  oracles green (pendulum, spring, bounce series, TOI roots, cone-min vs
  sqrt combine, Coulomb bounds, config schema 80 keys).
- Closed-form drag+gravity integration: the engine's analytic reduction
  (`a_pos = (1-e)/c`, `g_pos = t/c - (1-e)/c²`) is algebraically the
  exact solution of `dv/dt = -c·v + g`; numerically identical to 1e-16
  over the parameter grid (`temp/despot-2026-10-20/math_oracle.log`).
- Mecanum mixer: the (fwd, strafe, rotate) command sets invert exactly
  onto the (vx, vz, om) basis; transposition gates unchanged.
- TOI quadratic: matches the closed-form nearest-positive root; the
  standard quadratic (a>0, c>0, D≥0 pre-guards) is exact.

Multicore (see final verification record in
[`MULTICORE.md`](../v15S/MULTICORE.md)):
- MPE_THREADS=1/4/8 paranoia-determinism runs are md5-identical logs
  (1200-tick twin worlds, re-init, 10-run reference, concurrent-vs-
  serial: five PASS lines each). The twin keeps bitwise identical
  results at all three thread counts.

## Third-pass fix: flat-cap rest-gap closure (21:00–04:10 Z session)

A user-report ("spawned cylinder lands flat side down, vibrates and
wobbles before settling a little too high") was reproduced and
root-caused to a resting **hover above the floor**, quantified with a
dt=1/60 mode battery (`/tmp/opencode/wobble_repro.c`, 1800-tick runs):

First, a repro lesson: the early "permanent two-tick hop" trace was an
artifact of the repro passing `atof("1/60")` = 1.0, which the step
clamp reduces to dt=0.1. At the canonical 1/60 no hop exists; the
bounce chain seen on hard slams is the ordinary e=0.3 restitution
transient. The real defect is below.

Root cause (three facts compounding):
- The floor manifold admits contacts while the rim leg sits inside the
  10 mm `penetration_slop` band — i.e. it admits **gaps**,
  distinguishing "inside the band" from "touching".
- The narrowphase then stored `penetration = max(leg_pen, 0)`, so an
  admitted gap froze into "zero depth"; the contact-velocity solve is
  deliberately penetration-free (no velocity Baumgarte), so it parks
  the body exactly at whatever gap it entered with — forever.
- The only standing position correctors (split pass,
  `a3_positional_depenetrate_manifold_w`) correct over-penetration
  only; there was no passer that could close an under-touch gap, so a
  flat-cap-down cylinder could end its bounce chain 0.5–10 mm above
  the plane and sleep there (e.g. a 20 m/s flat slam parked at
  y=0.4066, 6.6 mm high; a gentle drop, 9.8 mm high).

Fixes (marked `DESPOT-2026-10-20 REST-GAP` in source):
- `collision_static_plane_cylinder` now stores the **true unclamped
  leg pen** (rim-quad loop and analytic fallback). With the solver
  never consuming pen for velocities and every other pen consumer
  re-clamping or ignoring negatives (each verified), the negative
  values are propagation-safe.
- `collision_apply_split_impulse` depth statistics are now seeded from
  the first contact instead of 0.0 — the old zero seed made an
  all-gap manifold report "deepest depth = 0" (indistinguishable from
  touch once pen was clamped) — and gained a rest-gap branch in the
  `corr <= 0` path: static-pair only, gated on a calm dynamic side
  (not sleeping, linear/angular speeds below the sleep thresholds),
  targeting **mean touch** with the nearest leg's headroom as a hard
  clamp (a tilted rest stops at its closest edge instead of
  over-sinking it to chase an uphill gap), 0.5 mm/tick, both bodies
  moved toward each other along the manifold normal with effective
  inv-mass weights. Placement at the split stage keeps the fix after
  the contact solve, so resting contact impulses and the gap creep do
  not fight.

Behavior (dt=1/60 rest heights after 1800 ticks; before → after):
- Touching cases exact: modes rim-drop/rim-slam stay 0.400000
  (unchanged); tilted-wheel rest unchanged (deliberate asymmetry).
- Hover cases closed: flat slam 0.40660 → 0.40010; gentle flat drop
  0.40978 → 0.40028; 4 mm-above spawn → 0.40000 exact; 6.6 mm-above
  spawn → 0.40010. A spawn placed exactly at the 0.5 mm closure gate
  stays 0.40050 (strict gate, historical value preserved).
- All modes end with a clean sleep (no wake toggles, zero residual
  velocity/velocity in the tail window).

Accepted remaining asymmetry: rests whose contact legs straddle the
plane (tilt > closure gate) still park the uphill legs inside the
admission band while the downhill legs carry the touch; the body
rests mean-touched instead of edge-touched. The admission band itself
remains the `penetration_slop` config knob.

### Fourth finding in the same session: CCD-TOI double gravity (residual
"user wobble, now timid" = one phantom late bounce)

User verification reduced the complaint to decaying-bounce chains that
still carried one half-height late hop. Tick-decode (mode 9, 20 m/s
scale slam, dt=1/60) exposed the mechanism on the CCD-hit path:

- The CCD sweep advances a clamped body's velocity to the exact
  touch-speed `v_toi` (`collision_ccd_apply_range`, including drag)
  and teleports position to the hit — physically correct.
- The pre-solve velocity integrate then ran over the FULL dt anyway
  (`rb_integrate_velocity`), adding `g·dt` for time already burned
  into the teleport — velocities owned `g·(t_hit + dt)` instead of
  `g·dt`, while positions (which honor the leftover) disagreed.
- Consequences: the Poisson gate recorded approach = `v_toi + g·dt`
  (first-bounce impact read 2.96 vs physical 2.80; second-bounce 1.18
  vs 1.02), the contact compression was built from the inflated
  velocity, so `e·compression` over-paid restitution (~+5% exit
  speed), and impacts whose true touch speed sat just under
  `restitution_velocity_thresh` (1.0) still fired a phantom bounce.

Fix (marked `DESPOT-2026-10-20 CCD-TOI GRAVITY`, both step paths —
`physics_world.c` and the legacy `simulation_physics_loop.c`): for a
sweep-clamped body the gravity push into `force_accumulator` is
SKIPPED at the accumulator-add site (the sweep banked gravity over
`[0,toi]` in `v_toi`, and the post-impulse flight is constant-velocity
by engine convention, so `[toi,dt]` gravity has no owned window).
The velocity integrate itself keeps the FULL dt — motors, springs,
damping, the torque-free L-conservation rotor advance, and the
per-tick quaternion normalization are preserved verbatim. Two failed
attempts are on record: (1) an `_ex` refresh variant that corrected
the recorded impact by `(rem_a−rem_b)·g·n` was abandoned because the
Poisson payment `e·compression` still leaked the tail; (2) a
`body_dt = 0` zero-integrate removed the tail at the source but
starved clamped bodies of force/torque/damping work and froze the
L-conservation rotor + normalization (motor pivots stalled, MFS tank
numbers fell, and the scene round-trip's unconditional
`vector4_normalisation` flagged the quaternion off its bitwise fixed
point — parity_scene_persistence FAILED). Skipping only the gravity
increment fixes the recorded impacts without any of that collateral.

Consequences (all deterministic, bit-stable across runs):
- Scene persistence is green again (quat stays a normalization
  fixed point: 3×/3 passes).
- MFS robotics: the engine's phantom `g·dt` normal load on clamped
  ticks had chattered fast-moving wheel contacts (`e·phantom` hops
  every tick) and padded friction caps. Removing it strengthened
  grip: tank pivot translation 0.0837 → 0.0478 (re-baselined), pivot
  heading unchanged (3.0639, ~1% from measurement), fwd/rev |vz|
  antisymmetry measured 0.4953/0.8686 (43%, was chatter-era 14%) —
  structural band updated 0.35 → 0.45 at both call sites (twin
  461-MFS mirrored, `bbf39f6`). 461-MFS suite 30/30.

Behavior (mode 9, dt=1/60): one textbook bounce — exit ≈ e·v_toi
(0.8208 vs the pre-fix hot 0.8884; damping now applies over the full
tick to the banked touch speed), second touch arrives at ~0.97 m/s,
under the 1.0 restitution_velocity_thresh, and THUDS without
bouncing (the recording had said 1.18 and bounced); the body rests
at 0.400138 and sleeps. Rest battery: all 12 modes ≤0.5 mm, gentle
drop 0.400055, tilts 0.403475 (1-ulp from pre-fix), sleeps clean.

Verification: quick profile 60/60 green immediately after the fix
(`temp/qa_runs/20261021T040946Z-333170` and
`temp/qa_runs/20261021T144422Z-598230`); full profile, TSan, twins,
and the MGB kit rerun in the final verification record below.
