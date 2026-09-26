# Codebase audit: 2026-09-24

## Scope and method

The audit focused on the active `v15S` engine and its build, test, TUI, scene
I/O, and MFS robotics paths. It reviewed numerical integration, collision and
constraint behavior, persistence, allocation/error handling, plugin rebuilds,
headless side effects, and test oracles. It also ran the 30 legacy tests plus
14 paranoia cases, the canonical Suite v2, MFS gates, TUI smoke, a normal
warning-enabled build, and GCC `-fanalyzer` across the active engine sources.

This is not a formal proof of the simulator or a sanitizer run. No `v15R3`
source tree is present in the workspace; only its release notes are available.
Floating-point and contact results remain bounded by the documented
tolerances and supported shape models.

## Findings implemented

- **Rigid-body math and world lifecycle:** removed an uninitialized-state probe
  from world initialization, corrected reset ordering for the contact speed
  bound, removed a dead conditional, made kinematic rotation integration
  effective, and validated exact position integration inputs.
- **CCD and contact geometry:** replaced cancellation-prone sphere-sweep roots
  with a double-precision stable quadratic solve and finite fallback; routed
  sphere, custom-proxy, and cylinder sweeps through it. Fixed coaxial cylinder
  caps colliding at a false capsule-end separation and added an exact
  axis-aligned box-top/cylinder case.
- **Scene files:** validate body/constraint fields, aggregate joint limits,
  CRC and trailing data before growing live pools; propagate flush, mode,
  close, and directory-sync failures around atomic save/rename.
- **Test truth:** repaired invalid or weak fixtures and oracles for exact
  free-flight, bounce restitution, rolling range, pendulum period, motor
  anchors, weld/rope behavior, energy/momentum, spring stretch, broadphase
  contact bands, cylinder pose, and actual floating-point subnormals.
- **Allocation handling:** mesh builders now validate sizes and arguments and
  return cleanly on allocation failure. The terminal editor now checks line
  buffer and undo allocations, restores snapshots transactionally, discards
  redo history on new edits, bounds substitution output, and clears undo state
  on reload/close.
- **Operational paths:** test/script scratch and logs are directed to the
  project `temp/` directory. Headless runs disable joystick probing. The suite
  build now re-enters the MFS Makefile before loading its bundle, avoiding a
  stale plugin after source changes. `tee` writes atomically to a direct file
  under `status/`, rejects external paths, and does not follow symlinks.
- **Release instructions:** corrected stale engine/MFS test counts.

## Verification record

Logs are retained under `temp/` in the workspace:

- `runner-final.log`: **44/44 pass**, zero blocking failures (30 legacy + 14
  paranoia tests).
- `verify-overall.log`: end-to-end build, canonical suite, MFS, and TUI smoke;
  completed successfully after the editor allocation fixes.
- `verify-mfs.log`: **11 gated pass, 0 fail**, plus five informational checks.
- `gcc-analyzer-final.log`: whole active engine build with `-fanalyzer`; no
  compile errors. GCC reports one retained-undo ownership warning at
  `mv_undo_push`; the allocated line array is stored in the static undo ring
  and released by undo-history cleanup on reload/close. The warning is an
  ownership-analysis false positive, not an unowned allocation.

The 44-case physics run preceded the later editor/render allocation changes;
the final end-to-end run recompiles the engine and reruns the canonical,
robotics, and TUI checks.

## Remaining technical limits

- Current CCD is principally translational. Rotation-only tunneling and
  arbitrary swept angular motion are not solved; custom shapes and cylinders
  still use proxy/conservative paths outside exact special cases.
- GUI and headless simulation still have duplicate step paths. Their shared
  behavior is tested, but this is not a proof of equivalence.
- Broader randomized physics fuzzing, differential checks across multiple
  analytic systems, and thread-sanitizer runs remain outstanding. The new
  fixed-seed matrix property sweep and combined ASan/UBSan profile are recorded
  below.
- Non-built historical artifacts and the `v15R3` source tree (absent from this
  workspace) were not audited.

## Workspace-boundary incidents during the audit

Before the hard-coded fixture paths were discovered, early runner invocations
created fixture outputs outside the project. The runner was stopped when this
was found; no outside files were inspected or cleaned. A stale plugin also
caused two headless suite attempts to try opening `/dev/input/js0`; the path
was absent, the probe was disabled for headless tests, and the MFS bundle build
dependency was corrected. Current verification logs and scratch remain under
the project `temp/` directory.

## Test suite upgrade: 2026-09-25

Counts: 29/29 is the frozen v15R3 tag; the current v15S head is 32/32
(29 physics + 3 diag-informational, canonical case name `module` — the
v1 binary was `module_test`). MFS strafe is emergent roller-anisotropy
(no chassis-force cheat); odometry is pure-encoder with `odom_slip`.

The unified runner in `tools/test_runner.py` now offers `quick`, `physics`, and
`full` profiles. It discovers the canonical C cases from their registry and
isolated cases from Makefile build targets, checks that the C binary's listed
tests match the source registry, and rejects missing, duplicate, malformed, or
failed per-test output. Each run writes JSON and JUnit summaries and command
logs under a unique directory in `temp/qa_runs/`. Eleven Python contract tests
exercise registry discovery, result accounting, MFS summaries, TUI snapshot
validation, command-launch failures, and both report formats.

The canonical `math3_inverse` gate now covers 256 fixed-seed symmetric positive
definite matrices scaled from 2^-24 through 2^24, in addition to analytic,
singular-axis, and non-finite cases. The full profile also runs the 30 isolated
legacy cases, 14 paranoia cases, 11 MFS gated/build checks and five
informational diagnostics, nine TUI snapshot files, an engine rebuild, and
repeats the C, MFS, and TUI checks with AddressSanitizer and
UndefinedBehaviorSanitizer.

The first sanitizer pass exposed missing world cleanup in the legacy
`cylinder_drop` and `driven_wheel` test programs and the MFS
`ftc_integration` fixture. Their success and failure exits now release the
world allocations. The FTC hotload test intentionally loads a plugin image
containing a second `mpe_module_desc`; its invocation disables only ASan's
duplicate-global heuristic. Leak detection, address checks, and UBSan remain
enabled for that test.

Final full-profile run: **220/220 runner checks passed**, with zero blocking
failures and 16 informational results. Its complete report and logs are in
`temp/qa_runs/20260925T142200Z-189691/`. The GTK engine was built but not
launched because this profile is headless; ThreadSanitizer and interactive GUI
runtime coverage remain outstanding.

After that full run, the harness gained a command-launch failure contract; the
quick profile was rerun through `verify.sh` with all 11 Python contracts and
the canonical suite. Its report is in
`temp/qa_runs/20260925T144930Z-203671/`.
TUI output was then parameterized to preserve snapshots per run. The regular
and ASan/UBSan snapshot targets were rerun separately; all nine snapshots in
each profile passed finite-state validation. Those artifacts are in
`temp/qa_runs/tui-artifact-check/`.
