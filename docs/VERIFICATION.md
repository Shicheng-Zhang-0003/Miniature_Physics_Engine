# Verification Suite

The unified test runner provides `quick`, `physics`, and `full` profiles:

```bash
python3 tools/test_runner.py --profile quick   # runner contracts + canonical C suite
python3 tools/test_runner.py --profile physics # also isolated legacy + paranoia cases
python3 tools/test_runner.py --profile full    # full build, physics, MFS, TUI, sanitizers
python3 tools/test_runner.py --list            # discover registered test cases
```

The canonical C suite runs all cases in one process with shared audited
builders (exact-name dispatch, saved/restored config, real Coulomb floors
everywhere). The full profile also runs each older C case in a separate
process to catch cross-test contamination. The runner verifies that reported
case names exactly match the C registry, distinguishes informational
diagnostics from gates, and fails on missing, duplicate, or malformed test
output. Its Python contract tests cover registry discovery, result parsing,
report generation, TUI snapshot validation, and command-launch failures.

### 2026-10-20 second-pass verification (post-fix rounds)

- `--profile quick` (60 checks) green immediately after each fix batch; the
  full `--profile full` run was launched detached and is recorded in
  `temp/despot-2026-10-20/full_profile_detached.log` (summary.json lands in
  `temp/qa_runs/` per the runner contract).
  NOTE on cited `temp/qa_runs/<timestamped-id>` paths anywhere in these
  docs: since 2026-10-24 the runner retains only the newest 10 run
  directories (`MPE_KEEP_QA_RUNS`, default 10) so full-profile runs do not
  grow without bound; cited paths are evidence from the date of writing and
  may have been pruned on disk since. The same applies to
  `temp/despot-2026-10-20/`, which is explicitly retained as the audit's
  own artifact directory.
- `validation/V01.sh` (ASan+UBSan build) and `V02.sh` (clean rebuild, zero
  warnings) now PASS after the makefile define/link hard-append fix
  (previously command-line CFLAGS silently built the removed GTK3 branch).
- External math oracles (10/10) green; the closed-form drag+gravity
  reduction matches the exact ODE solution to 1e-16 (grid in
  `temp/despot-2026-10-20/math_oracle.log`).
- `MPE_THREADS` 1/4/8 `test_paranoia_determinism` logs md5-identical.

| Test | Proves |
|---|---|
| `meta_rotation` | Yaw rotation equivariance to 5e-07 |
| `meta_convergence` | Solver error is bounded and does not diverge with iteration count |
| `meta_config_wiring` | Config parameters demonstrably reach the simulation |
| `sleep_settle` | Launch settles then sleeps-iff-enabled |
| `mouse_look_axes` | Mouse-look sign convention on 4 axes + diagonals |
| `keydrive_axes` | Keyboard-drive convention (G V B N C M) |
| `body_materials_live` | Bodies stamped with live materials |
| `two_world` | Independent `physics_world` instances |
| `revolute` | Hinge joints hold anchor and allow swing |
| `cylinder_drop` | Cylinder settles on the floor |
| `driven_wheel` | Torque → friction → translation |
| `math3_inverse` | Matrix inverse at small inertia tensors |
| `floor_collision_diag` | Floor contact diagnostics |
| `cylinder_sphere/cube/cylinder` | Cylinder narrowphase pairs |
| `scene_roundtrip` | Save/load v200 round-trip |
| `stack` | 6-cube tower stands |
| `f10_long_run` | 60-second long-run stability |
| `sleep_contact_wake` | Slow pushers wake sleepers |
| `f11_torture` | Deterministic config extremes without corruption |
| `module` | Per-world config, registry dispatch, custom shapes |
| `loader_lifecycle` | Plugin load/busy-unload/purge |
| `ftc_ecosystem` | Bundle load → attach → spawn/drive → detach |
| `diag_naming` | Every rejection names its source/code/site |

Run the focused canonical suite with `--profile quick`; use `--profile full`
for all registered suites and sanitizers.

Static and numerical checks used for the 2026-10-20 audit:

```bash
cd v15S/src
make analyze
cd ../..
python3 v15S/validation/property_sweep.py
cd v15S/src
make tsan                 # Linux launcher uses setarch -R by default
```

The audit report records the final run counts, focused regressions, and
remaining bounded limitations: [`AUDIT_REPORT_2026-10-20.md`](AUDIT_REPORT_2026-10-20.md).

External validation against non-project authorities is documented in [`VALIDATION.md`](VALIDATION.md).
