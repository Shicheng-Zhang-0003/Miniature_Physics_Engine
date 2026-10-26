# DESPOT Audit 2026-10-13 — full mathematical, programming, operational sweep

> Scope: the entire 475-MPE tree (active `v15S` engine + `tools/` + docs), with
> cross-reads of the sibling twins where they are authoritative: standalone
> `461-MFS` vs vendored `v15S/src/ecosystem/mfs`, and `4179-Magi/472-MGB` vs
> vendored `v15S/src/ecosystem/mgb`. No millennium prize was required; every
> lie below was measured against a textbook, a header contract, a run log, or
> a pristine-`HEAD` worktree (`git stash` A/B — the despot's own regression
> test for every behavioral change).

## Method

Three independent passes, all read-not-assumed, plus a fourth the previous
audit did not run:

- **Math:** every formula in `core/` + `physics/` re-derived against Mirtich,
  Baraff, Catto GDC 2005/2011/2013/2014, Gottschalk 1996, Coumans 2005,
  Ericson RTCD, Goldstein mechanics, Press numerical recipes. Oracles checked
  for circularity (does the expected value restate the code?).
- **Programming:** full-tree C read (core/physics/scene/config/render/ui_input/
  tui/tests/ecosystem), `makefile`, `tools/test_runner.py`, `build_tests.sh`,
  `run_all.sh`/`verify.sh`, `build_windows.bat`. Git history mined for
  recurrence (torture-leak, regime-wipe, SKIP-sentinel, count-drift, P11 cast
  class).
- **Operational:** every headline count re-derived from its cited source
  (registry vs `MIN_SUITE_ENTRIES` vs readme vs gates vs makefile NOTE;
  MFS summary vs rows; TUI scenes vs smoke; tunables vs schema; version macro).
- **Regression A/B (new):** every behavior-changing fix was verified against
  a pristine `HEAD` stash (`MFS tank`, `paranoia_config` SIGSEGV, MGB spawn
  naming, ASan loader). A fix that moves a baseline must prove the baseline
  moved for the documented reason — never by assertion.

Verification after fixes: `build_suite` + `engine` + `mpe-tui` warning-free,
`--profile quick` **59/59**, `--profile physics` 104/105 with the sole red
the documented pre-existing `legacy-f11_torture`, `tui-smoke` PASS,
`mgb_mpe_test` **32/32** (was 30/2 — see MGB-MKSTEMPS), MFS `16/20` gated
bit-identical to pristine (see MFS-TANK). All scratch under `temp/`
(`temp/despot-20261013/`); no directory outside the project tree was written
except the pre-approved `/tmp/opencode/` scratch (one manual TUI-adjacent
probe, since removed).

## Headline: the despot's own fix was broken (MGB-MKSTEMPS) — FIXED

The DESPOT-2026-10-12 P20 `mkstemp` hardening called `mkstemp()` on the
template `"%s/mgb_transcript_%d_XXXXXX.log"`. `mkstemp` requires `XXXXXX`
**last**; a trailing `.log` makes *every* call fail with `EINVAL`. Measured:
a scratch probe against the shipped objects returned `reason=8 (e_alloc),
errno=0` without ever reaching `fork` (strace: 4 pipes, clone, child
`execve → ENOEXEC → _exit`, parent never `openat`s the transcript — the
`mkstemp` never runs). Consequence: **no S2 bridge spawn could ever succeed**,
and the failure was misreported as `e_alloc` (out of memory) because the
`mkstemp`-failure branch calls `fail_spawn(mgb_spawn_e_alloc, 0)`,
discarding `errno`. The two spawn-naming gates (`noexec`/`ENOEXEC`) that
existed precisely to catch this failed 30/2 — and `mgb_mpe_test` is not part
of any runner profile, so nothing was red in CI.

Fixed: `mkstemps(tplt, 4)` (suffix-aware, keeps the `.log` suffix honestly;
`_GNU_SOURCE` guarded), `ENAMETOOLONG` on the overlong-template branch
instead of errno-0 `e_alloc`, and the real `errno` preserved on `mkstemp`
failure. Result: `mgb-mpe: PASS 32 FAIL 0`. The standing lesson from 10-03
("a floor below the real count is worse than no floor") has a sibling:
**a security fix whose own failure mode is untested breaks the thing it
secures, silently.** The spawn-naming gates are that test — they now run.

Forward-port: `472-MGB` still carries the pre-P20 predictable-pid code
(`mgb_transcript_<pid>.log` via `snprintf`+`fopen`) and additionally needs
the `mkstemps` form, not the `mkstemp` form. Tracked, not committed here
(separate repo); patch sketched in `REMAINING_WORK.md`.

## Mathematical lies found and dispositioned

| ID | Lie | Severity | Disposition |
|---|---|---|---|
| N1 | "Implicit-midpoint" gyro is explicit in the gyro term (`rigidbody.c`): `w_mid=w+α_ext·dt/2` advances by external torque only; true implicit midpoint solves `w'=w+dt·f((w+w')/2)` (Press). For torque-free `w_mid==w` it is explicit Euler (already admitted) | medium | **FIXED wording** (explicit-midpoint/RK2-split, not implicit) at site |
| N2 | Gyro clamp `4·\|w\|/dt` "never binds physically" (`rigidbody.c:930`): `\|α_gyro\|~|w|²ΔI/I_small`, so a needle at high `\|w\|` exceeds it (e.g. 1e5 vs 2.4e4) and is truncated with no counter | medium | **BOUNDED** at site (extreme-regime exception, benign plant measured 0.0000%) |
| N3 | L-fixed-point tolerance `1e-4·(\|L\|+1)` scales with momentum, not the iterated `\|Δw\|/\|w\|` — heavy/slow pass early, light/fast late; silent, ungated | low | **DOCUMENTED** at site |
| N4 | Clip second pick degenerate (`collision_narrowphase.c`): with `keep_count==1`, centroid==keep[0], `e2==0`, every candidate scores area 0 — first-untaken wins, picks 3-4 maximize against an arbitrary pair | medium | **DOCUMENTED** at site; farthest-point reseed ticketed |
| N5 | Cyl-vs-slab shortcut always ejects out the near face (`collision_cylinder.c`): embedded cylinder takes the long exit, violating Gottschalk/Ericson minimal-exit | medium | **DOCUMENTED** at site (resting/impact regime unaffected) |
| N6 | Cyl-cyl second point copies depth unchecked (`collision_cylinder.c:767`): laterally-offset near-coaxial `mid+perp·min_r·0.5` can lie outside the smaller cap — phantom support at full depth | low | **DOCUMENTED** at site |
| N7 | Rolling N read post-Poisson (`collision_solver.c`): bouncing contacts draw rolling torque from restitution impulse (should be compression N only); plus the 0.5mm `pen_eff` floor manufactures overlap for slop zero-depth contacts | medium/low | **DOCUMENTED** at both sites; per-phase N ticketed |
| N8 | Depenetration post-move wake used literal `0.01` while pre-wake gates on `wake_depth_thresh` (default 0.02) — knob lie | low-medium | **FIXED**: one knob, one meaning (`depenetration.c:161,165`) |
| N9 | `joints.revolute_beta` drives 5 joint types while axis/fixed drift use hard `0.1` — shared-key naming wart + undisclosed surface | low | **DOCUMENTED** at both `0.1` sites; schema help rewritten to shared-bias truth |
| N10 | CCD "overshoot ≤0.5·g·toi² < slop" holds only at canonical dt (1.4mm); at clamped dt=0.1 + torture g≈50 it is 0.25m ≫ slop | low (corner) | **DOCUMENTED** at site |
| N11 | `paranoia_contact_solver` T3 reads `v_start` from the engine + engine's own μ (circular within 15%); T5 rolling band 5–45m admits 5× error (smoke, not law) | medium/low | **FIXED halfway**: g pinned to CODATA `g_n=9.80665`; residual μ-circularity disclosed at site; independent form cited (`docs/VALIDATION.md`) |
| N12 | Small-box parallel-edge support collapses to a point (`collision_narrowphase.c:522`): `contact_spread>0.05` gate denies 2nd contact to boxes ≤0.1 — scale-dependent tipping, silent | low | **DOCUMENTED** at site |
| N13 | Split-impulse runtime clamps (slop→[0,0.05], beta→[0,1], bias→≤10) rewrote config silently (comment-only disclosure) | low | **FIXED**: counted via `mpe_clamp_solver_events` + bounded stderr, same discipline as input clamps |
| N14 | `dt>0.1→0.1` step clamp silent (twins with different lag diverge) | low | **FIXED**: counted via `mpe_clamp_solver_events` |

Verified correct, no change: semi-implicit Euler + post-scale damping
(disclosed O(dt²)); exp-map rotor + renormalise; all three inertia tensors;
15-axis SAT + slop admission; effective mass cyclic form; Poisson + Newton
bound; translational deepest-only split (deliberate, measured); q-form sphere
TOI + floor double quadratic + slab sweep; `min(μa,μb)` (disclosed 42% under
Box2D √); cyl-sphere SDF inside-normal sign; segment-OBB ternary
(Boyd-convex); segment-segment Ericson block.

## Programming weaknesses found and dispositioned

| ID | Weakness | Severity | Disposition |
|---|---|---|---|
| N1 | `sed s/nice/` single-target path raw `(int)` cast (`term_admin.c:277`); all-targets path already safe — P11 recurrence in the adjacent branch | high (UB) | **FIXED**: shared `term_float_to_int_safe` |
| N2 | `top -n` raw cast (`term_sys.c:55`) — finite 1e30 overflows before the 1..16 clamp | high (UB) | **FIXED** |
| N3 | `xxd` raw casts + no `<0` guard (`term_query.c:255-268`): `-s -100` reads before `bodies[idx]`; negative length over-reads into the next body | high (OOB/info-leak) | **FIXED**: safe narrow, reject `offset<0`, clamp length |
| N4 | `head/tail -n` raw casts (`term_query.c:398,430`) — UB before clamp, no upper cap | high (UB) | **FIXED** (cap 1e6) |
| N5 | `mod ls` / attach-dupe display loops trust corrupt count (`term_mod.c:62,111`) — P1 recurrence at the display path | high (OOB) | **FIXED**: E_COUNT-style refuse before indexing |
| N6 | v200 commit silently drops joints (`scene_load.c`): combined veto excluded springs; spring/fixed/distance/rope returns ignored (only rev/prism checked, still silent) | high (silent loss) | **FIXED**: per-pool veto (springs vs own cap), every return counted loudly + revision bump |
| N7 | microvim `:e` bypasses whitelist + ignores load failure (`microvim.c:648`): arbitrary-file read; failed load cleared buffer + undo, `:w` wrote foreign content over the old file; also passed a literal `!` into the path on force edit | med-high | **FIXED**: whitelist gate, `!` stripped via local copy (also closes a `-Wrestrict` overlap: `cmd` aliases `mv.command_buf`), failure keeps filename + undo + loud `E492` |
| N8 | `vi` extension-gate allows stray file creation in CWD (`term_admin.c:849`) | med-high | **DOCUMENTED** (confine-to-`status/` ticketed; realpath+O_NOFOLLOW at save is the full fix, a behavior change not snuck in) |
| N9 | `scene_allocate_object_id` hands out `0xFFFFFFFF` sentinel (`scene_init.c:31`) — aliases floor/cache via legacy load | med-high | **FIXED**: skip 0 and `0xFFFFFFFF` both directions |
| N10 | Editor dialog NaN/Inf (`editor_dialog.c:37`, `editor.c:149`): overflow `HUGE_VALF` passes strtof; NaN passes both range checks then `(int)NaN` | med-high (UB) | **FIXED**: `isfinite` at dialog accept + iters path (`<math.h>` added) |
| N11 | TUI scene builders index unchecked `add_*` (`tui_main.c` stress/ccd/springlab/demo/pendulum) — `bodies[-1]` OOB (f10 already guarded) | med-high | **FIXED**: `TUI_BUILDER_CHECK` aborts scene loudly at every deref site |
| N12 | `ftc_traj.c:249` uint32→int narrowing — IDs past `INT_MAX` stall prediction forever via `pred_id<0` | medium | **FIXED**: range-validate before narrowing (plugin ABI takes `const int*`, unchanged); loud stderr + abort instead of silent stall |
| N13 | `term_id_buffer` shared static scratch (`term_fs.c`) — P6/P7-shaped race | medium | **FIXED**: per-invocation heap buffer sized by argc |
| N14 | Parent-dir fsync follows symlinks (`mpe_config.c:314`, `scene_saving.c:69`) | medium | **FIXED**: `O_DIRECTORY\|O_NOFOLLOW` (Windows-guarded in config path) |
| N15 | `paranoia_config` bypasses runner TMPDIR (`paranoia_config.c:53`) — shared-`temp/` race across parallel runs | medium | **FIXED**: honor `TMPDIR`/`TMP`/`TEMP`, fallback legacy path |
| N16 | MGB re-stage leaves stale constraints (`mgb_sink_mpe.c:145` resets springs only) — second stage keeps hinges over recycled ids | medium | **FIXED**: reset both pools |
| N17 | `mpe_parallel_sanitize` `atoi` + uninit `tids` on break (`physics_world.c:42`) | medium | **FIXED**: `strtol` + clamp, zeroed arrays (join loop already launch-guarded) |
| N18 | `mpe_diag_total` narrowing (`mpe_diag.c:180`) + `config_menu` int↔float precision | low | **FIXED** (saturate at `INT_MAX`; menu noted) |
| N19 | MGB transcript truncation (`mgb_link.c:259`): 300B template into 256B field — silent detach of unlink path (new site of the P20 class, found while fixing MGB-MKSTEMPS) | medium | **FIXED**: refuse with unlink + `e_alloc` instead of truncating |
| N20 | `mkstemp`-failure branches discard `errno` (`fail_spawn(e_alloc, 0)`) — the exact misdiagnosis MGB-MKSTEMPS tripped over | low | **FIXED**: preserve `errno`; `ENAMETOOLONG` on the overlong-template branch |

Recurrence note: P11 cast class re-opened in 4 new sites (closed again —
the helper was `static` in one TU, so sibling TUs could not use it; it is now
shared via `term_priv.h`/`debug_terminal.c`). Torture-leak class stays
closed. Regime-wipe (P12 pins) confirmed intentional, not recurrence. SKIP
class stays closed. Count-drift re-opened once (runner docstring 44 vs floor
45; makefile NOTE 44/42) — closed again.

Self-inflicted red caught mid-audit: the `paranoia_config` TMPDIR edit used
`getenv` without `<stdlib.h>` (implicit-int return → pointer truncation →
SIGSEGV, exit −11). Caught by the physics profile, fixed with the include,
verified (`test_paranoia_config` exit 0 + isolation proven: no shared-`temp/`
write with `TMPDIR` set). The profile is the test for the test-writer.

## Operational findings

- **Runner floors hold**: 45/30/14 discovered == enforced. Docstring + makefile
  NOTE corrected to 45/43 (were 44/42 — the drift class again).
- **MFS twins still drifted** (`sync_mfs_check.sh` exit 1, ~40 source files):
  475-only `intake.c`, V-line + `mpe_diag`/`event_log` CORE delta, errexit
  scoping, KNOWN_FAILURES/MODELS/TESTING/straffe-sign docs. Unifying the twins
  remains a separate-repo task; the 475 tree is the authority for 475 builds.
- **MFS `tank` + `odometry_yaw` red are pre-existing**: pristine-`HEAD` stash
  A/B gives bit-identical `gated pass=16 fail=1` (tank pivot baselines +
  tank odom yaw; `drive_directions` passes — the CCD-margin signature from
  the 10-09 notes, not this audit's depenetration change).
- **ASan suite 41/43 pre-existing**: `loader_lifecycle` capsule `.so` load
  fails identically on pristine `HEAD` (host-ASan vs uninstrumented `.so`
  interposition class, same family as the documented `ftc_hotload`
  duplicate-global heuristic). Ungated in quick/physics; ticketed.
- **`build_windows.bat` parity**: `TMPDIR`/`TEMP`/`TMP` now exported at
  `temp\` (was unset — the Windows path had the same shared-scratch race);
  profile stays `quick` (full needs MSYS2 ASan/UBSan + MFS parity — tracked).
- **MFS gate count**: runner docstring corrected 15 → 20 gated.

## Verification record (this audit)

- `make build_suite` + `make engine` + `make mpe-tui`: clean, zero warnings
  (one `-Wrestrict` introduced mid-audit by the `:e` fix was caught by the
  build and fixed with a local copy).
- `python3 tools/test_runner.py --profile quick`: **59/59**, 0 blocking.
- `--profile physics`: **104/105** — sole red `legacy-f11_torture` (exit 1),
  the documented pre-existing extreme-parameter case.
- `make tui-smoke`: PASS on all scenes with the new builder guards.
- `make -C ecosystem/mgb test`: **32/32** (was 30/2).
- MFS `build_tests.sh`: `gated pass=16 fail=1` — verified identical on
  pristine `HEAD` (stash A/B).
- ASan suite: 41/43 — verified identical on pristine `HEAD` (pre-existing
  loader/`.so` class).

## Remaining open (bounded, not solved)

N4 farthest-point reseed; N7 per-phase friction re-snapshot (+ rolling
compression-N split); normal/tangent warm-start carry (dead by measurement);
full Gottschalk depth numeric agreement; oblique multi-body L oracle;
TSan on a capable host; MSVC `/fp:strict` proof; multithreaded solve (serial
by design); absolute-path jail + `O_NOFOLLOW` loader open; joint distinct
codes; wider state hash; `MATH_G_N` pin (N11 halfway: paranoia T3 now pins
`g_n`, canonical oracles still mix); drag analytic oracle; angmom vector
gate; ellipse closest-point; `WARNINGS_AS_ERRORS` CI; `vi` CWD-confinement
(N8); failed-`:e` buffer preservation (now loud + filename/undo-safe, content
still reset by the callee — needs staged load for full preservation);
472 P20+mkstemps forward-port. Each is ticketed above or in REMAINING_WORK.md
with the measurement that bounds it.
