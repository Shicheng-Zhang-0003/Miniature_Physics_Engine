# Version History

- **v15S — 2026-10-20: mathematical, programming, and operational audit.**
  Closed the concurrent-world worker-pool race and narrowphase shared-body
  races; added the prismatic angular lock and exact antipodal recovery;
  corrected cylinder/slab minimal-exit selection, cylinder cap-lens contacts,
  and small parallel-edge support; changed rolling resistance to use
  compression-only support; made MicroVim loads transactional and confined
  unlisted editor targets to direct `status/` files. See the detailed audit
  and current verification record in [`AUDIT_REPORT_2026-10-20.md`](AUDIT_REPORT_2026-10-20.md).

- **v15S — 2026-10-16: stable release candidate — all suites green.**
  Full multicore step pipeline: per-body CCD sweep (disjoint writes, per-pair
  margin replacing the global CCD obstacle margin that perturbed MFS tank /
  drive_directions / odometry_yaw — now unified 30/30 green (2026-10-19 audit)), per-pair narrowphase
  (pre-assigned manifold slots, bitwise identical at 1/4/8 threads), persistent
  pthread thread pool with dynamic load balancing. `legacy-f11_torture` guardrail
  fixed (safety_net is an operational guardrail, not a torture knob). Tank /
  drive_directions baselines rebased on the verified fixed point (heading 3.0639,
  disp 0.0837). Suite 46 registered / 44 blocking green; quick 60/60; all physics
  green under ASan/UBSan; determinism twins bitwise at 1/4/8 threads over 600-tick
  runs. `-Werror` on by default (0 warnings under `-Wall -Wextra -Werror`).
- **v15S — 2026-10-15: comprehensive audit + multicore.**
  Full audit of math, physics, programming, and operations. Island-based
  multicore solver (`MPE_THREADS`): per-island manifolds solved by
  dedicated threads, bitwise identical to serial at 1/4/8 threads
  (verified over 600-tick determinism twins). Damped warm-start
  infrastructure (`solver.warm_start_damping`, default 0.0=cold proven).
  Floor CCD now includes rotational lowest-point velocity (tumbling corners
  clamp before tunneling). S2 Bridge Phase 1: move existing bodies in place
  per refresh tick (`cfg->every > 0`), stable S2-atom → MPE-body mapping,
  zero-force bond lines that follow. `-Werror` on by default (0 warnings
  under `-Wall -Wextra -Werror`). Suite 46 registered / 44 blocking
  green; quick 59/59; determinism twins bitwise at 1/4/8 threads.
- **v15S — 2026-10-14: handling + keyboard drive.**
  Yaw runaway tamed (sustained rotate capped ≈ −4.4 rock-steady, was −13.5
  still creeping: governor + active damper, deadband 3.0–4.5 so green
  gates never see it; gamepad expo x^1.5); keyboard robot drive on
  G V B N C M (M verified free globally; pure mapping + `keydrive_axes`
  gate, pad keeps precedence while connected); FTC footprint into the 18 in
  cube (0.44, was 0.52 — centralized geometry, computed odom arms); aimed
  shots (power + elevation) so hive mouths score from legal spots
  (`shoot_power`, `hive_shot` gates). Suite 46 registered / 44 blocking
  green (floor 45 → 46); quick 60/60; MFS 29/30 (`tank` pre-existing;
  `yaw_stability`, `shoot_power`, `hive_shot` all green).
- **v15S — 2026-10-13: despot sweep (truth + hardening).**
  MGB-MKSTEMPS (P20's `mkstemp` broke every spawn with EINVAL — 32/32
  now); P11 cast recurrence ×4; `mod ls` count guard; v200 joint-commit
  accounting; microvim `:e` whitelist + failure propagation; TUI builder
  guards; solver-clamp + dt-clamp counters; depenetration single-knob
  wake; joint-beta help truth; N1/N2/N3/N4/N5/N6/N7/N9/N10/N11/N12
  honesty notes; runner docstring + makefile NOTE counts; Windows batch
  TMPDIR. Suite 45/43 green; quick 59/59; physics 104/105 (sole red the
  pre-existing `legacy-f11_torture`); tui-smoke PASS; MGB 32/32; MFS
  16/20 gated bit-identical to pristine.
- **v15S — 2026-10-09: diagnostics + S2 bridge.** `core/mpe_diag.h` makes
  every rejection name itself (subsystem, stable code, engine `file:line`,
  runtime values), fanned out to stderr, event log, debug terminal;
  `diag` replays the ring. Distinct return codes for every attach/detach/
  loader rejection — all still `< 0`. Everything modular now lives under
  `ecosystem/` (top-level `plugins/` removed) and the loader jails paths
  there. S2 molecular bridge renders a live petri dish: spawns `s2tui`,
  configures it through the existing shell, and stages strict `S2SAVE1`
  frames as static bodies with zero-force bond lines — display-only and
  stage-once. Suite 45 registered / 43 blocking green (added `diag_naming`,
  floor 44 → 45); runner quick 59/59; runner full 226 checks with 3 known
  pre-existing reds documented above.
- **v15S (through 2026-10-07)** — GTK4 port, module system (MPI hot-plug),
  per-world config, data-structure upgrades (growable pools, O(1) caches),
  kernel global-state removal, TUI stress suite (`stress`/`ccd` scenes,
  backend flags), 44 registered / 42 blocking green plus a 234-check
  verification profile (232 pass + 2 xfail + 6 info) green under ASan+
  UBSan; DESPOT audits 2026-10-01 through 2026-10-07 closed numerous
  findings (floor raised 42→44, F11 torture-leak recurrence closed, every
  physics formula re-verified against textbook statements, `[CLAMP-TAUTOLOGY]`
  closed, `[FRICTION-THRESH]` root-caused and fixed, regime matrix revived,
  FTC streamlined to one field / one robot / one controller).
**2026-10-20 (v15S, post-first-audit second pass):** close-out fixes across
core, physics, GUI parity, UI, scene loaders, plugin unload, MFS/MGB kits,
and the makefile operational gates: per-file markers `DESPOT-2026-10-20`;
V-01/V-02 gates green again after the `GTK_DEFINES`/`$(HARD_LIBS)`
hard-append fix; ledger addendum:
[`AUDIT_REPORT_2026-10-20.md`](AUDIT_REPORT_2026-10-20.md) "Second-pass
fixes". `MPE_THREADS` 1/4/8 twins re-verified md5-identical.

- **v15R3 (release)** — configuration system, physics-truth pass, full
  constraint framework, TUI debugger + snapshot suite, 29/29 headless green.
  Release notes: [`release_notes_v15R3.md`](../release_notes_v15R3.md).
- **v15R2** — config-system hardening + MFS robotics (prior RC, parked now
  consolidated in `v15S/src/ecosystem/mfs/`).
- **v1.4 Alpha RC3** — domain-driven restructure, spatial-hash broadphase,
  physics-world encapsulation.
- **v1.4 Alpha 2** — warm-starting solver, multi-point contact manifolds.
- **v1.4 Alpha RC1** — spring joints, joint renderer, color painting, OBB
  raycast selection.
- **v1.3** — established instanced rendering and spatial-hash direction.

See [`v15S/evolution.txt`](../v15S/evolution.txt) for the full lineage back
to stage 0.
