# MFS v10S — Release Notes (1.0-Stable)

> **Historical context:** MFS ships as one source tree twice — this
> standalone repository and the embedded twin at
> `475-MPE/v16R1/src/ecosystem/mfs/` — kept source-identical by
> `sync_mfs_check.sh` under `docs/SYNC_CONTRACT.md` (canonical direction:
> fixes land in 475 first, mirrors follow). `V1.0R3-Interim` (tagged
> 2026-10-05) was the working interim; v10S is the first **Stable**
> designation of the MFS train.
>
> **Validation note:** every figure below is a measured result from the
> 2026-10-25 session on the renamed tree (`475-MPE`'s v15S → v16R1 cutover
> happened the same day), captured on both trees to prove the twins behave
> identically. Nothing quoted here is inherited assertion.

- **Status:** Stable release / V1.0 baseline
- **Scope:** FTC-fleet robotics stack, BioBuzz gameplay subsystems, aim
  tooling, the twin sync contract, documentation, and the unified
  verification harness

## Overview

MFS is the robotics/game-simulation ecosystem of the Miniature Physics
Engine: a hand-written C stack that plugs into the engine's kernel as a
hot-loadable module bundle. It provides two robots (the FTC-style fleet
robot and the BioBuzz park/arcade robot), a physical piece economy
(pollen/nectar/balls with aero and Magnus), an aim solver that solves the
real flight, a per-module runtime config menu, and a matched verification
harness. Everything the ecosystem knows about its own history — every
despot-tagged fix, every failed hypothesis — lives in
`docs/KNOWN_FAILURES.md`; this release note states what is true at the tag.

## 1. The twin train

The tree exists twice on purpose: the standalone repo is the clean
reference for MFS-only work, the embedded twin runs against the full
engine. The guard compares `.c/.h/.md/.sh/.mk` source bytes (artifacts,
build dirs, repo metadata: `.gitignore`, `LICENSE`, the per-repo
`readme.md` landing pages, and the guard script itself are excluded) and
fails loudly with `exit 2` when it cannot find a twin, `exit 1` on drift.

Verified at this tag: `in sync (source-identical, artifacts ignored)` from
both roots. The `readme.md` exclusion is a contract decision dated
2026-10-25: the standalone README became a GitHub landing page while the
embedded twin keeps the module-lifecycle doc; both are per-repo objects.

## 2. The robotics stack (ftc-fleet)

- **Drive:** canonical mecanum mixer (single source of truth), the
  traction governor with impulse closure, wheel/robot front-truth labels,
  odometer overflow hold, and the FTC rotate convention in `drivetrain_mecanum`.
- **Electrical model:** 10-cell NiMH pack (~0.06 Ω, 3.0 Ah) with sag,
  fuse-heat accumulation, and brownout envelope on the 20 A fuse; motor
  observers with free-speed/stall endpoints and anti-saturation logic.
- **Contact realism:** per-pair CCD with resting-gate effects (documented
  baseline effects in `docs/KNOWN_FAILURES.md`), revolute-joint rollers on
  mecanum wheels, and bundle-internal hot-load attach through the engine
  tick table.
- **The shooter flywheel is a real body** on a revolute pivot — it scrapes
  down when the disc rim grinds a perimeter wall while assisted spin-up
  gates torque at ready (behavior diagnosed 2026-10-25, recovery is a
  spin-up toggle; remediation options are noted, not implemented).

## 3. BioBuzz gameplay subsystems

- **Intake:** multi-piece carry (2×2 grid, mixed pollen/nectar sizing),
  whole-sphere aperture-fit capture with a 1.5 m/s relative-speed gate,
  possession validation, and state-set ejection.
- **Aim tooling:** `ftc_aim_params_solve` — deterministic grid first-hit at
  the steepest settling elevation, mini-world entry-arc march that mirrors
  live double-aero flight, guilded pose cache, gamepad auto-aim throttle.
- **Match processing:** one field, four robots, preloads, staged nectar
  seats, flower/garden/table scoring in `docs/FTC_SPECS.md` order.
- **Runtime config:** per-module key-5 menu, four live setters (Shooter
  launch elevation / Launch speed / Launch backspin / Launch power floor),
  default elevation 65°.

## 4. Known open items (honesty section)

- `aim_solve` registers **1 red gate on both trees** (32 total, 31 pass):
  the solved rest lands on the hive roof (`local (0.000, 1.109, -0.841)`
  — the pre-`fbb6d66` trace), suspected to be re-opened by release-day
  churn. Failing identically on both trees is proof the mirror is
  faithful; the fix lands in 475 first per the canonical-direction
  contract, then mirrors.
- Flywheel wall-grind recovery and the intake velocity-gate strictness
  ("takes a few sojourns" when a piece is compressed against the wall)
  are documented engine-faithful behaviors, awaiting an implementation
  decision (holding torque at ready / widened gate), not regressions.

## 5. Harness and verification

The unified suite builds and runs through `build_tests.sh` (dual-mode:
embedded `$MFS/../../core` exists, standalone discovers the engine via
`MFS_ENGINE_SRC`, else sibling `../475-MPE/v16R1/src`): suite **32 total,
31/32** (one aim_solve gate, section 4), FTC gated **pass=4 fail=1**, 12
informational diagnostics ungated (teleop/mechanum/odometry/integration/
hotload/idle-spin rigs), plus the embedded engine quick profile **60/60**
including the engine-version contract gate. Windows runs through MSYS2
(MSYS2 bash produces native artifacts). Embedded run: `cd v16R1/src &&
ecosystem/mfs/build_tests.sh`; standalone run: `./build_tests.sh` from
this root with the engine discovered on the sibling.

## 6. Versioning

The bundle descriptors now carry the release: `.version = "1.0-Stable"`
in `mfs_ecosystem.c` (ecosystem desc), `ftc_module.c` (fleet module desc),
and the `mfs_module_1.c` module desc via `MFS_MODULE_1_VERSION` — plus the
suite's own sample descriptor in `tests/mfs_suite_c.c`. Tags: `V1.0-Stable`
in this standalone repo (next to predecessor `V1.0R3-Interim`),
`mfs-V1.0-Stable` in the 475 engine repo (which already carries its own
lowercase `v1.0-Stable` engine-era tag — the scope prefix avoids that
collision).
