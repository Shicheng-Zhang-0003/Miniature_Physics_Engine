# MPE Glossary

> Every project gets its own vocabulary; MPE's is unusually dense because
> the audit culture and the robotics ecosystem coined many terms. One line
> each; deeper links after the terms. Sorted alphabetically within groups.

## Engine & physics

| Term | Meaning |
|---|---|
| **MPE** | Miniature Physics Engine — the hand-written C rigid-body engine + renderer in this repo. |
| **world** | `physics_world*` — one self-owned simulation: bodies, manifolds, config binding. The kernel holds **zero** global game-sim state. |
| **canonical dt** | The fixed timestep all guarantees are stated for: `1/60 s`. |
| **island** | A connected component of bodies joined by contacts/constraints; solver sweeps parallelize **across** islands, never within one (order preserved). |
| **pair handler** | Registered shape-vs-shape collision function (e.g. `capsule-sphere`); may run concurrently under `MPE_THREADS > 1`. |
| **stage backend** | The currently active broadphase or solver *implementation* on a world (built-in names: `"hash"` broadphase, `"seq-impulse"` solver). |
| **tick module** | A per-world pre/post-step hook from a loaded module ("forcefield" etc.). Enclosed per-tick in a snapshot table (load/unload only at tick boundaries). |
| **slop** | Allowed residual penetration band; a penetration in the band is treated as friction-only contact (no restitution), never "fought" away. |
| **CCD / TOI** | Continuous collision detection; the swept time-of-impact `t_hit` at which a fast body first touches. Current rule: translation-only for volume pairs; rotational lowest-point handling for the floor plane only. |
| **Poisson restitution** | The restitution solve strategy: `e`-scaled impulse pair (approach/compression) rather than reversing velocities. |
| **Catto split / split impulse** | Separating position correction from velocity solving so penetration stays coupled to contact velocity, not repeated into it. MPE uses whole-body translation only (per-contact split tried twice and measured worse). |
| **cold solver / warm start** | MPE starts every contact impulse fresh (cold). Warm start attempts regressed badly and remain off (measured, not stylish). |
| **regime matrix** | The suite re-run set (default / light / heavy / brittle / sticky contact regimes) — a warm stack in one regime is no promise for the others. |
| **twin run / bitwise twins** | Two engine runs whose entire recording matches bit-for-bit — the determinism gate (`MPE_THREADS = 1/4/8`). |

## Determinism

| Term | Meaning |
|---|---|
| **det_math.h** | The deterministic-transcendental contract replacement for libm within the tick; documented error bounds per function. |
| **libm fallback counter** | Atomic counters (`det_fallback_pow_count`, `det_fallback_trig_count`) that count every out-of-contract transcendental call — so a cross-platform desync can never happen silently. |
| **FP-state pinning** | `det_pin_fp_state()` at world init: round-to-nearest, x86 `MXCSR` / ARM `FPCR` denormal flush flags pinned. |

## Modules & ecosystem

| Term | Meaning |
|---|---|
| **MPI** | MPE Module Interface — the stable C ABI in `core/mpe_module.h` (`MPE_MODULE_ABI 1`). |
| **ecosystem / member** | The jail-root for live-loadable modules `ecosystem/<member>/`: `capsule`, `mfs`, `mgb` (+ your own, e.g. the tutorial's `flow`). |
| **ecosystem bundle** | A `.so` that exports several modules' descriptors at once (e.g. `mfs_ecosystem.so`); its inner objects take precedence over plain `.so`s of the same name. |
| **hot-load / hot-load cycle** | `mod load → attach → tick → detach → unload` with the suite green both sides (the loader must never leave a dangling pointer behind). |
| **`E_BUSY` (-2) / `-3 stale`** | Loader protections: unload refuses while a world can still call into the image; remount refuses changed on-disk files rather than silently running the old image. |
| **MFS** | The robotics “miniature fleet simulator” ecosystem member: mecanum drivetrains, motor models, gamepad shell verbs, BioBuzz matches. Lives in `ecosystem/mfs/`. |
| **FTC** | FIRST Tech Challenge—style robotics; the MFS *purpose*, not a sponsor. See `v15S/FTC.md`. |
| **mecanum (analytic mode)** | The default drivetrain mode: no roller bodies are built; roller geometry is computed analytically. The fully articulated roller model (8 bodies × 4 wheels on free bearings) is kept for forensics and does not converge in this GS solver (measured). |
| **BioBuzz** | The built-in regulation league: robots intake & launch pieces at a hive. See `v15S/FTC.md` and `ftc/biobuzz.c`. |
| **hive / up-cell / mouth** | The match target structure; its mouth is divided into staged 3-deep “nectar seats”. An *up-cell* is one mouth cell in its tilted local frame. |
| **entering band / march** | An arc is “entering” when the ball ends the 150-tick march settled inside the up-cell window; the *march* replays a candidate trajectory through a throwaway mirrored world with real aero. |
| **aim solve** | `ftc_aim_params_solve`: deterministic grid + march, first hit at the lowest elevation row; publishes `(power, elevation)` and caches by pose. |
| **MGB / S2** | The S2 molecular bridge (registry name `s2-bridge`): display-only one-way S2→MPE rendering of a live `s2tui` child process. |
| **shell verb** | A `mpe-tui` shell command (e.g. `ftc spawn`, `ftc aim`, `mod load`) — the headless “remote control” of the engine. |

## Verification & audits

| Term | Meaning |
|---|---|
| **profiles (quick / physics / full)** | The test-runner scopes: contracts + canonical suite; + legacy & paranoia bins; + sanitizers, MFS, TUI, build gates. |
| **canonical suite** | The C regression suite (`v15S/src/tests/mpe_suite_main.c`): registry floors prevent entries silently disappearing (currently 46 entries, 2 informational). |
| **registry floor** | The minimum entry-count contract in the runner (`MIN_SUITE_ENTRIES` etc.) — deleting a test fails verification rather than quietly greening it. |
| **QA run artifacts** | Timestamped per-run logs/JSON/JUnit under `temp/qa_runs/<UTC-stamp>/` (newest 10 kept). |
| **paranoia suites** | The adversarial suite family (`tests/paranoia_*.c`): FP edges, integration corners, UAF/lifetime traps, determinism. |
| **[SKIP] contract** | Self-skipped coverage fails the run unless `--allow-skip` — otherwise suites rot into skipping everything that is inconvenient. |
| **DESPOT** | The *external* audit practice: an independent outside reviewer that returns dated findings ("DESPOT-YYYY-MM-DD") which are either fixed or mounted. The dates in code comments are those verdicts. See `v15S/docs/DESPOT_AUDIT_*.md`. |
| **audit tag format** | `DESPOT-YYYY-MM-DD` (external audit round) · `MPE_*NNN(S)` / `MFS_NNNx` (project internal work items) · `FIX-AUDIT-…` (a fix landing an audit finding).   e.g. `DESPOT-2026-10-20 CCD-TOI GRAVITY` marks the fix era of that finding. |
| **twin repos (461-MFS / 472-MGB)** | The sibling checkouts the ecosystem was born independently in; the sync contract keeps shared files identical (`tools/sync_mfs_check.sh`). |
| **LIE-## / P#-N** | Audit-severity tags from audit rounds (e.g. `DESPOT-2026-10-07 LIE-06`: a documented claim that measured false; `P1-2`: priority-1 finding #2). |

## Reading order

1. [`readme.md`](../readme.md) → 2. [`docs/QUICKSTART.md`](QUICKSTART.md) →
3. [`docs/OVERVIEW.md`](OVERVIEW.md) → 4. this glossary →
5. [`v15S/ARCHITECTURE.md`](../v15S/ARCHITECTURE.md) →
6. pick your depth: [`v15S/PHYSICS.md`](../v15S/PHYSICS.md) (equations),
   [`v15S/MULTICORE.md`](../v15S/MULTICORE.md) (threads), or
   [`v15S/docs/NUMERICAL_GUARANTEES.md`](../v15S/docs/NUMERICAL_GUARANTEES.md)
   (what the numbers hold and where they stop).
