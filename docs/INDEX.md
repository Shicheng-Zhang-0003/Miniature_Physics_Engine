# 📚 Documentation Index — v15S

Paths are relative to the repository root. **New in this revision:** the C
API reference, the worked module tutorial, the glossary, the restored v15R3
release notes (accidentally deleted by the v15S release commit while still
referenced), and the MFS sub-document section.

## Start here (the five-minute path)

| Document | Contents |
|---|---|
| [`readme.md`](../readme.md) | Status banner, overview, quick start, doc route |
| [`docs/QUICKSTART.md`](QUICKSTART.md) | Build, run, and controls |
| [`v15S/BUILD.md`](../v15S/BUILD.md) | Build instructions (Linux / MSYS2-MinGW / headless), make targets, `-Werror` |
| [`v15S/how_to_use.md`](../v15S/how_to_use.md) | The using-the-engine manual: keys, menus, debug terminal, workflows |
| [`docs/OVERVIEW.md`](OVERVIEW.md) | Design priorities, modular kernel |
| [`docs/WHATS_NEW.md`](WHATS_NEW.md) | v15S feature highlights and inherited v15R3 features |
| [`docs/GLOSSARY.md`](GLOSSARY.md) | Project vocabulary + audit-tag decoder (DESPOT-…, E_BUSY, march, up-cell, …) |

## Write code (API & modules)

| Document | Contents |
|---|---|
| [`docs/API_REFERENCE.md`](API_REFERENCE.md) | Public C surface: world lifecycle, config, module ABI, registry, loader, joints, det_math, ownership rules |
| [`docs/MODULE_TUTORIAL.md`](MODULE_TUTORIAL.md) | Worked example: write, build, hot-load, attach, verify, unload a module (+ the pitfalls audit history) |
| [`v15S/MODULES.md`](../v15S/MODULES.md) | MPI contract (hot-plug physics), loader jail, rejection codes |
| [`v15S/CONFIG.md`](../v15S/CONFIG.md) | 80 tunables, live menu, terminal env/export, round-trip save |
| [`v15S/DEV_GUIDE.md`](../v15S/DEV_GUIDE.md) | Contributing, commit style, audit method, definition of done |
| [`v15S/ARCHITECTURE.md`](../v15S/ARCHITECTURE.md) | Kernel internals, per-world object, data structures |

## Engine physics & numerics

| Document | Contents |
|---|---|
| [`v15S/PHYSICS.md`](../v15S/PHYSICS.md) | Every equation as implemented: integrator, solver, collision, joints |
| [`v15S/docs/NUMERICAL_GUARANTEES.md`](../v15S/docs/NUMERICAL_GUARANTEES.md) | Precise numerical budgets — what holds, where, and what does not |
| [`v15S/MULTICORE.md`](../v15S/MULTICORE.md) | Threading model, island parallelism, determinism guarantees |
| [`v15S/docs/DIAGNOSTICS.md`](../v15S/docs/DIAGNOSTICS.md) | Self-reporting diagnostics & how to read them; S2 bridge transcript |

## Limits, gates & policy

| Document | Contents |
|---|---|
| [`v15S/KNOWN_LIMITATIONS.md`](../v15S/KNOWN_LIMITATIONS.md) | The complete honest list — measured, tracked, labeled (see also headline-only [`docs/LIMITATIONS.md`](LIMITATIONS.md)) |
| [`v15S/REMAINING_WORK.md`](../v15S/REMAINING_WORK.md) | Live TODO and open items |
| [`v15S/RELEASE_GATES.md`](../v15S/RELEASE_GATES.md) | Release exit criteria (P0/P1/P2/P3 + v15S addendum) |
| [`v15S/RELEASE_POLICY.md`](../v15S/RELEASE_POLICY.md) | Release classes; v15S as evolution head over the frozen v15R3 record |
| [`v15S/TESTING.md`](../v15S/TESTING.md) | Test suite, regime matrix, metamorphic tests, oracle discipline |

## Verification, validation & external audits

| Document | Contents |
|---|---|
| [`docs/VERIFICATION.md`](VERIFICATION.md) | Test runner profiles (`quick` / `physics` / `full`), gates, how to run |
| [`docs/VALIDATION.md`](VALIDATION.md) | External physics validation against non-project authorities |
| [`v15S/docs/EXTERNAL_VERIFICATION.md`](../v15S/docs/EXTERNAL_VERIFICATION.md) | Consolidated equation-level external truth record |
| [`docs/AUDIT_REPORT_2026-10-20.md`](AUDIT_REPORT_2026-10-20.md) | Full mathematical / programming / operational audit of the v15S head |
| [`v15S/docs/AUDIT_REPORT_2026-10-21.md`](../v15S/docs/AUDIT_REPORT_2026-10-21.md) | Second robotics-physics deep dive (traction, CCD resting gate, fleet wiring) |
| [`v15S/docs/DESPOT_AUDIT_2026-10-01.md`](../v15S/docs/DESPOT_AUDIT_2026-10-01.md) | External audit — math, programming, operational |
| [`v15S/docs/DESPOT_AUDIT_2026-10-07.md`](../v15S/docs/DESPOT_AUDIT_2026-10-07.md) | External audit — full tree; the LIE-ledger round |
| [`v15S/docs/DESPOT_AUDIT_2026-10-08.md`](../v15S/docs/DESPOT_AUDIT_2026-10-08.md) | External audit — "Tandem-Helix: everything open" |
| [`v15S/docs/DESPOT_AUDIT_2026-10-12.md`](../v15S/docs/DESPOT_AUDIT_2026-10-12.md) | External audit — full sweep |
| [`v15S/docs/DESPOT_AUDIT_2026-10-13.md`](../v15S/docs/DESPOT_AUDIT_2026-10-13.md) | External audit — full sweep |
| [`v15S/docs/DESPOT_AUDIT_2026-10-19.md`](../v15S/docs/DESPOT_AUDIT_2026-10-19.md) | External audit — engine truth ledger across the twin repos |

## History & records

| Document | Contents |
|---|---|
| [`docs/VERSION_HISTORY.md`](VERSION_HISTORY.md) | Release lineage v1.2-alpha → v15S |
| [`release_notes_v15S.md`](../release_notes_v15S.md) | Current release notes (historical findings on shooting / intake correctness) |
| [`release_notes_v15R3.md`](../release_notes_v15R3.md) | v15R3 record, as tagged (`a3_release_freeze = 1`) — restored 2026-10-25 after the v15S release commit deleted it while three docs still cited it |
| `fix_log.txt` | Execution history of fix scripts (tracked intentionally; see `.gitignore`) |

## MFS robotics sub-documents (`v15S/src/ecosystem/mfs/docs/`)

| Document | Contents |
|---|---|
| [`ARCHITECTURE.md`](../v15S/src/ecosystem/mfs/docs/ARCHITECTURE.md) | MFS internal structure and data flow |
| [`FTC_SPECS.md`](../v15S/src/ecosystem/mfs/docs/FTC_SPECS.md) | Control scheme and regulation details for the league |
| [`MODELS.md`](../v15S/src/ecosystem/mfs/docs/MODELS.md) | Motor / drivetrain / physics models behind `ftc/` |
| [`TESTING.md`](../v15S/src/ecosystem/mfs/docs/TESTING.md) | MFS suite (30 gated cases) and how to run it (`build_tests.sh`) |
| [`VALIDATION.md`](../v15S/src/ecosystem/mfs/docs/VALIDATION.md) | MFS-level validation records |
| [`KNOWN_FAILURES.md`](../v15S/src/ecosystem/mfs/docs/KNOWN_FAILURES.md) | Measured failures the MFS layer documents and works around |
| [`SYNC_CONTRACT.md`](../v15S/src/ecosystem/mfs/docs/SYNC_CONTRACT.md) | Twin-tree sync discipline (461-MFS / 472-MGB), what `sync_mfs_check.sh` governs |

## Keeping this index honest

Adding a document? Add a row here in the same commit. Deleting one? Remove
its row **and** fix anything that points at it (search first:
`grep -rn '<filename>' docs v15S README*`). This file previously missed
`v15S/docs/AUDIT_REPORT_2026-10-21.md`, `v15S/docs/EXTERNAL_VERIFICATION.md`,
`v15S/how_to_use.md`, the whole MFS sub-document section, and link-rotted
`release_notes_v15R3.md` — that is exactly the drift class this rule guards
 against.
