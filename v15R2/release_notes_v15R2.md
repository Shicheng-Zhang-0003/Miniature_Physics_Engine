# MPE v15R2 — Release Candidate 2

**Public release name:** `V1.5RC2`  
**Codebase version:** `v15R2`  
**Release date:** August 2026  
**License:** GPL-3.0

---

## What is this?

`v15R2` is the second release candidate in the v15 configuration-system
cycle. It completes the hardening pass after `v15R1`: the engine source is
now contained in the `v15R2/` release tree, known correctness fixes have been
applied, and the release gates and validation tooling have been refreshed for
the current version.

The centralised configuration system introduced in `v15R1` remains the
headline feature: 57 live tunables, persistent `status/engine.cfg` storage,
an in-engine configuration menu, and terminal `env`, `export`, and `config`
commands.

## Changes since v15R1

- Reorganised the active engine into `core`, `physics`, `render`, `scene`,
  `ui_input`, and `config` domains under `v15R2/src`.
- Corrected release identity and validation paths for v15R2, including the
  startup version string, user-facing overlay, and V01/V02/V04 scripts.
- Repaired the ASan/UBSan build workflow and made release checks fail closed:
  sanitizer build failures and skipped P0 gates cannot be reported as passes.
- Fixed MicroVim file state and line-length handling, terminal capture cleanup,
  uptime, and filesystem-sanitisation wake behaviour.
- Removed obsolete integration, force, perspective, and camera code while
  retaining the documented v15R2 physics and configuration behavior.
- Improved renderer failure visibility and release/repository hygiene.

## Validation status

Automated checks completed on 22 August 2026:

- [x] ASan + UBSan build (`validation/V01.sh`)
- [x] Optimised clean build with zero compiler warnings (`validation/V02.sh`)
- [x] GUI validation: startup/lifecycle, F5–F11, and several-minute idle run
- [x] Manual sanitizer torture run (F5, F6, F7, F8, F10, F11)
- [x] Completed `validation/V03.py` gate log with no skipped gates

`V1.5RC2` must not be published until every remaining manual item is recorded
as passing in the P0 gate log.

## Known limitations

- Native Wayland mouse locking is not supported; run under X11 when mouse lock
  is required.
- Scene save/load preserves bodies, but not spring joints, stable object IDs,
  or sleep state.
- Rendering becomes the primary bottleneck at high object counts (roughly
  1,136 objects in the documented stress profile).
- Scene state remains file-scope global state; full `PhysicsWorld`
  encapsulation is deferred.

## Build

```bash
cd v15R2/src
make clean
make
./engine
```

Use `./compile_asan` for an ASan + UBSan build. The release gates and manual
test instructions are in `RELEASE_GATES.md` and `validation/`.

## Deferred work

The next development cycle will address larger architectural and physics work:
`PhysicsWorld` encapsulation, a split of `simulation.c`, scene format v2,
continuous collision detection, generic constraints, solver islanding, UI
state-machine cleanup, and Wayland mouse-lock support.

---

See [release_notes_v15R1.md](release_notes_v15R1.md) for the original v15
configuration-system release candidate.
