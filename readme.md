# 🧊 MINIATURE PHYSICS ENGINE (MPE) — v15S

> **STATUS: STABLE — VERIFIED.** Quick profile 60/60, canonical C suite 44/44
> blocking green, MFS unified suite 30/30, determinism twins bitwise identical
> at `MPE_THREADS=1/4/8`. Full-profile physics tests are green under
> ASan/UBSan; the remaining reds are documented pre-existing limitations, not
> regressions.
>
> **Active head:** `v15S` — GTK4 port with a modular kernel, per-world config,
> and no kernel global simulation state. Build from `v15S/src`; verify from the
> repository root with `python3 tools/test_runner.py --profile full`.

**License:** GPL-3.0 · **Language:** C · **UI:** GTK4 · **Renderer:** OpenGL
3.3 Core (via libepoxy) · **Windowing:** GTK4 + X11/Wayland

## 📋 Overview

MPE is a custom-built **3D rigid-body physics engine and real-time rendering
pipeline**, written entirely in **C**.

The headless core uses libc, libm, libdl, and pthreads. The interactive engine
adds GTK4 and OpenGL through libepoxy; GTK4 chooses X11 or Wayland at runtime.
`mpe-tui` additionally needs ncurses.

It is built around four priorities: **mathematical transparency**,
**cache-efficient data layouts**, **deterministic simulation**, and
**real-time scaling**. Supported plugin points are broadphase, solver,
narrowphase shape-pair handlers, and per-world tick modules through the stable
C ABI (`core/mpe_module.h`). Joints remain engine-managed.

Read the full overview in [`docs/OVERVIEW.md`](docs/OVERVIEW.md).

## 🎮 Quick Start

```bash
sudo apt update
sudo apt install build-essential pkg-config libgtk-4-dev libepoxy-dev
# Optional, for the mpe-tui terminal debugger:
sudo apt install libncurses-dev

cd v15S/src
make clean
make
./engine
```

`-Werror` is on by default (`WARNINGS_AS_ERRORS=1`); opt out with
`make WARNINGS_AS_ERRORS=0` for legacy toolchains.

Controls: `W A S D` move, mouse look (left-click to lock, `Escape` to
release), `Space` jump/fly, `0` toggles Game/Debug, `1` opens the debug
terminal, `F5`–`F11` run validation scenes. Full table and platform notes:
[`docs/QUICKSTART.md`](docs/QUICKSTART.md).

## 📚 Read Next

| Document | Contents |
|---|---|
| [`docs/OVERVIEW.md`](docs/OVERVIEW.md) | Design priorities and modular kernel |
| [`docs/WHATS_NEW.md`](docs/WHATS_NEW.md) | v15S feature highlights |
| [`docs/QUICKSTART.md`](docs/QUICKSTART.md) | Build, run, and controls |
| [`docs/VERIFICATION.md`](docs/VERIFICATION.md) | Test profiles, gates, external validation |
| [`docs/LIMITATIONS.md`](docs/LIMITATIONS.md) | Headline limitations |
| [`docs/VERSION_HISTORY.md`](docs/VERSION_HISTORY.md) | Release history and lineage |
| [`docs/INDEX.md`](docs/INDEX.md) | Complete documentation index |

## 🧪 Verify

```bash
python3 tools/test_runner.py --profile quick
```

Use `--profile physics` for isolated legacy/paranoia cases and `--profile full`
for all suites and sanitizers. See [`docs/VERIFICATION.md`](docs/VERIFICATION.md).

---

**License:** GPL-3.0. See [`LICENSE`](LICENSE).
