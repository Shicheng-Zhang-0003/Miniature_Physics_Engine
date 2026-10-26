# MPE Build Instructions — v16R1

> Build for Linux (primary), Windows (MSYS2/MinGW cross), and the headless
> test binaries. `-Werror` is on by default.

---

## 1. Dependencies (Ubuntu / Debian)

```bash
sudo apt update
sudo apt install build-essential pkg-config libgtk-4-dev libepoxy-dev
# Optional, for the mpe-tui terminal debugger:
sudo apt install libncurses-dev
```

Wayland relative-pointer lock needs `libwayland-client` (packaged with
GTK4 on most distros).

## 2. Build & Run (Linux)

```bash
cd v16R1/src
make clean
make
./engine
```

`-Werror` is on by default (`WARNINGS_AS_ERRORS=1`). All sources compile
clean under `-Wall -Wextra -Werror`. Opt out with
`make WARNINGS_AS_ERRORS=0` for legacy toolchains.

### Make targets

| Target | Purpose |
|---|---|
| `make` | Build `engine` (GTK4 GUI) |
| `make mpe-tui` | Build the ncurses terminal debugger |
| `make headless` | Build headless test binary |
| `make debug` | Debug build (`-O0 -g`, separate object dir) |
| `make release` | Optimised release rebuild (`-O3 -DNDEBUG`) |
| `make build_suite` | Build canonical C test suite |
| `make build_suite_scalar` | Suite with SIMD forced off |
| `make installcheck` | Full `--profile full` verification |
| `make installcheck-v1` | Isolated legacy + paranoia regressions |
| `make check-deps` | Verify GTK4/epoxy/X11 present |
| `make check-deps-tui` | Verify ncurses present |
| `make check-flags` | Verify determinism flags present |
| `make check-new-files` | Surface untracked .c files |
| `make uninstall` | Remove installed files |

## 3. Windows (MSYS2 / MinGW)

```bash
make MPE_WINDOWS=1
```

Cross-compile:
```bash
make CC=x86_64-w64-mingw32-gcc MPE_WINDOWS=1
```

No `-ldl`/`-rdynamic` on Windows (LoadLibrary shim in `mpe_platform.h`);
no X11 (Wayland/X11 code is `#ifdef GDK_WINDOWING_X11`-guarded). Static
links `winpthread` + `libgcc` so `.exe`/`.dll` run without MSYS2 DLLs in
PATH.

## 4. Determinism Flags (Mandatory)

Hard-appended on every compile/link so environment or command-line CFLAGS
can never silently drop them:

```
-ffp-contract=off -fno-fast-math -MMD -MP
```

`ENABLE_NATIVE=1` (`-march=native`) breaks bitwise portability; requires
`ALLOW_NONPORTABLE=1` to override.

MSVC (`cl`) needs `/fp:strict` (no FMA contraction) for the same
guarantee — unproven on Windows.

## 5. Verification

```bash
cd /home/magi-01/Desktop/work/projects/475-MPE
python3 tools/test_runner.py --profile quick
python3 tools/test_runner.py --profile full
```

Headless determinism (no GTK):
```bash
cd v16R1/src
make test_determinism
MPE_THREADS=1 ./test_determinism
MPE_THREADS=4 ./test_determinism
MPE_THREADS=8 ./test_determinism
```

## 6. Installation

```bash
make install              # installs to ~/.local by default (PREFIX override)
make installcheck         # full verification profile
make uninstall
```

Installs `mpe-engine`, `mpe-tui`, shaders, and ecosystem plugins.

## 7. Clean

```bash
make clean                # remove build objects
make distclean            # also remove config and generated state
```

`status/engine.cfg` is preserved across `make clean` (user data); use
`distclean` to remove it.
