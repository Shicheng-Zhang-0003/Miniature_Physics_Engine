# Quick Start

## Build (Ubuntu / Debian)

```bash
sudo apt update
sudo apt install build-essential pkg-config libgtk-4-dev libepoxy-dev
# Optional, for the mpe-tui terminal debugger:
sudo apt install libncurses-dev
```

```bash
cd v16R1/src
make clean
make
./engine
```

`-Werror` is on by default (`WARNINGS_AS_ERRORS=1`). All sources compile
clean under `-Wall -Wextra -Werror`. Opt out with
`make WARNINGS_AS_ERRORS=0` for legacy toolchains.

## Controls

| Action | Input |
|---|---|
| Move | `W A S D` |
| Look around | Mouse (left-click to lock) |
| Jump / fly up | `Space` |
| Fly down (Debug) | `Shift` |
| Release mouse | `Escape` |
| Toggle Game / Debug mode | `0` |
| Open debug terminal | `1` (Debug) |
| Stability stack test | `F5` |
| Sleep / wake test | `F6` |
| Editor torture test | `F7` |
| Spawn stress test (300 objects) | `F8` |
| Validation report | `F9` |
| Long-run validation (60 s) | `F10` |
| Config torture test | `F11` |

For all platforms and make targets, see [`BUILD.md`](../v16R1/BUILD.md).
For the full user guide (modes, editor, terminal, robots), see
[`how_to_use.md`](../v16R1/how_to_use.md).
