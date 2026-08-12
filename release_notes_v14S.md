```markdown
# MPE v14S — Stable Release

**Release date:** August 2026
**Tag:** `v14S`
**License:** GPL-3.0

---

## What is this?

`v14S` is the **first stable release** of the v1.4 series of the Miniature Physics Engine. It is the hardened, validated form of `v14A3` (Alpha RC3), promoted after passing all 12 mandatory P0 gates defined in [`RELEASE_GATES.md`](RELEASE_GATES.md).

This release represents the culmination of 51 A3 patches, 23 MPE tasks, and 11 stable-release hardening fixes (S-01 through S-11).

---

## Highlights

### Physics Engine
- **Spatial hash grid broadphase** with adaptive cell sizing (replaced sweep-and-prune)
- **Warm-starting impulse solver** (16 iterations) with body-local contact caching and property stamps
- **Sutherland–Hodgman face clipping** for OBB–OBB contacts (up to 4 contact points per manifold)
- **15-axis SAT** narrowphase for cube–cube collision
- **Semi-implicit Euler** integration with quaternion angular dynamics
- **Fixed 60 Hz timestep** with accumulator and 5-substep spiral-of-death cap
- **Sleep system** — bodies below velocity thresholds sleep automatically
- **Positional depenetration pass** for pile stability
- **Spring joints** with damping, rendered as magenta lines

### Rendering
- **GPU instancing** — all spheres in one draw call, all cubes in one
- **Custom GLSL Phong shading** with equatorial axis rings for rotation visibility
- **Wireframe selection outline** and spring-joint overlay
- **Cached uniform locations** (no per-frame `glGetUniformLocation` spam)

### Debug & Validation
- **POSIX-style debug terminal** — drive the entire simulation from a virtual shell (`touch`, `rm`, `mv`, `ln`, `chmod`, `chown`, `kill`, `ps`, `top`, `df`, `export`, and more)
- **Built-in validation suite**: F5 (stack), F6 (sleep/wake), F7 (editor torture), F8 (stress), F9 (report), F10 (60-second long-run)
- **Overflow counters** visible in overlay and validation report (broadphase nodes, pairs, manifolds, dedupe)

### Architecture
- **Domain-driven module layout**: `core/`, `physics/`, `render/`, `scene/`, `ui_input/`
- **Stable object IDs** — selection, joints, and terminal references survive deletion/reorder
- **Scene save/load** (binary format, magic `MPE3`)

---

## Changes from v14A3 to v14S (Stable Hardening)

These are the S-series fixes applied during the RC freeze:

| ID | Change |
|---|---|
| S-01 | Shader/render failure no longer silently continues — engine reports failure and skips draws |
| S-02 | Broadphase description corrected (spatial hash grid, per-substep) |
| S-03 | Physics timestep corrected to 60 Hz (removed phantom 120 Hz) |
| S-04 | Validation checklist version bumped to match codebase |
| S-05 | User guide updated with debug terminal, keyboard-only keys, Enter-spawn, F5–F10 |
| S-06 | Scene format limitations documented (joints not saved, IDs reassigned, sleep not persisted) |
| S-07 | Removed dead `#include <complex.h>` from 4 files |
| S-08 | Manifold capacity constant named (`A3_MAX_MANIFOLDS`) |
| S-09 | Removed dead `c_key_pressed` input field |
| S-10 | Build instructions unified on `make` (removed redundant `src/compile` wrapper) |
| S-11 | Version bumped to `v14S`; release freeze lifted |

---

## Changes from v1.3 to v14S (Full Feature Summary)

### v1.4 Alpha RC1
- Interactive spring joint system (pendulums, chains)
- Dynamic spring joint renderer (magenta lines)
- Object color painting (8-preset sub-menu)
- Robust constraint deletion via stable object IDs
- Slab-method OBB raycast selection

### v1.4 Alpha 2
- Warm-starting contact solver (persists impulses across frames)
- Multi-point contact manifolds via Sutherland–Hodgman face clipping

### v1.4 Alpha RC3 (v14A3)
- Domain-driven architecture restructure
- Broadphase bounding sphere radius bug fix
- Sutherland–Hodgman polygon buffer safety
- Physics world encapsulation (`PhysicsWorld` struct)
- Fixed-timestep physics accumulator
- Spatial hash grid broadphase (replaced sweep-and-prune)
- Adaptive cell sizing
- Sleep system with staticize/restore solver trick
- Positional depenetration pass
- POSIX debug terminal (28 commands)
- F5–F10 validation test suite
- Spawn overlap separation
- NaN sanitization and hardening

### v14S (this release)
- All S-01 through S-11 hardening fixes (see table above)
- Full P0 gate validation pass
- Documentation aligned with implementation

---

## Known Limitations

- **Wayland:** Mouse locking does not work under native Wayland. Run under X11 or set `GDK_BACKEND=x11`.
- **Scene format:** Save/load preserves bodies but **not** spring joints, object IDs, or sleep state. Scene format v2 is planned for v15.
- **Object count:** Performance degrades gradually above ~1136 objects. Rendering is the primary bottleneck.
- **Global state:** The engine still uses file-scope globals. Full encapsulation is deferred to v15.
- **Spawn key:** Enter spawns objects. Shift is fly-down in Debug Mode.

---

## Build

### Dependencies (Ubuntu/Debian)
```bash
sudo apt install build-essential pkg-config libgtk-3-dev libepoxy-dev
```

### Compile and run
```bash
cd src
make clean
make
./engine
```

For other distributions (Fedora, Arch, SUSE, Alpine, Gentoo, Nix), see [`install/linux/linux_install_instructions.md`](install/linux/linux_install_instructions.md).

---

## Controls (Quick Reference)

| Action | Input |
|---|---|
| Move | WASD |
| Look | Mouse (left-click to lock) |
| Jump / Fly up | Space |
| Fly down (Debug) | Shift |
| Spawn | Hold Enter |
| Select | Right-click or R (Debug) |
| Object menu | E |
| Apply impulse | F |
| Delete | Middle-click or Delete (Debug) |
| Debug terminal | T or 1 (Debug) |
| Toggle mode | 0 |
| World settings | 7 |
| Spawner settings | 8 |
| Save/Load | 9 |
| Validation tests | F5–F10 |

---

## Validation

All mandatory P0 gates passed:

- [x] Release freeze policy active
- [x] Build passes cleanly
- [x] Startup prints correct version
- [x] Shader/render failures visible
- [x] Input and focus-loss stable
- [x] Editor torture tests pass
- [x] Physics stacks settle
- [x] Sleeping stacks remain sleeping
- [x] Overflow counters visible
- [x] F5–F10 validation tests pass
- [x] Documentation matches code
- [x] Repository artifacts clean
- [x] Sanitizer (ASan + UBSan) validation passes

---

## What's Next (v15 cycle)

The following are explicitly deferred to the v14S → v15S development cycle:

- Full global-state removal and `PhysicsWorld` encapsulation
- `simulation.c` god-file split (v15A1)
- Scene format v2 (joints, stable IDs, sleep state)
- Multithreading
- Continuous collision detection
- Generic constraint framework
- Solver islanding
- UI state-machine rewrite
- Wayland mouse-lock support

---

## Links

- [README](README.md) — project overview
- [User Guide](how_to_use.md) — full controls and usage
- [Release Gates](RELEASE_GATES.md) — criteria used for this release
- [Release Policy](RELEASE_POLICY.md) — freeze rules that governed this cycle
- [Validation Checklist](A3_VALIDATION.md) — test procedures used

---

*MPE is free software, licensed under the GNU GPL v3. See [LICENSE](LICENSE).*
