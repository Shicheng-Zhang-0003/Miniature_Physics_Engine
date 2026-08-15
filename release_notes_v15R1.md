# MPE v15R1 — Release Candidate 1

**Release date:** August 2026
**Tag:** `v15R1`
**License:** GPL-3.0

---

## What is this?

`v15R1` is the **first release candidate** of the v15 series of the
Miniature Physics Engine. It introduces the **centralised configuration
system** — a single-source-of-truth registry for all 57 tunable engine
parameters, with live in-engine editing, persistent storage, and terminal
integration.

This release builds on the stable `v14S` foundation and represents
Tasks 24–41 of the MPE task sequence.

---

## Highlights

### Centralised Configuration System
- **`src/config/` module** with LOCKED constants manifest and tunable registry
- **57 tunable parameters** across 13 categories, each with metadata
  (display name, help text, type, min/max bounds, debug-only flag)
- **Live in-engine config menu** (key `6`) with category navigation and
  numerical input dialogs
- **Persistent storage** to `status/engine.cfg` (text INI format)
- **Startup load / exit save** lifecycle
- **Terminal integration** — `env`, `export KEY=value`, `config save|load|reset`
- **F9 config report** — validation report now dumps all config values
- **F11 config torture test** — randomises all tunables and runs 60s idle

### Architecture
- **`mpe_constants.h`** — single source of truth for all compile-time constants
- **`mpe_config.h`** — typed config struct (`mpe_config_t`) + registry API
- **`mpe_config_schema.c`** — declarative parameter registry (add one line to add a parameter)
- **`mpe_config.c`** — init/get/set/clamp/reset/save/load implementation
- **`config_menu.c`** — category-tree menu with clamped editing and debug-only gating
- **Hot-path reads** are direct struct access (`g_cfg.world.gravity`), zero overhead
- **Cold-path access** via registry for menu/terminal/save operations

### Engine Migration (v14S → v15R1)
All hardcoded globals and magic numbers migrated to the config store:
- World: gravity, drag, floor friction
- Timestep: solver iterations, max substeps, speed clamps
- Sleep: thresholds, timer, wake thresholds
- Solver: penetration slop, bias factor, restitution tuning, friction threshold, warm-start distance
- Depenetration: correction factor, max correction, slop, wake depth, rebuild iterations
- Broadphase: cell size default/min/max/multiplier, max cell span
- Joints: max acceleration, spring k, damping (normal + soft)
- Boundary: floor emergency slop, floor velocity slop
- Spawner: mass, radius, cube mass/extent, speed, friction, overlap separation
- Body defaults: restitution, friction per type
- Camera: move speed, mouse sensitivity, steer sensitivity, horizontal friction, jump height, IJKL speed
- Render: light position, ambient, specular coefficient/exponent
- UI: change rates, long-run ticks, enter-spawn timing

---

## Changes from v14S to v15R1

| Area | Change |
|---|---|
| Config system | New `src/config/` module with registry, menu, persistence |
| Constants | All compile-time constants consolidated into `mpe_constants.h` |
| Globals removed | `world_gravity_y`, `world_drag_coefficient`, `world_surface_friction_*`, `variable_change_rate`, `jump_height`, `spawn_*` globals eliminated |
| Physics params | All magic numbers in solver/sleep/depenetration/boundary migrated to config |
| Terminal | `env`/`export` rewritten to iterate registry; `config save|load|reset` added |
| Menu | New config menu (key 6) with 13-category navigation |
| Scene menu | Expanded to 6 options (save/load/clear/save config/reset config/exit) |
| Validation | F9 config dump; F11 config torture test added |
| Docs | Full documentation update for v15R1 |

---

## Known Limitations

- **Wayland:** Mouse locking does not work under native Wayland. Run under X11, or try `GDK_BACKEND=x11 ./engine`.
- **Scene format:** Save/load preserves bodies but **not** spring joints, object IDs, or sleep state. Scene format v2 is planned post-v15R1.
- **Object count:** Performance degrades gradually above ~1136 objects. Rendering is the primary bottleneck at high counts.
- **Global state:** The engine still uses file-scope globals for scene state. Full encapsulation is deferred to v15S.
- **F11 torture test:** After F11, config remains randomised until manually reset via config menu (key 6 → Reset) or terminal (`config reset`).

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

---

## Controls (Quick Reference)

| Action | Input |
|---|---|
| Move | `W A S D` |
| Look | Mouse (left-click to lock) |
| Jump / Fly up | `Space` |
| Fly down (Debug) | `Shift` |
| Spawn | Hold `Enter` |
| Select | Right-click or `R` (Debug) |
| Object menu | `E` |
| Apply impulse | `F` |
| Delete | Middle-click or `Delete` (Debug) |
| Debug terminal | `T` or `1` (Debug) |
| Toggle mode | `0` |
| **Config menu** | **`6`** |
| World settings | `7` |
| Spawner settings | `8` |
| Save/Load scene | `9` |
| Validation tests | `F5`–`F10` |
| **Config torture test** | **`F11`** |

---

## Validation

All mandatory P0 gates must pass:
- [ ] Release freeze policy active
- [ ] Build passes cleanly
- [ ] Startup prints correct version
- [ ] Config system initialises
- [ ] Shader/render failures visible
- [ ] Input and focus-loss stable
- [ ] Config menu works correctly
- [ ] Editor torture tests pass
- [ ] Physics stacks settle
- [ ] Sleeping stacks remain sleeping
- [ ] Overflow counters visible
- [ ] F5–F11 validation tests pass
- [ ] Config system round-trips
- [ ] Documentation matches code
- [ ] Repository artifacts clean
- [ ] Sanitizer (ASan + UBSan) validation passes

---

## What's Next (v15S cycle)

The following are explicitly deferred to the v15R1 → v15S development cycle:
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
- [README](readme.md) — project overview
- [User Guide](how_to_use.md) — full controls and usage
- [Release Gates](RELEASE_GATES.md) — criteria used for this release
- [Release Policy](RELEASE_POLICY.md) — development rules for this cycle

---

*MPE is free software, licensed under the GNU GPL v3. See [LICENSE](LICENSE).*
