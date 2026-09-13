# 🧊 MINIATURE PHYSICS ENGINE (MPE)

> **MPE-only:** the engine builds and all headless tests pass with MPE core only: `cd v15R3/src && make && python3 ../../tools/test_runner.py`.

<!-- MPE_RELEASE_FREEZE_NOTICE_BEGIN -->
> **Current development tree:** `v15R3` continues the v15 configuration-system work. It is not a tagged stable release; use the release gates before promotion.
<!-- MPE_RELEASE_FREEZE_NOTICE_END -->
<!-- MPE_RELEASE_GATES_NOTICE_BEGIN -->
> **Release quality:** the current criteria are in [`v15R3/RELEASE_GATES.md`](v15R3/RELEASE_GATES.md). The current candidate notes are in [`v15R3/release_notes_v15R2.md`](v15R3/release_notes_v15R2.md); [`v15R3/release_notes_v15R1.md`](v15R3/release_notes_v15R1.md) documents the prior RC.
<!-- MPE_RELEASE_GATES_NOTICE_END -->

**License:** GPL-3.0 · **Language:** C · **UI:** GTK3 · **Renderer:** OpenGL 3.3 Core

---

## 📋 Overview

MPE is a custom-built **3D rigid-body physics engine and real-time rendering pipeline**, written entirely in **C**. It runs on a **zero-dependency core** — the only external requirements are **GTK3** (windowing/UI) and **OpenGL** (render backend).

MPE is built around four priorities:

- **Mathematical transparency** — every integrator, solver, and collision test is hand-written and inspectable.
- **Cache-efficient data layouts** — tightly packed structs and contiguous instance buffers.
- **Deterministic simulation** — fixed-timestep physics decoupled from render framerate.
- **Real-time scaling** — GPU instancing and an O(N) spatial-hash broadphase.

---

## ✨ What's New in v15R3

`v15R3` is the active development cycle for the centralised configuration system (MPE-only run; MFS robotics is parked in `v15R3/robotics_backup/` — see that folder's `README_PARKED.md`). Highlights currently present in the tree:

- **Domain-driven architecture** — clean `core`, `physics`, `render`, `scene`, `ui_input` modules.
- **Warm-starting contact solver** with multi-point Sutherland–Hodgman manifolds for stable stacking.
- **3D spatial-hash grid broadphase** with adaptive cell sizing.
- **Interactive spring-joint system** with live magenta rendering.
- **POSIX-style debug terminal** — drive the whole simulation from a shell.
- **Built-in validation suite** (F5–F11), including a 60-second long-run stability test and config torture test.
- **Shader/render failure visibility** — the engine no longer continues silently in a broken render state.

---

## 🎨 Rendering System

### Hardware-Instanced Rendering

MPE eliminates per-object draw calls using **GPU instancing**:

- The CPU packs model matrices + colors into contiguous buffers.
- The GPU batches all dynamic bodies into **two instanced draws** (spheres, cubes).
- The grid, selection outline, and spring-joint overlay share a utility shader with cached uniform locations.

### Shading

- Custom **GLSL Phong** lighting (ambient + diffuse + specular).
- **Equatorial axis rings** painted on every object (red/green/blue) so rotation is visible at a glance.

---

## ⚙️ Physics Engine

### Broadphase — Spatial Hash Grid

Objects are mapped into hashed grid buckets; collision checks are limited to local neighborhoods for **average O(N)** scaling. Cell size adapts to object radii. A sleep system removes inactive bodies from the solver.

### Narrowphase

| Pair | Method |
|---|---|
| Sphere–Sphere | Analytical distance test |
| Sphere–OBB | Closest-point projection |
| OBB–OBB | Separating Axis Theorem (15 axes) + Sutherland–Hodgman face clipping |

### Solver

- **Impulse-based sequential solver**, 16 iterations, with **warm starting**.
- Static + kinetic friction, rolling friction, Baumgarte penetration correction.
- Positional depenetration pass for pile stability.

### Integration

- **Semi-implicit (symplectic) Euler** for linear motion.
- **Quaternion-based angular integration** (no gimbal lock).
- **Fixed 60 Hz timestep** with an accumulator and 5-substep cap (spiral-of-death prevention).

---

## 🧮 Mathematics Core

A fully custom, dependency-free math library: 3D vectors, 4×4 matrices, quaternions, and inertia tensors — designed for tightly packed, cache-friendly structs.

---

## 🖥️ Platform & Rendering Stack

| Layer | Technology |
|---|---|
| Windowing / UI | GTK3 |
| Graphics API | OpenGL 3.3 Core (via libepoxy) |
| Lighting | Custom GLSL Phong |
| Debug visualization | Axis rings, wireframe selection, joint lines, overflow counters |

---

## 🎮 Controls

### Movement & Camera

| Action | Input |
|---|---|
| Move | `W A S D` |
| Look around | Mouse (left-click to lock) |
| Jump / fly up | `Space` |
| Fly down (Debug) | `Shift` |
| Steer camera mouse-free (Debug) | `I J K L` |
| Release mouse | `Escape` |
| Re-lock mouse (Debug) | `M` |
| Toggle Game / Debug mode | `0` |

### Spawning

| Action | Input |
|---|---|
| Spawn object | Hold `Enter` |
| Spawner settings | `8` |

### Selection & Editing

| Action | Input |
|---|---|
| Select object | Right-click (raycast) **or** `R` (Debug) |
| Open object menu | `E` |
| Apply impulse | `F` |
| Delete object | Middle-click **or** `Delete` (Debug) |
| World settings | `7` |
| Save / Load scene | `9` |

### Debug Terminal & Validation

| Action | Input |
|---|---|
| Open debug terminal | `T` or `1` (Debug) |
| Stability stack test | `F5` |
| Sleep / wake test | `F6` |
| Editor torture test | `F7` |
| Spawn stress test (300 objects) | `F8` |
| Validation report | `F9` |
| Long-run validation (60 s) | `F10` |
| **Config torture test** | **F11** |

---

## 🐚 Debug Terminal

In Debug Mode, press `T` (or `1`) to open a **POSIX-style shell** over the physics world. The simulation is exposed as a virtual filesystem:

| Path | Contents |
|---|---|
| `/obj` | All rigid bodies |
| `/joint` | All spring joints |
| `/world` | World variables (gravity, drag, friction) |
| `/camera` | Camera state |
| `/spawner` | Spawner settings |

A few examples:

```
touch new.sph            # spawn a sphere
ln 1 2                   # spring-join objects 1 and 2
mv 3 /pos/0/10/0         # teleport object 3
chown 5.0 3              # set object 3's mass to 5 kg
chmod static 3           # make it immovable
kill -STOP 3             # put it to sleep
ps aux                   # list every body with state
export GRAVITY=-2.0      # change world gravity
```

Type `help` for the full command list, `man <command>` for usage. `Ctrl+L` clears, `Esc` closes. Mutating commands require Debug Mode; in Game Mode the terminal is read-only.

---

## 🧪 Validation Tests

MPE ships with built-in stability tests:

| Key | Test |
|---|---|
| `F5` | 10-cube stability stack |
| `F6` | Sleeping cube + moving projectile (sleep/wake) |
| `F7` | Editor torture: select, joint, delete, reset |
| `F8` | Spawn stress: up to 300 mixed objects |
| `F9` | Print validation report |
| `F10` | Long-run validation: 3600 ticks (60 s) of idle stability |

`F10` monitors for NaN values, fallen objects, and residual motion, printing `PASS`/`FAIL` at the end.

---

## 🛠️ Build Instructions

### Dependencies (Ubuntu / Debian)

```bash
sudo apt update
sudo apt install build-essential pkg-config libgtk-3-dev libepoxy-dev
```

For other distributions (Fedora, Arch, SUSE, Alpine, Gentoo, Nix), see [install/linux/linux_install_instructions.md](install/linux/linux_install_instructions.md).

### Build and run

```bash
cd v15R3/src
make clean
make
./engine
```

---

## ⚠️ Known Limitations

- **Wayland:** Mouse locking does not work under native Wayland. Run under X11, or try `GDK_BACKEND=x11 ./engine`.
- **Scene format:** v200 saves bodies (with stable IDs, sleep state, damping) plus spring and revolute joints, with a CRC32 integrity footer. Files ≤v153 still load via the legacy reader.
- **Object count:** Performance degrades gradually above ~1136 objects; rendering is the primary bottleneck at high counts.
- **Global state:** All simulation state (bodies, IDs, joints, constraints, caches, solver scratch) is owned by `physics_world`; the file-scope sim globals are retired. App/UI state (camera, input, selection, terminal, diagnostics) remains global by design.

---

## 📜 Version History

- **v15R3 (development)** — ongoing v15 configuration-system work, MPE-only. *(current tree)*
- **v15R2** — config-system hardening + MFS robotics (prior RC, see `release_notes_v15R2.md`).
- **v1.4 Alpha RC3** — domain-driven restructure, spatial-hash broadphase, physics-world encapsulation.
- **v1.4 Alpha 2** — warm-starting solver, multi-point contact manifolds.
- **v1.4 Alpha RC1** — spring joints, joint renderer, color painting, OBB raycast selection.
- **v1.3** — established instanced rendering and spatial-hash direction.

See `evolution.txt` for the full lineage back to stage 0.

---

### Screenshots

<img width="4424" height="1824" alt="Screenshot from 2026-07-18 17-18-52" src="https://github.com/user-attachments/assets/5d1d044d-3926-469e-ab27-9f3719452324" />
<img width="4558" height="1908" alt="Screenshot from 2026-07-18 17-20-09" src="https://github.com/user-attachments/assets/acebe348-707e-485e-835c-08cd1b1dc0fa" />

---

## 🧪 Headless test suite

MPE ships a headless regression suite (no GTK/OpenGL required):

| Test | Proves |
|------|--------|
| `two_world` | Independent `physics_world` instances |
| `revolute` | Hinge joints hold anchor and allow swing |
| `cylinder_drop` | Cylinder settles on the floor |
| `driven_wheel` | Torque → friction → translation |
| `math3_inverse` | Matrix inverse at small inertia tensors |
| `floor_collision_diag` | Floor contact diagnostics |
| `cylinder_sphere/cube/cylinder` | Cylinder narrowphase pairs |
| `list4_cylinder_floor` | Tipped-cylinder floor regression |
| `scene_roundtrip` | Save/load v153 format round-trip |
| `static_hold` | Coulomb stick holds / yields past friction angle |
| `rolling_decay` | Contact-patch rolling resistance decay |
| `ccd_sweep` | Swept TOI: no tunneling at 144 m/s |
| `kinematic` | Velocity-driven platforms carry bodies |

Run with `python3 tools/test_runner.py`.

### Determinism and precision
- Fixed 1/60 s timestep, fixed solver iteration order, exact IEEE `+ - * / sqrt`.
- Per-tick transcendentals (damping retention, rotation rotors) use fixed-coefficient polynomials (`v15R3/src/core/det_math.h`), bit-identical on all IEEE-754 targets; the build disables FP contraction (`-ffp-contract=off`).
- Proven by `determinism`: twin worlds agree bitwise over 600 ticks.
- float32 world: the playable volume is bounded (±250 m), where float resolution (~0.03 mm at the corners) sits 300× below contact slop. No origin rebasing required inside the boundary box.

