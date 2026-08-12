```markdown
# 🧊 MINIATURE PHYSICS ENGINE (MPE)

<!-- MPE_RELEASE_FREEZE_NOTICE_BEGIN -->
> **Stable release:** `v14S` is the hardened, validated stable form of the v1.4 engine. The `v14A3` release-candidate freeze is complete — all mandatory P0 gates passed before tagging.
<!-- MPE_RELEASE_FREEZE_NOTICE_END -->
<!-- MPE_RELEASE_GATES_NOTICE_BEGIN -->
> **Release quality:** `v14S` was promoted under the criteria in [`RELEASE_GATES.md`](RELEASE_GATES.md). See [`RELEASE_NOTES_v14S.md`](RELEASE_NOTES_v14S.md) for the full change list.
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

## ✨ What's New in v14S

`v14S` completes the release-candidate freeze with a full pass of correctness, stability, validation, documentation, and hygiene fixes. Highlights carried into stable:

- **Domain-driven architecture** — clean `core`, `physics`, `render`, `scene`, `ui_input` modules.
- **Warm-starting contact solver** with multi-point Sutherland–Hodgman manifolds for stable stacking.
- **3D spatial-hash grid broadphase** with adaptive cell sizing.
- **Interactive spring-joint system** with live magenta rendering.
- **POSIX-style debug terminal** — drive the whole simulation from a shell.
- **Built-in validation suite** (F5–F10), including a 60-second long-run stability test.
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
| Spawn object | Hold `Shift` **or** hold `Enter` |
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
For other distributions (Fedora, Arch, SUSE, Alpine, Gentoo, Nix), see [`install/linux/linux_install_instructions.md`](install/linux/linux_install_instructions.md).

### Build and run
```bash
cd src
make clean
make
./engine
```

---

## ⚠️ Known Limitations

- **Wayland:** Mouse locking does not work under native Wayland. Run under X11, or try `GDK_BACKEND=x11 ./engine`.
- **Scene format:** Save/load preserves bodies but **not** spring joints, object IDs, or sleep state. Scene format v2 is planned post-v14S.
- **Object count:** Performance degrades gradually above ~1136 objects; rendering is the primary bottleneck at high counts.
- **Global state:** The engine still uses file-scope globals; full encapsulation is deferred to v15.

---

## 📜 Version History

- **v14S (stable)** — hardened, validated stable form of v1.4. *(this release)*
- **v1.4 Alpha RC3** — domain-driven restructure, spatial-hash broadphase, physics-world encapsulation.
- **v1.4 Alpha 2** — warm-starting solver, multi-point contact manifolds.
- **v1.4 Alpha RC1** — spring joints, joint renderer, color painting, OBB raycast selection.
- **v1.3** — established instanced rendering and spatial-hash direction.

See `evolution.txt` for the full lineage back to stage 0.

---

### Screenshots

<img width="4424" height="1824" alt="Screenshot from 2026-07-18 17-18-52" src="https://github.com/user-attachments/assets/5d1d044d-3926-469e-ab27-9f3719452324" />
<img width="4558" height="1908" alt="Screenshot from 2026-07-18 17-20-09" src="https://github.com/user-attachments/assets/acebe348-707e-485e-835c-08cd1b1dc0fa" />
```
