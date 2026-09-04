# 🧊 MINIATURE PHYSICS ENGINE (MPE)

<!-- MPE_RELEASE_FREEZE_NOTICE_BEGIN -->
> **Current development tree:** `v15R2` continues the v15 configuration-system work. It is not a tagged stable release; use the release gates before promotion.
<!-- MPE_RELEASE_FREEZE_NOTICE_END -->
<!-- MPE_RELEASE_GATES_NOTICE_BEGIN -->
> **Release quality:** the current criteria are in [`v15R2/RELEASE_GATES.md`](v15R2/RELEASE_GATES.md). The current candidate notes are in [`v15R2/release_notes_v15R2.md`](v15R2/release_notes_v15R2.md); [`v15R2/release_notes_v15R2.md`](v15R2/release_notes_v15R2.md) documents the prior RC.
<!-- MPE_RELEASE_GATES_NOTICE_END -->

# Note: This codebase is the latest snapshot of the MPE FTC Simulator, or MFS. Stability changes were made in MFS that were determined to be beneficial to the original MPE engine standalone. Future v16 Series development will focus on modularisation to convert MFS into a plug and install kernel module add-on to the original MPE system.

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

## ✨ What's New in v15R2

`v15R2` is the active development cycle for the centralised configuration system. Highlights currently present in the tree:

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
cd src
make clean
make
./engine
```

---

## ⚠️ Known Limitations

- **Wayland:** Mouse locking does not work under native Wayland. Run under X11, or try `GDK_BACKEND=x11 ./engine`.
- **Scene format:** Save/load preserves bodies but **not** spring joints, object IDs, or sleep state. Scene format v2 is planned post-v15R2.
- **Object count:** Performance degrades gradually above ~1136 objects; rendering is the primary bottleneck at high counts.
- **Global state:** The engine still uses file-scope globals; full encapsulation is deferred to v15.

---

## 📜 Version History

- **v15R2 (development)** — ongoing v15 configuration-system work. *(current tree)*
- **v1.4 Alpha RC3** — domain-driven restructure, spatial-hash broadphase, physics-world encapsulation.
- **v1.4 Alpha 2** — warm-starting solver, multi-point contact manifolds.
- **v1.4 Alpha RC1** — spring joints, joint renderer, color painting, OBB raycast selection.
- **v1.3** — established instanced rendering and spatial-hash direction.

See `evolution.txt` for the full lineage back to stage 0.

---

### Screenshots

<img width="4424" height="1824" alt="Screenshot from 2026-07-18 17-18-52" src="https://github.com/user-attachments/assets/5d1d044d-3926-469e-ab27-9f3719452324" />
<img width="4558" height="1908" alt="Screenshot from 2026-07-18 17-20-09" src="https://github.com/user-attachments/assets/acebe348-707e-485e-835c-08cd1b1dc0fa" />

<!-- MFS_FTC_SECTIONS_BEGIN -->

---

## 🤖 MFS — FTC Robotics Simulator

This tree also carries **MFS** (MPE FTC Simulator), a robotics simulation
layer currently merged into the engine. MFS turns MPE into an FTC-oriented
robot simulator: a mecanum drivetrain with physically-truthful wheel contact,
DC motor electrical models, battery voltage sag, and dead-reckoning odometry.

> MFS exists to make autonomous tuning realistic. Motor commands become torque,
> torque spins cylinder wheels, wheels grip the floor through anisotropic
> roller friction, and sensors read back from simulated state. Nothing is
> faked at the contact layer.

### Capabilities
- **Cylinder wheel bodies** with correct axle inertia (`I = ½·m·r²`)
- **Mecanum drivetrain** via real anisotropic roller friction (±45° rollers) — no chassis-force cheats
- **Tank drivetrain** via motor torque → wheel traction
- **DC motor model** — BackEMF, gear ratio, Kt/Kv, stall/free-speed limits
- **Battery model** — voltage sag under multi-motor load
- **Odometry** — wheel encoders + IMU-style heading for dead reckoning
- **Revolute joints** — wheels hinged to the chassis with axis correction
- **Idle hold** — gearbox-style lock so a parked robot stays parked

### Robot controls
| Key | Action |
|-----|--------|
| `G` | Drive forward |
| `B` | Drive backward |
| `V` | Strafe right |
| `N` | Strafe left |
| `C` | Rotate left (CCW) |
| `H` | Rotate right (CW) |

Spawn the robot from the debug terminal with `touch robot`.

### Headless test suite
MFS ships a headless regression suite (no GTK/OpenGL required):

| Test | Proves |
|------|--------|
| `two_world` | Independent `physics_world` instances |
| `revolute` | Hinge joints hold anchor and allow swing |
| `teleop_drive` | Tank drive moves the robot |
| `mecanum_drive` | Strafe via real roller friction |
| `cylinder_drop` | Cylinder settles on the floor |
| `driven_wheel` | Torque → friction → translation |
| `math3_inverse` | Matrix inverse at small inertia tensors |
| `ftc_integration` | Drive / turn / strafe sequence |
| `physics_truth` | 24 physical-law assertions |

Run with `python3 tools/test_runner.py`.

### Physics truth gate
`physics_truth` asserts the laws the simulator depends on: free-fall gravity,
cylinder inertia, restitution, rolling kinematics (`v = ω·r`), rolling
resistance, motor free-speed and stall torque, BackEMF braking, static and
kinetic friction thresholds, numerical stability, energy conservation, and
revolute anchor holding. This suite is the gate that keeps MFS honest.

---

## 🧩 Roadmap — v16 Modularisation

MPE and MFS are currently a single tree. The plan is to separate them cleanly.

| Milestone | State |
|-----------|-------|
| **v15R2** | Config system + MFS merged *(current)* |
| **v15S** | Stabilisation — final **merged** release |
| **v16R1** | Begin splitting MFS out of the MPE mainframe |
| **v16+** | MPE kernel + module ecosystem |

Beginning at **v16R1** (after **v15S**):

- **MPE returns to being a standalone physics-engine kernel**, but gains a
  **kernel-module plugin state system** — a defined host into which modules
  register their state and hooks.
- **MFS becomes the first module ecosystem** built on that plugin system,
  rather than code fused into the engine.
- Within MFS, the plan is a hierarchy of **modules and submodules** — the
  drivetrain, motor, battery, sensor, and odometry layers already present are
  the natural candidates — each loadable against the bare MPE kernel.

**Goal:** run MPE on its own with no robotics present, and drop MFS (or any
future ecosystem) in as a plugin.

<!-- MFS_FTC_SECTIONS_END -->
