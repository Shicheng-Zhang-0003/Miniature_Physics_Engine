# MPE v15R2 — Release Candidate 2

| | |
|---|---|
| **Public release / tag** | `V1.5RC2` |
| **Internal designation** | `v15R2` |
| **Release date** | September 2026 |
| **License** | GPL-3.0 |
| **Language / UI / Renderer** | C · GTK3 · OpenGL 3.3 Core |

---

## What is this?

`v15R2` is the second release candidate of the v15 series. It ships **two
layers in a single tree**:

1. **MPE core** — the rigid-body physics engine, now hardened around the
   centralised configuration system introduced in `v15R1`.
2. **MFS** (MPE FTC Simulator) — a first-robotics layer that turns MPE into
   an FTC-oriented robot simulator, currently merged into the engine.

This is the last series where the two live fused together. See the
[Roadmap](#-roadmap--v16-modularisation) for the planned split.

> MFS exists to make autonomous tuning realistic: motor commands become torque,
> torque spins cylinder wheels, wheels grip the floor through anisotropic
> roller friction, and sensors read back from simulated state. Nothing is
> faked at the contact layer.

---

## 🧊 MPE Core — Configuration System

The headline v15 feature, carried over from `v15R1` and hardened in `v15R2`:

- **69 live tunables** in a declarative registry — add a parameter in one line.
- **`mpe_constants.h`** — single source of truth for locked compile-time constants.
- **`mpe_config.h` / `mpe_config_schema.c` / `mpe_config.c`** — typed config
  struct, registry, and init/get/set/clamp/reset/save/load implementation.
- **Persistent storage** to `status/engine.cfg` (save → restart → load round-trip).
- **In-engine config menu** (key `6`) for live parameter editing.
- **Terminal integration** — `env`, `export KEY=value`, `config save|load|reset`.
- **F9 config report** and **F11 config torture test** (randomises all tunables, 60 s idle).

---

## 🤖 MFS — FTC Robotics Simulator

A physically-truthful robotics layer. The drivetrain is a 4-wheel mecanum
platform hinged to a chassis via revolute joints.

### Drivetrain & contact
- **Cylinder wheel bodies** with correct axle inertia (`I = ½·m·r²`).
- **Mecanum strafe** via real **anisotropic roller friction** (±45° rollers) —
  no chassis-force cheats.
- **Tank drive** via motor torque → wheel traction, clamped to the friction limit.
- **Rolling resistance** so a released robot coasts to a stop.
- **Idle hold** — a gearbox-style lock so a parked robot stays parked.

### Power & sensing
- **DC motor model** — BackEMF, gear ratio, Kt/Kv, stall/free-speed limits.
- **goBILDA motor presets** (e.g. 5203 series).
- **Battery model** — voltage sag under multi-motor load.
- **Odometry** — wheel encoders + IMU-style heading for dead reckoning.

### Controls
| Key | Action |
|-----|--------|
| `G` / `B` | Drive forward / backward |
| `V` / `N` | Strafe right / left |
| `C` / `H` | Rotate left / right |

Spawn the robot from the debug terminal with `touch robot`.

---

## ⚙️ Engine & Architecture Improvements

Work that began in MFS and benefits the standalone engine:

- **`physics_world` encapsulation** — multi-world simulation state, decoupled
  from the legacy global scene.
- **Per-world warm-start contact cache** — impulse caching isolated per world.
- **Constraint framework** — revolute (hinge) joints with Baumgarte axis
  correction to prevent drift.
- **Cylinder narrowphase** collision support.
- **Fixed-timestep accumulator** — deterministic 60 Hz robot physics regardless
  of render framerate.
- **Sleeping system** — inactive bodies leave the solver to suppress jitter.

---

## 🧪 Testing & Validation

- **Headless regression suite** — 9 tests, no GTK/OpenGL required:
  `two_world`, `revolute`, `teleop_drive`, `mecanum_drive`, `cylinder_drop`,
  `driven_wheel`, `math3_inverse`, `ftc_integration`, `physics_truth`.
  Run with `python3 tools/test_runner.py`.
- **Physics truth suite** — 24 assertions over the laws the simulator depends
  on: free-fall gravity, cylinder inertia, restitution, rolling kinematics
  (`v = ω·r`), rolling resistance, motor free-speed & stall torque, BackEMF
  braking, static/kinetic friction thresholds, numerical stability, energy
  conservation, and revolute anchor holding.
- **Release validation** — `validation/V01–V04` (sanitizer build, clean build +
  warning review, P0 gate walk, long-run guide).
- **In-engine tests** — `F5` stability stack, `F6` sleep/wake, `F7` editor
  torture, `F8` spawn stress, `F9` report, `F10` 60 s long-run, `F11` config torture.

### The physics-truth gate
MFS is held to a simple standard: **the simulation must tell the truth.** The
`physics_truth` suite is the gate that enforces it — any change that breaks a
physical law fails CI. This is what makes the simulator trustworthy for
autonomous tuning.

---

## 📋 Known Limitations

- **Wayland mouse-lock** is not supported; run under X11 (`GDK_BACKEND=x11`).
- **Scene save/load** preserves bodies but not joints, robot assemblies, stable
  object IDs, or sleep state.
- **Odometry axis convention** is not yet aligned with the physics frame
  (queued); drive-power tuning for full-speed traversal is queued.
- **Global state** remains in the legacy GUI path; full encapsulation is deferred.
- Rendering is the primary bottleneck at high object counts (~1,136 objects).

---

## 🗺️ Roadmap — v16 Modularisation

MPE and MFS are currently one tree. They will be separated cleanly:

| Milestone | State |
|-----------|-------|
| **v15R2** | Config system + MFS merged *(this release)* |
| **v15S** | Stabilisation — final **merged** release |
| **v16R1** | Begin splitting MFS out of the MPE mainframe |
| **v16+** | MPE kernel + module ecosystem |

Beginning at **v16R1**, MPE returns to being a standalone physics-engine
kernel with a **kernel-module plugin state system**, and MFS becomes the
**first module ecosystem** on that plugin system — a hierarchy of modules and
submodules loadable against the bare kernel.

---

## 🔨 Build

```bash
# Dependencies (Ubuntu/Debian)
sudo apt install build-essential pkg-config libgtk-3-dev libepoxy-dev

cd v15R2/src
make clean
make
./engine
```

Headless tests (no display needed):

```bash
python3 tools/test_runner.py
```

---

## 📜 Lineage

- `v15R2` — config-system hardening + MFS robotics (this release).
- `v15R1` — introduced the centralised configuration system.
- `V1.5RC2` buffer — direct translation layer from MPE v1.5.
- See `evolution.txt` for the full lineage back to stage 0.

---

*Release quality criteria: [`RELEASE_GATES.md`](RELEASE_GATES.md). Policy:
[`RELEASE_POLICY.md`](RELEASE_POLICY.md).*
