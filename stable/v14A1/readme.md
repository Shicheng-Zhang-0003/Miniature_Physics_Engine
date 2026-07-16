# MINIATURE PHYSICS ENGINE (MPE)

## Version 1.4 — "Alpha 2"

---

## 🧠 Overview

MPE is a custom-built 3D rigid body physics engine and rendering pipeline written entirely in **C**.

It is designed with a **zero-dependency core architecture**, excluding only:
- GTK3 (windowing + UI)
- OpenGL (render backend)

The goal of MPE is to prioritize:
- Mathematical transparency
- Cache-efficient data layouts
- Deterministic physics simulation
- High-performance real-time scaling

---

## 🚀 Version 1.4 Alpha 2 Highlights

Version 1.4 Alpha 2 introduces:
- **Warm-Starting Contact Solver**: Persists normal and tangent impulses across frames, initializing sequential iterations with cached forces to reduce micro-jitter and drastically improve stack stability.
- **Multi-Point Contact Manifolds**: Sutherland-Hodgman face-clipping polygon algorithm for OBB-OBB face contacts, generating up to 4 contact points to enable stable resting positions and support stacking.

---

## 🚀 Version 1.4 Alpha RC1 Highlights

Version 1.4 Alpha RC1 introduces:
- **Interactive Constraint/Joint System**: Users can now visually link rigid bodies together using interactive spring joints, creating pendulums and chains.
- **Dynamic Spring Joint Renderer**: Active spring joints are rendered in real time as glowing magenta lines in the OpenGL viewport.
- **Object Color Painting**: Support for customizing rigid body colors from a built-in color preset sub-menu.
- **Robust Constraint Deletion**: Shift correction on joint object indices prevents constraints from breaking when rigid bodies are deleted.
- **Slab-Method OBB Raycast Selection**: Precise slab-method ray-box intersection test for select click accuracy.

---

## 🚀 Version 1.3 Highlights

Version 1.3 represents a major architectural convergence, transitioning MPE from a physics sandbox into a scalable simulation engine capable of handling **10,000+ rigid bodies in real time**.

---

## 🎨 Rendering System

### Hardware Instanced Rendering

MPE eliminates per-object draw calls using GPU instancing.

- CPU packs transformation matrices into contiguous buffers
- GPU handles batch rendering via instanced draw calls
- Scene is rendered in **two draw calls total**:
  - Spheres
  - Cubes

This removes the traditional O(N) CPU draw-call bottleneck.

---

## 🧮 Physics Optimization

### Spatial Hash Grid Broadphase

The previous Sweep-and-Prune system has been replaced with a **3D spatial hash grid**.

Key properties:
- Objects mapped into hashed grid buckets
- Collision checks limited to local neighborhoods
- Average complexity: **O(N)** scaling
- Sleep system removes inactive bodies from simulation

---

### Collision Detection

#### Narrowphase systems:
- Sphere ↔ Sphere: analytical distance test
- Sphere ↔ OBB: closest-point projection
- OBB ↔ OBB: Separating Axis Theorem (15-axis test)

---

### Collision Resolution

Impulse-based solver supporting:
- Static and kinetic friction
- Rolling friction via torque at contact points
- Penetration correction (Baumgarte stabilization)

---

### Integration

- Semi-implicit Euler integration (linear motion)
- Quaternion-based angular integration (no gimbal lock)

---

## 📐 Mathematics Core

MPE includes a fully custom math library:

- 3D vectors
- 4x4 matrices
- quaternions
- inertia tensors

Design goals:
- tightly packed structs
- cache-friendly memory layout
- zero external math dependencies

---

## 🌍 Platform & Rendering Stack

- **Windowing / UI:** GTK3
- **Graphics API:** OpenGL 3.3 Core (via libepoxy)
- **Lighting Model:** Custom GLSL Phong shading
- **Debug Visualization:**
  - Axis indicators
  - rotational torque overlays

---

## 🎮 Controls

| Action | Input |
|------|------|
| Move camera | WASD |
| Look around | Mouse (lock with left click) |
| Jump | Space |
| Sprint / Shift action | Shift |
| Spawn object | Shift (hold) |
| Select object | Right click (raycast OBB/Sphere) |
| Delete object | Middle click |
| Apply force | F |
| Toggle modes | 0 |
| World settings | 7 |
| Spawner settings | 8 |
| Save/Load scene | 9 |

---

## ⚙️ Build Instructions

### Dependencies (Ubuntu / Debian)

```bash
sudo apt update
sudo apt install build-essential pkg-config libgtk-3-dev libepoxy-dev

### To actually build:

cd src
make clean
make
./engine


