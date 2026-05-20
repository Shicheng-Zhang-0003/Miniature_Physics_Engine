# Miniature Physics Engine — Release 1.0-RS

**Release date:** May 2026
**Platform:** Linux (Ubuntu 24.04 LTS, X11)
**Language:** C · GTK3 · OpenGL (libepoxy)

---

## What is MPE?

The Miniature Physics Engine is a from-scratch 3D rigid body physics simulator written entirely in C. Every mathematical component — vectors, quaternions, 3×3 and 4×4 matrix operations, inertia tensors, collision detection, impulse resolution — is hand-implemented with no physics middleware. The renderer uses GTK3 as the window and overlay system with OpenGL via libepoxy for object rendering.

The goal was always simplicity, correctness, and efficiency. No Bullet. No PhysX. No Eigen. Just C and math.

---

## What's in 1.0-RS

### Rigid body physics
- Full linear and angular dynamics with quaternion orientation integration
- Sphere and oriented bounding box (OBB/cube) rigid bodies
- Per-object inertia tensors — spherical (`2/5 mr²`) and box (`m/12 * (h²+d²), ...`)
- Impulse-based collision resolution with restitution and rotational coupling
- Position correction to prevent object sinking under sustained contact
- Sleep thresholds on linear (`< 0.05 m/s`) and angular (`< 0.01 rad/s`) velocity to eliminate floating-point jitter
- Spring joint system with Hooke's law and damping between arbitrary object pairs

### Collision detection
- Sphere–sphere, sphere–OBB, and OBB–OBB narrowphase
- OBB–OBB uses the full 15-axis Separating Axis Test (6 face axes + 9 edge cross products)
- Sweep-and-prune broadphase (SAP) computed once per frame, reused across all 12 substeps
- 12 physics substeps per frame for stable stacking and rolling

### Floor contact
- Spheres: rolling friction with torque generation at the contact point, correct for both kinetic and static regimes
- Cubes: localised force application at the lowest OBB vertex, generating physically correct rolling torque and sliding friction

### Rendering
- Sphere mesh generated procedurally at startup with configurable resolution
- Cube mesh with 24 face vertices and correct per-face normals
- Equatorial axis rings painted directly onto object surfaces via local-space shader logic — red (X), green (Y), blue (Z) — making rotation immediately readable
- GLSL vertex and fragment shaders with ambient, diffuse, and specular lighting
- Wireframe overlay on selected objects (sphere outline / cube edge rings)
- Reference grid in debug mode

### Character / camera
- Game Mode: grounded WASD with horizontal inertia, gravity, jumping, world boundary enforcement
- Debug Mode: free-flying WASD in look direction, no gravity, no boundaries, IJKL fallback steering without a mouse
- Mouse lock/unlock via left click / Escape

### Scene persistence
- Save and load full scenes to `status/scene.dat`
- Format encodes type, mass, radius, half-extents, position, velocity, angular velocity, orientation (quaternion), colour, restitution, and static state
- Spheres and cubes both round-trip correctly

### Menus and runtime configuration
- Spawner menu: sphere mass/radius, cube mass/size, spawn type toggle
- World menu: gravity, air drag coefficient, surface friction
- Viewpoint menu: movement speed, jump height
- Per-object menu: mass, radius (spheres), friction, immovable toggle
- All numeric inputs use a GTK dialog for direct value entry rather than increment-only controls
- Adjustment rate (for toggle-style controls) tunable at runtime via Left/Right arrows; finer in Debug Mode

---

## Performance

Tested on Core Ultra 5 125H · Intel Arc Graphics · 32 GB RAM · Ubuntu 24.04.4 LTS · 2880×1880 · 120 Hz · X11

- ~105 MB RAM at startup
- ~1 MB additional RAM per 1136 objects
- Broadphase O(n log n) sort + O(n) sweep; narrowphase and integration O(n) per substep
- Stable at 60 FPS well past 500 simultaneous dynamic objects

---

## Installation

```bash
sudo apt install gcc make libgtk-3-dev libepoxy-dev
cd src
make
./engine
```

**Note on Wayland:** Mouse locking does not work correctly under native Wayland. Run under X11, or launch with `GDK_BACKEND=x11 ./engine`.

---

## Known issues

**Ray selection on cubes** uses a bounding sphere approximation. Clicks near cube corners may not register if they fall outside the bounding sphere radius. Precise OBB ray casting is planned for a future release.

**Wayland mouse lock** is broken by design in the current GTK3 pointer warp approach. A Wayland-native pointer lock using `zwp_pointer_constraints_v1` would be the correct long-term fix.

**Object count ceiling** — performance degrades gradually above ~1136 simultaneous objects. Rendering is the primary bottleneck at that scale.

---

## Project history

| Phase | Versions | Summary |
|---|---|---|
| 1 | 0.9.0 – 0.9.3-STBL | Core math, GTK window, basic sphere physics |
| 2 | 0.9.4-Alpha | Physics + rendering combined, scene system |
| 3 | 0.9.4 – 0.9.8 | Cube support, OBB collision, rolling friction, save/load, character movement |
| 4 | 0.9.9 – 1.0-RS | Stability pass, menu overhaul, axis rings, codebase reorganisation |

All mathematical components were written from scratch. No physics or linear algebra libraries were used at any stage.
