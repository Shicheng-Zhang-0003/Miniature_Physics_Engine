```markdown
# Miniature Physics Engine — User Guide
### Version 1.4 Stable (v14S)

---

## Starting the Engine

From the `src/` directory, run:

```
./engine
```

The window opens in Game Mode by default. Left click anywhere inside the window to lock the mouse. Press Escape to release it.

---

## Modes

The engine has two modes, toggled with the `0` key at any time.

**Game Mode** is the default. Gravity applies to the camera, WASD movement is grounded, you can jump, and the world has boundaries — objects and the camera are contained within a 500×500×500 unit box.

**Debug Mode** removes all boundaries and gravity from the camera. WASD flies freely in the direction you're looking. Use this for placing objects precisely, inspecting scenes from any angle, or spawning objects in mid-air.

The current mode is shown in the top-left status bar.

---

## Camera Controls

| Input | Game Mode | Debug Mode |
|---|---|---|
| Mouse | Look around | Look around |
| W A S D | Walk (grounded, inertia) | Fly in look direction |
| Space | Jump | Fly up |
| Shift | — | Fly down |
| I J K L | — | Steer camera (mouse-free) |
| M | — | Re-lock mouse cursor |
| Escape | Release mouse | Release mouse |

In Game Mode, releasing WASD does not stop instantly — horizontal momentum bleeds off over a short distance, giving natural arc through the air when jumping while moving.

---

## Spawning Objects

**Enter** spawns an object in front of the camera. Hold Enter for 0.3 seconds to begin rapid-fire spawning.

The active spawn type (sphere or cube) is shown in the status bar. To configure spawning, press `8`:

```
8 → Spawner Menu
  1 → Sphere settings
    1 → Mass
    2 → Radius
  2 → Cube settings
    1 → Mass
    2 → Size (half-extent)
  3 → Toggle spawn type (sphere / cube)
```

Each leaf option opens a text input dialog where you type the value directly and press OK.

---

## Selecting Objects

Right click an object to select it. The status bar updates to show the object's type, index number, position, and speed.

Keyboard-only alternative (Debug Mode): press `R` to select via raycast from the camera.

Once selected:

| Input | Action |
|---|---|
| `E` | Open / close the object property menu |
| `F` | Apply an impulse force (launches the object) |
| Middle mouse click | Delete the object |
| `Delete` | Remove the selected object (Debug Mode only) |

The object property menu:

```
E → Object Menu
  1 → Mass
  2 → Radius (spheres only — no effect on cubes)
  3 → Friction (kinetic; static auto-set to kinetic + 0.1)
  4 → Immovable toggle (Up/Down to toggle, Enter to save)
  5 → Mark for Joint
  6 → Link Joint (if another object is marked) / Colour Selection
  7 → Colour Selection (if joint link available)
```

Mass, radius, and friction open a text input dialog. Immovable objects have zero inverse mass — forces and collisions do not move them, making them useful as static walls or floors.

---

## Debug Terminal

In Debug Mode, press `T` or `1` to open the **POSIX-style debug terminal**. In Game Mode the terminal is read-only; mutating commands require Debug Mode.

The terminal presents the physics world as a virtual filesystem:

| Path | Contents |
|---|---|
| `/obj` | All rigid bodies |
| `/joint` | All spring joints |
| `/world` | World variables (gravity, drag, friction) |
| `/camera` | Camera state |
| `/spawner` | Spawner settings |

Common commands:

| Command | Effect |
|---|---|
| `ls`, `ll` | List objects or joints |
| `cat /obj/3` | Inspect object 3 |
| `touch new.sph` | Spawn a sphere |
| `touch new.cube` | Spawn a cube |
| `rm 3` | Delete object 3 |
| `rm -rf /obj/all` | Delete all objects |
| `mv 3 /pos/0/10/0` | Teleport object 3 |
| `mv 3 /vel/0/20/0` | Apply impulse to object 3 |
| `ln 1 2` | Create a spring joint between objects 1 and 2 |
| `ln -s 1 2` | Create a soft spring joint |
| `chmod static 3` | Make object 3 immovable |
| `chown 5.0 3` | Set object 3 mass to 5 kg |
| `kill -STOP 3` | Put object 3 to sleep |
| `kill -CONT 3` | Wake object 3 |
| `kill -9 3` | Delete object 3 |
| `ps aux` | List all objects with state |
| `top` | Show fastest-moving objects |
| `df` | Show capacity usage |
| `export GRAVITY=-2.0` | Change world gravity |

Type `help` for the full command list or `man <command>` for usage. `Ctrl+L` clears the screen. `Escape` closes the terminal.

---

## Validation Tests

The engine includes built-in test keys for stability validation:

| Key | Test |
|---|---|
| F5 | Spawn a 10-cube stability stack |
| F6 | Spawn sleeping cube + moving projectile (sleep/wake test) |
| F7 | Editor torture test: select, joint, delete, reset |
| F8 | Spawn stress test: up to 300 mixed objects |
| F9 | Print validation report to console |
| F10 | Long-run validation: 3600 ticks (60 seconds) of idle stability |

F10 spawns a predefined scene (stack + pile + spheres) and monitors for NaN values, fallen objects, and residual motion over 60 seconds, printing `PASS` or `FAIL` to the console at completion.

---

## World Settings

Press `7` to open the world and viewpoint settings:

```
7 → Settings Menu
  1 → Spawning
    1 → Launch velocity
    2 → Spawn friction (applied to newly spawned objects)
  2 → Viewpoint
    1 → Movement speed
    2 → Jump height (metres)
  3 → World
    1 → Gravity (m/s², negative = downward)
    2 → Air resistance coefficient (0.1–1.0; lower = more drag)
    3 → Surface friction (floor kinetic friction)
```

**Change rate** controls how much arrow keys adjust values in toggle-style menus (Immovable, spawn type). Left/Right arrow keys change it. In Debug Mode the step is ±0.01; in Game Mode it is ±0.2. The current rate is shown in the status bar.

---

## Saving and Loading

Press `9` to open the scene menu:

```
9 → Scene Menu
  1 → Save current scene
  2 → Load saved scene
  3 → Exit engine
```

Scenes are saved to `status/scene.dat`. Saving overwrites any existing file. Loading clears the current scene and replaces it entirely. Both spheres and cubes are saved and restored correctly, including position, velocity, orientation, colour, mass, friction, restitution, and static state.

**Known limitations:** Spring joints are **not** saved and will be lost on save/load. Object IDs are reassigned on load, so any external references (selection, terminal, joints) to specific IDs will not survive. Sleeping state is not persisted — all objects load awake. These limitations will be addressed in scene format version 2 (post-v14S).

---

## Physics Reference

All values are in SI units (metres, kilograms, seconds).

| Property | Default (Sphere) | Default (Cube) |
|---|---|---|
| Mass | 1.0 kg | 2.0 kg |
| Radius / Size | 0.5 m | 0.5 m half-extent |
| Restitution (bounce) | 0.5 | 0.5 |
| Kinetic friction | 0.2 | 0.3 |
| Static friction | 0.3 | 0.4 |
| World gravity | −9.81 m/s² | — |
| Air drag coefficient | 0.99 | — |
| Physics timestep | 60 Hz fixed | — |

The engine uses a fixed 60 Hz physics timestep with an accumulator, allowing up to 5 physics ticks per rendered frame to prevent spiral-of-death. Each physics tick uses 16 solver iterations, giving stable collision resolution for stacked objects and rolling behaviour. Spheres use rolling friction with torque generation at the contact point. Cubes use localised force application at the lowest vertex to generate rotational torque on floor contact.

Objects with velocity below 0.05 m/s and angular velocity below 0.01 rad/s are put to sleep automatically to prevent floating-point jitter.

---

## Object Colour Coding

Each object has painted equatorial axis rings to make rotation visible:

- **Red ring** — lies in the YZ plane, shows rotation around the X axis
- **Green ring** — lies in the XZ plane, shows rotation around the Y axis
- **Blue ring** — lies in the XY plane, shows rotation around the Z axis

Spheres default to blue. Cubes default to orange. Both can have their colours changed via the object property menu (option 6/7 → Colour Selection).

---

## Performance

The engine consumes approximately 1 MB of additional RAM per 1136 objects spawned. An initial run uses roughly 105 MB at rest.

Broadphase collision detection uses a 3D spatial hash grid and runs once per physics substep, keeping performance stable at high object counts. Tested on a Core Ultra 5 125H with Intel Arc Graphics at 2880×1880 resolution under X11.

---

## Known Limitations

**Wayland:** Mouse locking does not function correctly under native Wayland. The engine must be run under X11. On systems that default to Wayland, install basic X11 drivers (`xorg`, `xserver-xorg`) and launch the engine in an X11 session. Forcing X11 via `GDK_BACKEND=x11 ./engine` may also work depending on your compositor.

**Object count:** Performance degrades gradually above approximately 1136 objects. The physics and broadphase scale linearly with object count; rendering is the primary bottleneck at high numbers.

**Scene format:** Save/load preserves bodies but not spring joints, object IDs, or sleep state. Scene format v2 is planned post-v14S.

---

## Installation (Ubuntu 24.04 LTS)

Install dependencies:

```bash
sudo apt install gcc make libgtk-3-dev libepoxy-dev
```

Build:

```bash
cd src
make
```

Run:

```bash
./engine
```

The engine has been tested on Ubuntu 24.04.4 LTS. Intel MacOS users may attempt to install the same dependencies via Homebrew, but this is unsupported. Windows is not supported.
```

---

## A3_VALIDATION.md — full rewrite for v14S

```markdown
# MPE v14S Validation Checklist

> This checklist was used to validate the `v14S` stable release.
> It is subordinate to [`RELEASE_GATES.md`](RELEASE_GATES.md), which is the authoritative gate list.

<!-- MPE_RELEASE_GATES_SECTION_BEGIN -->
## v14S Release Gates

The stable release is controlled by [`RELEASE_GATES.md`](RELEASE_GATES.md).

Minimum mandatory gates:

- [x] Release freeze policy is active.
- [x] Build passes.
- [x] Startup prints the correct version.
- [x] Shader/render failures are visible.
- [x] Input and focus-loss behaviour is stable.
- [x] Editor deletion/joint torture tests pass.
- [x] Physics stacks settle without explosion.
- [x] Sleeping stacks remain sleeping.
- [x] Overflow counters are visible where applicable.
- [x] F5–F10 validation tests pass.
- [x] Documentation matches code.
- [x] Repository artifacts are cleaned.
- [x] Sanitizer validation passes.

All mandatory P0 gates passed before tagging `v14S`.
<!-- MPE_RELEASE_GATES_SECTION_END -->

This checklist validates the full 1–51 A3 patch sequence plus the S-01 through S-11 stable-release fixes.

## Built-in Test Keys

| Key | Test |
|---|---|
| F5 | Spawn 10-cube stability stack |
| F6 | Spawn sleeping cube + moving projectile |
| F7 | Editor torture test: select, joint, delete, reset |
| F8 | Spawn stress test: up to 300 mixed objects |
| F9 | Print validation report to console |
| F10 | Long-run validation: 3600 ticks (60 seconds) of idle stability |

## Core Lifecycle

- [x] Closing the window quits the program.
- [x] Losing window focus clears stuck input.
- [x] Mouse lock state does not remain stuck after focus loss.

## Editor State

- [x] Deleting selected object does not crash.
- [x] Deleting jointed object does not crash.
- [x] Deleting marked joint object does not crash.
- [x] Loading a scene resets selection/menu/cache state.
- [x] Object menu closes when selection becomes invalid.

## Physics

- [x] Objects rest on floor without explosive jitter.
- [x] Cubes stack with reasonable stability.
- [x] Spheres and cubes collide correctly.
- [x] Objects with restitution bounce on floor.
- [x] Surface friction affects sliding.
- [x] Sleeping objects wake when hit.
- [x] Joints remain valid after deletion.

## Broadphase

- [x] Sleeping objects are still discoverable.
- [x] High object counts do not allocate quadratic dedupe memory.
- [x] Broadphase overflow counters are visible if overflow occurs.
- [x] Node pool grows instead of silently dropping objects.

## Rendering

- [x] Ground grid is visible.
- [x] Wireframe selection renders.
- [x] Joint lines render.
- [x] No per-frame uniform query spam in grid/wireframe/joint paths.
- [x] Shader/render failure does not silently continue (S-01).

## Build

- [x] make -C src clean succeeds.
- [x] make -C src succeeds.
- [x] Startup prints: MPE v14S

## Sanitizer Validation

- [x] AddressSanitizer build compiles and runs.
- [x] UndefinedBehaviorSanitizer build compiles and runs.
- [x] Normal validation passes under sanitizer builds.
- [x] No severe sanitizer errors.

## Final Pass

Run:

```
make -C src clean
make -C src
./src/engine
```

Then manually test:

1. Press F5 — stack settles.
2. Press F6 — sleeping cube wakes on impact.
3. Press F7 — editor torture does not crash.
4. Press F8 — spawn stress remains observable and does not silently fail.
5. Press F9 — validation report prints useful state.
6. Press F10 — long-run validation completes and prints PASS.
7. Save/load scene with menus open — no crash.
8. Delete selected/jointed objects — no crash.
9. Set friction to 0 and restitution above 0 — objects slide and bounce.
10. Open debug terminal (T) — commands execute correctly.

All checks passed. `v14S` tagged.
```
