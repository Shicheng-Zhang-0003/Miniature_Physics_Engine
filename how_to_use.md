# Miniature Physics Engine — User Guide
### Version 1.0-RS

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
| Space | Jump | — |
| Escape | Release mouse | Release mouse |
| I J K L | — | Steer camera (mouse-free) |

In Game Mode, releasing WASD does not stop instantly — horizontal momentum bleeds off over a short distance, giving natural arc through the air when jumping while moving.

---

## Spawning Objects

**Shift** spawns an object in front of the camera. Hold Shift for 0.3 seconds to begin rapid-fire spawning.

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

Once selected:

| Input | Action |
|---|---|
| `E` | Open / close the object property menu |
| `F` | Apply an impulse force (launches the object) |
| Middle mouse click | Delete the object |

The object property menu:

```
E → Object Menu
    1 → Mass
    2 → Radius (spheres only — no effect on cubes)
    3 → Friction (kinetic; static auto-set to kinetic + 0.1)
    4 → Immovable toggle (Up/Down to toggle, Enter to save)
```

Mass, radius, and friction open a text input dialog. Immovable objects have zero inverse mass — forces and collisions do not move them, making them useful as static walls or floors.

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
| Physics substeps | 12 per frame | — |

The engine runs at 60 FPS with 12 substeps per frame, giving stable collision resolution for stacked objects and rolling behaviour. Spheres use rolling friction with torque generation at the contact point. Cubes use localised force application at the lowest vertex to generate rotational torque on floor contact.

Objects with velocity below 0.05 m/s and angular velocity below 0.01 rad/s are put to sleep automatically to prevent floating-point jitter.

---

## Object Colour Coding

Each object has painted equatorial axis rings to make rotation visible:

- **Red ring** — lies in the YZ plane, shows rotation around the X axis
- **Green ring** — lies in the XZ plane, shows rotation around the Y axis
- **Blue ring** — lies in the XY plane, shows rotation around the Z axis

Spheres default to blue. Cubes default to orange. Both can have their colours changed in future versions.

---

## Performance

The engine consumes approximately 1 MB of additional RAM per 1136 objects spawned. An initial run uses roughly 105 MB at rest.

Broadphase collision detection (sweep-and-prune) runs once per frame rather than per substep, keeping performance stable at high object counts. Tested on a Core Ultra 5 125H with Intel Arc Graphics at 2880×1880 resolution under X11.

---

## Known Limitations

**Wayland:** Mouse locking does not function correctly under native Wayland. The engine must be run under X11. On systems that default to Wayland, install basic X11 drivers (`xorg`, `xserver-xorg`) and launch the engine in an X11 session. Forcing X11 via `GDK_BACKEND=x11 ./engine` may also work depending on your compositor.

**Object count:** Performance degrades gradually above approximately 1136 objects. The physics and broadphase scale linearly with object count; rendering is the primary bottleneck at high numbers.

**Ray selection on cubes:** Right-click selection on cubes uses a bounding sphere approximation. Clicks near the corners of a cube may not register if they fall outside the bounding sphere. Precise OBB ray selection is a planned improvement.

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
