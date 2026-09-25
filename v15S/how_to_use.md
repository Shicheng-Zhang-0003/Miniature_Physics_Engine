# Miniature Physics Engine — User Guide
### v15S Guide (GTK4 + modular kernel)

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
| Escape | Release mouse | Release mouse |

In Game Mode, releasing WASD does not stop instantly — horizontal momentum bleeds off over a short distance, giving natural arc through the air when jumping while moving.

---

## Spawning Objects

**Enter** spawns an object in front of the camera. Hold Enter for 0.3 seconds to begin rapid-fire spawning.

The active spawn type (sphere, cube, or cylinder) is shown in the status bar. To configure spawning, press `8`:

```
8 → Spawner Menu
  1 → Sphere settings
    1 → Mass
    2 → Radius
  2 → Cube settings
    1 → Mass
    2 → Size (half-extent)
  3 → Toggle spawn type (sphere / cube / cylinder, Up/Down cycle, Enter confirms)
  4 → Cylinder settings
    1 → Mass
    2 → Radius
    3 → Half-length (axle, local X)
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

In Debug Mode, press `1` to open the **POSIX-style debug terminal**. In Game Mode the terminal is read-only; mutating commands require Debug Mode.

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
| `touch new.cyl` | Spawn a cylinder |
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
| `mod ls` | List loaded physics modules |
| `mod load ./plugins/mpe_capsule.so` | Hot-plug a foreign shape |
| `mod attach capsule-shape` | Attach a module to the world |
| `mod use-solver seq-impulse` | Swap the solver backend |
| `mod load ecosystem/mfs/mfs_ecosystem.so` | Load the MFS robotics bundle |
| `eco attach mfs-simulator` | Attach the bundle to the primary world |
| `eco command mfs-simulator help` | List bundle commands |
| `ftc spawn` | Spawn an FTC robot (5203 26.9 mecanum, at origin) |
| `ftc drive 0 tank 1 1` | Drive robot 0 forward (persists until changed) |
| `ftc drive 0 mecanum 0 1 0` | Strafe robot 0 right |
| `ftc drive 0 stop` | Stop robot 0 |
| `ftc telemetry 0` | Pose, odometry, battery, per-wheel state |
| `ftc list` | List all robots |
| `ftc preset 26.9` | List matching motor presets |
| `eco config mfs-simulator get shooter_rpm` | Read bundle config |

Type `help` for the full command list or `man <command>` for usage. `Ctrl+L` clears the screen. `Escape` closes the terminal.

---

## Terminal Debugger (`mpe-tui`)

A terminal-only companion binary: a live inspector and a scriptable
state-dump suite (needs only ncurses; no GTK/OpenGL):

```bash
cd v15S/src
make mpe-tui
./mpe-tui                         # live ncurses inspector (needs a TTY)
./mpe-tui --snapshot 600          # one full state dump (pipeable, diffable)
./mpe-tui --stream 600 --every 60 # dumps every 60 ticks
./mpe-tui --snapshot 10 --scene tower|pendulum|springlab|f10|demo|stress|ccd
./mpe-tui --snapshot 60 --scene demo --solver seq-impulse --broadphase hash
```

Scenes: `demo` (everything), `tower`, `pendulum`, `springlab` (zero-g
vacuum, scene-local config), `f10` (validation replica), `stress`
(300 mixed bodies + every joint type + CCD ball), `ccd` (60/144/300 m/s
wall battery). Each scene runs on its own config copy, so vacuum and
torture settings never leak between runs.

Live screens (`1`–`5`, `Tab` cycles): body overview, per-object
characteristics + mathematics (quaternion, euler, inertia tensors, momentum,
energy), joint/constraint detail with live endpoint geometry, pairwise scene
graph, help. Keys: `j/k` select, `Space` pause, `s` single-step, `+/-`
time scale, `/` filter, `q` quit. Snapshot sections (`[engine]`, `[body i]`,
`[springs]`, `[constraints]`, `[pairs]`, `[islands]`, `[stats]`,
`[result]`) are fixed-format and deterministic, and report pool usage
(`bodies=n/cap`, `cacheCount=n/cap`). `make tui-smoke` checks
every scene dumps finite state.

---

## Configuration System (Key 6)

Press `6` to open the **Configuration Menu**. This provides live access to all 78 tunable engine parameters.

The menu is organised into 13 categories:

| Category | Parameters |
|---|---|
| World | Gravity, drag, floor friction |
| Timestep | Solver iterations, max substeps, speed clamps |
| Sleep | Sleep/wake thresholds, timer |
| Solver | Penetration slop, bias, restitution, friction |
| Depenetration | Correction factor, max correction, slop |
| Broadphase | Cell size, multiplier, max span |
| Joints | Max acceleration, spring k, damping |
| Boundary | Floor slop values |
| Spawner | Mass, radius, speed, friction |
| Body Defaults | Restitution, friction per type |
| Camera | Speed, sensitivity, jump height |
| Render | Light position, ambient, specular |
| UI | Change rates, spawn timing |

Navigation:
- Select a category number to view its parameters
- Select a parameter number to edit it via dialog
- `0` goes back / more categories
- `6` closes the menu
- Parameters marked `[D]` require Debug Mode to edit

Config is saved to `status/engine.cfg` on exit and loaded on startup.

### Terminal Config Commands

In the debug terminal:
- `env` — list all parameters grouped by category
- `export KEY=value` — set a parameter (e.g., `export world.gravity=-2.0`)
- `config save` — save config to file
- `config load` — reload config from file
- `config reset` — reset all parameters to defaults

### F11 Config Torture Test

Press `F11` to randomise all 78 tunables to extreme bounded values and run
a 60-second long-run validation. This stress-tests the engine under
adversarial parameter combinations (each press uses the next seed, printed
for bisection). F11 is a robustness verdict: PASS means no NaN and nothing
fell through the world — speeds are reported, never gated (under extremes,
perpetual fall/creep can be the TRUE outcome). Solver resolution is pinned
during torture (gravity −17…−1, ≥96 iterations — the proven envelope for the
10:1 validation column; material/world extremes stay fully random). F10 at
defaults keeps the full settle verdict (final < 0.25 m/s, run-max < 2.0 m/s
past the 2 s opening transient). After the test, use
key 6 → Reset Defaults or terminal `config reset` to restore normal behaviour.

### Truth-validation switches

- `sleep.enable = 0` (terminal `export sleep.enable=0`, debug-only) disables
  the sleep optimizer so the solver alone must settle the scene.
- `nice_value` is a per-object settle tool, not physics: keep 0 for truth.
- `world.drag` is linear-viscous retention (`c = −ln(drag)`), not quadratic
  aero; `1.0` is vacuum truth.

---

## Module System (Hot-Plug Physics)

Every pipeline stage is swappable at runtime through the Module
Interface (`src/core/mpe_module.h`, ABI v1):

- **Shapes** — custom bodies (`object_custom`, id ≥ 100) dispatch via the
  pair registry. Example: `plugins/mpe_capsule.c`.
- **Backends** — `hash` broadphase and `seq-impulse` solver are the
  builtins; foreign ones register under their own names.
- **Tick modules** — `pre_step`/`post_step` hooks (force fields, motors,
  loggers) attach per world.

In the debug terminal: `mod ls`, `mod load <file.so>`,
`mod unload <name>`, `mod attach|detach <name>`,
`mod use-broadphase|use-solver <name|builtin>`. In TUI:
`--broadphase NAME --solver NAME`. Each world carries its own config,
so two worlds can run different physics side by side.

Lifetime rules (all enforced, all tested by `loader_lifecycle`):
- Paths are jailed to `plugins/<name>.so` (modules) or
  `ecosystem/mfs/<name>.so` (ecosystem bundles), relative to the
  process working directory (normally `v15S/src`).
- `mod unload` refuses with "busy" while any live world still references
  the code (attached tick module, active stage backend). Detach/reset
  first, then retry. Unload otherwise detaches every live world,
  resets aliasing stage slots to builtin, and purges leftover pair
  handlers pre-`dlclose`.
- Builtins (`hash`, `seq-impulse`, the six collision pairs) refuse
  silent takeover; foreign stages register under their own names.
- Pair handlers must self-unregister in a destructor (see
  `plugins/mpe_capsule.c`); tick hook tables are snapshotted, so hooks
  may attach/detach mid-tick.
- Ecosystem bundles export `mpe_ecosystem_desc` (loader precedence over
  any inner `mpe_module_desc`): `mod load ecosystem/mfs/mfs_ecosystem.so`.

---

## Driving FTC Robots

The MFS robotics bundle (`ecosystem/mfs/mfs_ecosystem.so`) hot-plugs
through the same module system. Full session from the debug terminal:

```
mod load ecosystem/mfs/mfs_ecosystem.so   # load the bundle
eco attach mfs-simulator                  # attach it to the primary world
ftc spawn                                 # mecanum robot, 5203 26.9, at origin
ftc drive 0 tank 1 1                      # full forward (persists until changed)
ftc telemetry 0                           # pose, odometry, battery, wheels
ftc drive 0 stop                          # stop
```

Notes:
- `ftc spawn [preset-substr] [mecanum|tank] [x y z]` — e.g.
  `ftc spawn 50.9 tank 2 0 -3`. Presets list via `ftc preset [substr]`.
- Drive commands persist: the tick module re-applies them every tick
  until `stop` or a new command. The simulation must be stepping (the
  engine steps continuously while open).
- A tile floor is required for traction. `spawn` adds a 20×20 Coulomb
  slab (top y=0, μ 1.0/0.8) automatically when the world has no
  floor-like body, and says so. Without frictional contact, drive
  commands produce slip-regime artifacts instead of motion.
- `ftc list` shows every robot with live odometry; a `SLIP` tag marks
  ticks where roller thrust bypassed the wheel encoders (structurally
  unobservable strafe — fused from chassis motion, flagged honestly).
- Bundle config: `eco config mfs-simulator get shooter_rpm`
  (module_1 game state); bundle verbs via `eco command mfs-simulator
  <spawn|drive|list|telemetry|help> [...]` — the same surface the
  `ftc` commands drive through.
- Detach with `eco detach mfs-simulator`, unload with
  `mod unload ecosystem/mfs/mfs_ecosystem.so` (refused while referenced).

## Validation Tests

The engine includes built-in test keys for stability validation:

| Key | Test |
|---|---|
| F5 | Spawn a 10-cube stability stack |
| F6 | Spawn sleeping cube + moving projectile (sleep/wake test) |
| F7 | Editor torture test: select, joint, delete, reset |
| F8 | Spawn stress test: up to 300 mixed objects (repeatable: each batch stacks above the last) |
| F9 | Print validation report to console |
| F10 | Long-run validation: 3600 ticks (60 seconds) of idle stability |
| F11 | Config torture (robustness verdict — see above) |

F10 spawns a predefined scene (Coulomb floor slab + stack + pile + spheres) and monitors for NaN values, fallen objects, and residual motion over 60 seconds, printing `PASS` or `FAIL` to the console at completion. The floor is load-bearing: without frictional contact the pile disperses instead of settling (measured 0/27 asleep, KE=30 at 60 s); with it the scene settles dead calm (27/27 asleep, KE=0, run-max 0.0). Verdict thresholds: final speeds < 0.25/0.5, post-transient run-max < 2.0.

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
  3 → World (shortcuts into the same values as menu 6)
    1 → Gravity (m/s², negative = downward)
    2 → Air resistance coefficient (0.1–1.0; lower = more drag)
    3 → Surface friction (floor kinetic friction)
    4 → Rolling resistance coefficient
    5 → Solver iterations per tick (1–128)
```

Toggle-style menus (Immovable, spawn type) use `Up`/`Down` to flip and `Enter` to confirm. Change-rate values (`ui.change_rate_*`) remain editable via menu 6 or the terminal.

---

## Saving and Loading

Press `9` to open the scene menu:

```
9 → Scene Menu
  1 → Save current scene
  2 → Load saved scene
  3 → Exit engine
```

Scenes are saved to `status/scene.dat` (v200: LE fields, stable IDs, CRC32 footer, atomic tmp→rename). Saving overwrites any existing file. Loading clears the current scene and replaces it entirely. Bodies (sphere/cube/cylinder incl. position, velocity, orientation, colour, mass, friction, restitution, static/kinematic/sleep/nice_value/stable ID+generation) plus spring joints and revolute/fixed/distance/prismatic/rope joints (anchors, axes, motors, limits) are saved and restored. Files v130/v140/v150/v151/v152/v153 load via the legacy reader (IDs remapped, cylinders become spheres pre-R3-04).

**Joint truth:** the solver supports spring, revolute, fixed, prismatic, distance, and rope constraints (see them live in `mpe-tui --scene demo`). The in-engine menus create spring joints; scene v200 persists springs + revolute/fixed/distance/prismatic/rope. Fixed/distance/prismatic/rope currently have no in-engine creation UI — solver + persistence + TUI-demo only.

**Known limitations:** Per-object config (beyond nice_value) is not persisted. Big-endian hosts unsupported: little-endian only; big-endian fails at compile time (scene_saving.c #error).

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

The engine uses a fixed 60 Hz physics timestep with an accumulator, allowing up to 5 physics ticks per rendered frame to prevent spiral-of-death. Each physics tick runs 64 sequential-impulse solver iterations by default (timestep.solver_iterations, 1–128), giving stable collision resolution for stacked objects and rolling behaviour. Rolling resistance applies contact-patch torque (Hertz patch, shared-patch split for body-body); static/kinetic Coulomb friction uses a two-tangent disc clamp.

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

**Wayland:** supported. The GTK4 port uses Wayland-safe input (no X11
pointer warping); the mouse locks via cursor capture and works under both
Wayland and X11 sessions.

**Scene format:** v200 saves bodies (stable IDs, sleep, damping) plus spring and revolute/fixed/distance/prismatic/rope joints, with CRC32 footer and atomic write. Fixed/distance/prismatic/rope have no in-engine creation UI yet (solver + persistence + TUI only). Files v130/v140/v150/v151/v152/v153 load via the legacy reader. Truth labels: `world.drag` is linear-viscous (not quadratic aero); `nice_value`/`angular_damping_scale` are NON-PHYSICAL settle tools (0/1.0 = truth); `sleep.enable=0` runs sleepless truth validation; boundary walls are a plastic safety net, not material contact.

---

## Object Types
The engine supports four object types:
- **Sphere** — spawned via `touch new.sph`, spawner menu, or `Enter`
- **Cube** — spawned via `touch new.cube`, spawner menu, or `Enter`
- **Cylinder** — spawned via `touch new.cyl`, spawner menu, or `Enter`; axle along local X, correct `I = ½·m·r²` inertia, exact flat-cap contacts, dedicated instanced mesh
- **Custom** (`object_custom`, id ≥ 100) — foreign shapes via hot-plugged pair handlers. Example: capsule (`plugins/mpe_capsule.c`): segment + radius with the bounding invariant `radius = √(h²+rc²)` (the engine owns `radius` as the bounding radius for broadphase; `sanitize` preserves it). Custom bodies render as cubes.



---

## Installation (Ubuntu 24.04 LTS)

Install dependencies:

```bash
sudo apt install gcc make libgtk-4-dev libepoxy-dev
```

Build:

```bash
cd v15S/src
make
```

Run:

```bash
./engine
```

Verification profiles (run from the repository root; no display is needed):

```bash
python3 tools/test_runner.py --profile quick   # harness checks + canonical C suite
python3 tools/test_runner.py --profile physics # plus isolated legacy + paranoia cases
python3 tools/test_runner.py --profile full    # engine, MFS, TUI, ASan + UBSan repeats
python3 tools/test_runner.py --list            # discovered test inventory
```

Every run writes JSON and JUnit summaries and command logs below `temp/qa_runs/`.

The engine has been tested on Ubuntu 24.04.4 LTS. Intel MacOS users may attempt to install the same dependencies via Homebrew, but this is unsupported. Windows is not supported.

For release validation, follow [RELEASE_GATES.md](RELEASE_GATES.md) and the scripts in `validation/`.
