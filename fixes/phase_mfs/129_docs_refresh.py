#!/usr/bin/env python3
"""
MFS 129: Documentation refresh — align all docs with actual codebase state
============================================================================
The codebase has progressed far beyond what the docs describe. This script
rewrites/patches every stale document to match reality:

  1. readme.md          — full rewrite (mecanum passes, sim.c split, cylinders)
  2. FTC_PLAN_MANIFEST  — Milestone 2 → COMPLETED, Milestone 3 partial
  3. scope.md           — add resolution header, mark ~30 items resolved
  4. how_to_use.md      — add robot controls, cylinders, touch robot
  5. evolution.txt      — update lineage
  6. docs/phase_a/      — mark simulation_c_audit as stale
  7. RELEASE_GATES.md   — add FTC gate section
  8. RELEASE_POLICY.md  — mention FTC pivot

Usage:
    cd <project_root>
    python3 fixes/phase_mfs/129_docs_refresh.py [--dry-run]
"""

import sys
import subprocess
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
ROOT = SCRIPT_DIR.parent.parent
SRC = ROOT / "v15R2" / "src"
TOOLS = ROOT / "tools"

sys.path.insert(0, str(TOOLS))

DRY_RUN = "--dry-run" in sys.argv


def log(msg):
    print(f"  [129] {msg}")


def write_file(path: Path, content: str, label: str):
    if DRY_RUN:
        log(f"  [DRY RUN] Would write {label} ({len(content)} bytes)")
        return
    path.write_text(content)
    log(f"  [OK] {label} written ({len(content)} bytes)")


def read_file(path: Path) -> str:
    return path.read_text()


# ============================================================
# 1. readme.md — FULL REWRITE
# ============================================================
def step_readme():
    log("Step 1: Rewriting readme.md")
    content = """\
# MPE FTC Simulator — README
> Status: **Milestone 2 (Physics Keystone) complete. Milestone 3 in progress.**
> Original project: Miniature Physics Engine v15R2
> Current direction: FTC robotics simulator with real physics, headless testing,
> and future FTC-style hardware abstraction.

---

## 1. What This Project Is

This repository is an FTC-oriented robotics simulator built on top of the
Miniature Physics Engine (v15R2). The long-term goal is to provide an FTC
simulation environment comparable in spirit to what WPILib simulation provides
for FRC:

- repeatable autonomous testing
- realistic drivetrain behaviour (real cylinder wheels, anisotropic roller friction)
- motors with BackEMF, gear ratios, battery voltage sag
- headless tests suitable for CI
- eventually, a user-facing robot-programming API (HardwareMap / OpMode)

---

## 2. Current Test Status

All 8 headless tests **PASS**:

| Test | Status | What it proves |
|---|---|---|
| `two_world` | PASS | Separate `physics_world` instances are independent |
| `revolute` | PASS | Revolute joint holds anchor, allows swing |
| `teleop_drive` | PASS | Robot drives forward under motor power |
| `mecanum_drive` | PASS | Robot strafes via **real anisotropic roller friction** |
| `cylinder_drop` | PASS | Cylinder rests on floor (narrowphase works) |
| `driven_wheel` | PASS | Torque → friction → translation (grounded wheel rolls) |
| `math3_inverse` | PASS | Matrix inverse handles small inertia tensors |
| `ftc_integration` | PASS | Full drive/turn/strafe sequence |

---

## 3. Repository Layout

```text
v15R2/src/
  core/
    physics_world.c/.h          Multi-world physics state + full pipeline
    rigidbody.c/.h              Body data, integration, cylinder support
    simulation_camera.c/.h      Camera tick (extracted from simulation.c)
    simulation_physics_loop.c/.h Fixed-timestep physics loop
    validation_report.c/.h      F9 status report
    long_run_validation.c/.h    F10 validation
    physics_halt.c              Halt state
    math3D.h / math4_special.h  Linear algebra
    event_log.c/.h              Engine event ring buffer

  physics/
    collision_mechanics.c/.h    Narrowphase + solver + mecanum friction
    broadphase.c/.h             Spatial hash grid
    constraint.c/.h             Constraint pool + dispatch
    revolute_joint.c/.h         Revolute solver + Baumgarte axis drift fix
    spring_joint.c/.h           Legacy spring joints
    depenetration.c/.h          Positional depenetration

  robotics/
    robot.c/.h                  FTC robot aggregate (chassis + 4 cylinder wheels)
    drivetrain.c/.h             Tank + Mecanum IK, real traction
    motor.c/.h                  DC motor model (BackEMF, Kt, Kv, gearing)
    motor_presets.c/.h          5 goBILDA/REV presets
    battery.c/.h                Voltage sag model (12.8V, 0.015Ω, 30Ah)
    gui_robot_registry.c/.h     GUI proxy sync + fixed-timestep accumulator

  ui_input/
    simulation_input_dispatch.c  Keyboard bindings (G/V/B/N/C/H robot drive)
    simulation_menu_dispatch.c   Scene menu handling
    editor_dialog.c              Numerical input dialog
    debug_terminal.c             POSIX-style terminal (146 KB)
    microvim.c/.h                Modal text editor
    overlay.c                    HUD overlay
    input_control.c/.h           GTK key/mouse wiring
    camera.c/.h                  Camera math
    object_spawner.c/.h          Spawn logic
    object_selector.c/.h         Raycast selection
    mouse_lock.c/.h              X11 pointer grab
    config_menu.c/.h             Config menu (key 6)

  render/                       OpenGL instanced rendering + shaders
  scene/                        Scene save/load, boundary, ID remap
  config/                       69-parameter config registry

  tests/
    two_world_test.c
    revolute_test.c
    teleop_drive_test.c
    mecanum_drive_test.c
    cylinder_drop_test.c
    driven_wheel_test.c
    math3_inverse_test.c
    ftc_integration_test.c
    ftc_debug_test.c            (diagnostic, not in CI runner)

tools/
  build_check.py                Clean build + compiler diagnostics
  test_runner.py                Headless test runner (all 8 tests)
  refactor.py                   Safe Python refactoring helper
  project_audit.py              Read-only architecture audit

fixes/
  Historical fix scripts (bash + python). Do not run against active code.
```

---

## 4. Build

```bash
cd v15R2/src
make clean && make
```

Preferred project-level build check:

```bash
python3 tools/build_check.py
```

---

## 5. Test

```bash
python3 tools/test_runner.py            # all 8 tests
python3 tools/test_runner.py --list     # show available tests
python3 tools/test_runner.py mecanum    # filtered
```

All 8 tests pass. No expected failures.

---

## 6. Robot Controls (GUI)

| Key | Action |
|---|---|
| `T` (terminal) → `touch robot` | Spawn FTC robot at (5, rest_height, 5) |
| `G` | Drive forward |
| `B` | Drive backward |
| `V` | Strafe right |
| `N` | Strafe left |
| `C` | Rotate left (CCW) |
| `H` | Rotate right (CW) |

The robot uses goBILDA 5203 30:1 motors, mecanum drivetrain, cylinder wheels
with real anisotropic roller friction. An orange nose sphere indicates heading.

---

## 7. Development Rules

- Use `tools/refactor.py` for scripted source edits
- Use `tools/build_check.py` after every structural change
- Use `tools/test_runner.py` after every physics/robotics change
- Do not fake physics to make tests pass
- Do not reintroduce chassis-force cheats for mecanum

---

## 8. Current Architectural State

### Resolved (since original audit)
- `simulation.c` split from 1123 → ~130 lines (9 modules extracted)
- Cylinder narrowphase + inertia implemented
- Mecanum strafe via real anisotropic roller friction (no chassis cheat)
- Motor gear ratio pipeline fixed (no double-application)
- Revolute axis drift corrected (Baumgarte β=0.1)
- Fixed-timestep accumulator for robot physics (60 Hz)
- GUI robot proxy sync (physics_world → obj_per_scene)
- All dead code removed (rb_integrate, define_forces, frame_timer.c, etc.)

### Remaining debt
- `debug_terminal.c` is 146 KB (single translation unit)
- Global `obj_per_scene` / `object_count` still used by GUI path
- Scene save/load doesn't persist joints, robots, or cylinders
- No solver islanding (PHYS-001)
- No CCD (PHYS-002)
- No rolling resistance (PHYS-004)
- No SIMD (PERF-001)
- No frustum culling (PERF-003)

---

## 9. Roadmap

### Phase A — Stabilize ✅
Python tooling green. Bash scripts retired. Test status clear.

### Phase B — Split simulation.c ✅
9 modules extracted. Physics usable headlessly.

### Phase C — Real Drivetrain Physics ✅
Cylinder wheels. Anisotropic friction. Mecanum via roller contact.
Motor model correct. Battery sag. Revolute joints with axis correction.

### Phase D — Solver Hardening (in progress)
- [ ] Per-world contact cache
- [ ] Solver islanding
- [ ] Rolling resistance
- [ ] Warm-starting for constraints

### Phase E — Sensors & Closed-Loop Control
- [ ] Encoders, IMU, distance sensors
- [ ] PID controller
- [ ] Motor velocity/position modes

### Phase F — Mechanisms
- [ ] Prismatic joints
- [ ] Arms, slides, servos, intakes

### Phase G — FTC Platform & API
- [ ] Robot JSON loader
- [ ] HardwareMap abstraction
- [ ] OpMode lifecycle
- [ ] Gamepad model, telemetry

---

## 10. Project Philosophy

This simulator prioritises truthful physics over convenient demos.
A robot that drives correctly here should behave the same way on a real
FTC field. Motor commands become torque. Torque moves wheels. Wheels grip
the floor through contact friction. Sensors read from simulated state.
User robot code should not know whether it is running against real or
simulated hardware.
"""
    write_file(ROOT / "readme.md", content, "readme.md")


# ============================================================
# 2. FTC_PLAN_MANIFEST.md — update milestone status
# ============================================================
def step_manifest():
    log("Step 2: Updating FTC_PLAN_MANIFEST.md")
    path = ROOT / "fixes" / "FTC_PLAN_MANIFEST.md"
    content = read_file(path)

    # Update status line
    content = content.replace(
        "**Current Status:** 🟢 **GREEN** (77/77 fix scripts passing, 0 build failures)",
        "**Current Status:** 🟢 **GREEN** (all 8 headless tests passing, 0 build failures)"
    )
    content = content.replace(
        "**Active Phase:** Milestone 2 (The Physics Keystone)",
        "**Active Phase:** Milestone 3 (Solver Hardening)"
    )

    # Mark Milestone 2 items as done
    content = content.replace(
        "## Milestone 2: The Physics Keystone 🟢 [ACTIVE]",
        "## Milestone 2: The Physics Keystone ✅ [COMPLETED]"
    )
    content = content.replace(
        "- [ ] **090: Cylinder Headers**",
        "- [x] **090: Cylinder Headers**"
    )
    content = content.replace(
        "- [ ] **091: Cylinder Implementation**",
        "- [x] **091: Cylinder Implementation**"
    )
    content = content.replace(
        "- [ ] **092: Cylinder–Floor Narrowphase**",
        "- [x] **092: Cylinder–Floor Narrowphase**"
    )
    content = content.replace(
        "- [ ] **093: Cylinder–Object Narrowphase**",
        "- [x] **093: Cylinder–Object Narrowphase** *(cylinder-floor done; cylinder-vs-sphere/cube deferred)*"
    )
    content = content.replace(
        "- [ ] **094: Anisotropic Friction Model**",
        "- [x] **094: Anisotropic Friction Model**"
    )
    content = content.replace(
        "- [ ] **095: Real Drivetrain Rebuild**",
        "- [x] **095: Real Drivetrain Rebuild**"
    )
    content = content.replace(
        "- [ ] **096: Drivetrain Validation**",
        "- [x] **096: Drivetrain Validation**"
    )

    # Mark Milestone 3 item 100 as partially done
    content = content.replace(
        "- [ ] **100: Iterative Constraint Solve**",
        "- [x] **100: Iterative Constraint Solve** *(constraint_solve_all moved inside solver iteration loop; warm-starting deferred)*"
    )

    # Update Milestone 1 note about mecanum
    content = content.replace(
        "(Note: Mecanum currently uses a chassis-force workaround due to sphere wheels).",
        "(Note: Mecanum now uses real anisotropic roller friction on cylinder wheels.)"
    )

    # Add post-milestone-2 fixes
    m3_header = "## Milestone 3: Solver Hardening"
    insert_text = """
### Post-Milestone-2 Fixes (097–128)
- [x] **100–106:** GUI robot bridge, proxy sync, spawn at (5,y,5)
- [x] **107–114:** Drive key mapping (G/V/B/N/C/H), motor gear ratio fix, battery tuning
- [x] **117:** Real traction (torque → ground force, clamped by friction)
- [x] **118–121:** Traction block repairs, config include fix
- [x] **122:** Physics pipeline fix (motor gearing, broadphase cylinder, fixed timestep)
- [x] **123–124:** Input held-state, velocity cap, damping balance
- [x] **125–125a:** Heading indicator (orange nose sphere)
- [x] **126–126a:** Revolute axis drift fix (Baumgarte β=0.2→0.1)
- [x] **127–127a:** Camera float fix, strafe diagnostics
- [x] **128–128a:** Strafe diagnostic scope fix

"""
    content = content.replace(m3_header, insert_text + m3_header)

    write_file(path, content, "FTC_PLAN_MANIFEST.md")


# ============================================================
# 3. scope.md — add resolution header
# ============================================================
def step_scope():
    log("Step 3: Patching scope.md header")
    path = ROOT / "scope.md"
    content = read_file(path)

    header = """\
# v15R2 — Complete Defect & Debt Audit

> **STATUS UPDATE (post-Milestone-2):**
> This audit was taken before the FTC physics-first pivot. Approximately 30+
> items listed below have since been resolved. Key resolutions:
>
> - **ARCH-002** (simulation.c 44KB god file): RESOLVED — split to ~130 lines
> - **ARCH-009** (monolithic physics_step_increment): RESOLVED — extracted
> - **ARCH-012–017** (dead code): ALL RESOLVED — removed
> - **BUG-001–012**: ALL RESOLVED
> - **PHYS-003** (only spring joints): RESOLVED — revolute joints implemented
> - **REND-010** (render_init every frame): RESOLVED
> - **TERM-010** (MicroVim line length): RESOLVED
> - **QUAL-002** (spelling): RESOLVED
> - **DOC-001–008**: MOSTLY RESOLVED
>
> Still open: PHYS-001 (islanding), PHYS-002 (CCD), PHYS-004 (rolling friction),
> PERF-001 (SIMD), PERF-003 (frustum culling), ARCH-001 (partial), ARCH-003
> (debug_terminal 146KB), ARCH-006 (scene format), SAVE-001–008.

"""
    content = content.replace(
        "# v15R2 — Complete Defect & Debt Audit\n",
        header,
        1
    )
    write_file(path, content, "scope.md")


# ============================================================
# 4. how_to_use.md — add robot section
# ============================================================
def step_how_to_use():
    log("Step 4: Patching how_to_use.md")
    path = ROOT / "v15R2" / "how_to_use.md"
    content = read_file(path)

    robot_section = """
---

## FTC Robot (MFS)

### Spawning a Robot
Open the debug terminal (`T` in debug mode) and type:
```
touch robot
```
This spawns a 4-wheel mecanum robot at position (5, rest_height, 5) with
goBILDA 5203 30:1 motors. An orange nose sphere shows the heading (+Z local).

### Driving the Robot
| Key | Action |
|---|---|
| `G` | Drive forward |
| `B` | Drive backward |
| `V` | Strafe right |
| `N` | Strafe left |
| `C` | Rotate left (CCW) |
| `H` | Rotate right (CW) |

Keys are held-state: press and hold to drive, release to stop.
The robot HUD in the top-left overlay shows battery voltage and average RPM.

### Physics Model
- **Wheels:** Cylinders (radius 0.05m, half-width 0.02m) with revolute joints
- **Friction:** Anisotropic roller friction for mecanum (±45° rollers)
- **Motors:** BackEMF, Kt/Kv, gear ratio, thermal accumulation
- **Battery:** 12.8V nominal, 0.015Ω internal resistance, 30Ah capacity
- **Traction:** Torque → ground force clamped by friction (no chassis cheat)
- **Timestep:** Fixed 60Hz accumulator (deterministic)

### Object Types
The engine supports three object types:
- **Sphere** — spawned via `touch new.sph` or spawner menu
- **Cube** — spawned via `touch new.cube` or spawner menu
- **Cylinder** — used for robot wheels (axle along local X)

"""
    # Insert before "## Installation" section
    anchor = "## Installation (Ubuntu 24.04 LTS)"
    if anchor in content:
        content = content.replace(anchor, robot_section + "\n---\n\n" + anchor, 1)
    else:
        content += robot_section

    write_file(path, content, "how_to_use.md")


# ============================================================
# 5. evolution.txt — update
# ============================================================
def step_evolution():
    log("Step 5: Updating evolution.txt")
    path = ROOT / "v15R2" / "evolution.txt"
    content = """\
MPE Evolution Lineage
=====================

V0.9-Buffer:  Direct Translation Layer from V1.5RC2 of MPE
V1.5RC2:      v15R2 — Configuration System (69 tunables, registry, menu)
MFS:          MPE FTC Simulator — Physics-first pivot
              Milestone 1: Foundation & Robotics Core ✅
              Milestone 2: Physics Keystone (cylinders, anisotropic friction) ✅
              Milestone 3: Solver Hardening (in progress)

Current Head: MFS (post-Milestone-2)
"""
    write_file(path, content, "evolution.txt")


# ============================================================
# 6. docs/phase_a/simulation_c_audit.md — mark stale
# ============================================================
def step_phase_a_audit():
    log("Step 6: Marking phase_a audit as stale")
    path = ROOT / "docs" / "phase_a" / "simulation_c_audit.md"
    content = read_file(path)

    stale_header = """\
> ⚠️ **STALE DOCUMENT** — This audit was taken before the increment split.
> `simulation.c` has since been reduced from 1123 lines to ~130 lines.
> All 16 functions listed here have been extracted into 9 separate modules.
> This document is retained for historical reference only.

---

"""
    content = stale_header + content
    write_file(path, content, "simulation_c_audit.md")


# ============================================================
# 7. RELEASE_GATES.md — add FTC section
# ============================================================
def step_release_gates():
    log("Step 7: Patching RELEASE_GATES.md")
    path = ROOT / "v15R2" / "RELEASE_GATES.md"
    content = read_file(path)

    ftc_section = """
### 14. FTC Robotics (MFS)
- [X] `ftc_robot_create` spawns chassis + 4 cylinder wheels with revolute joints
- [X] Mecanum strafe works via real anisotropic roller friction (no chassis cheat)
- [X] Tank drive works via motor torque → wheel traction
- [X] Motor model: BackEMF, gear ratio, Kt/Kv correct
- [X] Battery voltage sag under multi-motor load
- [X] All 8 headless tests pass (`python3 tools/test_runner.py`)
- [X] Robot visible in GUI via proxy sync
- [X] Robot drivable via G/V/B/N/C/H keys
- [X] Fixed-timestep accumulator (60Hz deterministic)
- [X] Revolute axis drift corrected (Baumgarte)
- [ ] Scene save/load preserves robot assemblies
- [ ] Prismatic joints (arms/slides)
- [ ] Sensors (encoders, IMU, distance)
- [ ] FTC HAL (HardwareMap, OpMode)

"""
    # Insert before "## Release Decision"
    anchor = "## Release Decision"
    if anchor in content:
        content = content.replace(anchor, ftc_section + "\n---\n\n" + anchor, 1)
    write_file(path, content, "RELEASE_GATES.md")


# ============================================================
# 8. RELEASE_POLICY.md — mention FTC
# ============================================================
def step_release_policy():
    log("Step 8: Patching RELEASE_POLICY.md")
    path = ROOT / "v15R2" / "RELEASE_POLICY.md"
    content = read_file(path)

    content = content.replace(
        "## Cycle Goal\nThe v15 series introduces the centralised configuration system.",
        "## Cycle Goal\nThe v15 series introduced the centralised configuration system.\n"
        "The current cycle (MFS) is the **FTC Simulator pivot**: real cylinder wheels,\n"
        "anisotropic mecanum friction, motor/battery models, and headless test infrastructure."
    )

    # Add FTC to accepted changes
    content = content.replace(
        "5. Validation improvements for the new system.",
        "5. Validation improvements for the new system.\n"
        "6. FTC robotics: robot creation, drivetrain, motor, battery, sensors.\n"
        "7. Cylinder physics: narrowphase, inertia, anisotropic friction.\n"
        "8. Constraint framework: revolute joints, axis correction.\n"
        "9. Headless test infrastructure and CI tooling."
    )

    write_file(path, content, "RELEASE_POLICY.md")


# ============================================================
# MAIN
# ============================================================
def main():
    print("=" * 60)
    print("MFS 129: Documentation Refresh")
    print("=" * 60)

    if DRY_RUN:
        print("  ** DRY RUN — no files will be modified **")
        print()

    if not SRC.exists():
        print(f"FATAL: Source directory not found: {SRC}")
        return 1

    steps = [
        ("readme.md", step_readme),
        ("FTC_PLAN_MANIFEST.md", step_manifest),
        ("scope.md", step_scope),
        ("how_to_use.md", step_how_to_use),
        ("evolution.txt", step_evolution),
        ("simulation_c_audit.md", step_phase_a_audit),
        ("RELEASE_GATES.md", step_release_gates),
        ("RELEASE_POLICY.md", step_release_policy),
    ]

    for name, func in steps:
        try:
            func()
        except Exception as e:
            print(f"\n[FAIL] Step '{name}' raised: {e}")
            import traceback
            traceback.print_exc()
            return 1

    print()
    if not DRY_RUN:
        log("Running build verification...")
        result = subprocess.run(
            [sys.executable, str(TOOLS / "build_check.py"), "--quick"],
            cwd=str(ROOT), capture_output=True, text=True, timeout=120
        )
        if result.returncode != 0:
            print(result.stdout[-2000:] if result.stdout else "")
            print(result.stderr[-2000:] if result.stderr else "")
            log("[FAIL] Build failed after doc changes (unexpected)")
            return 1
        log("[PASS] Build still clean!")

        log("Running headless tests...")
        result = subprocess.run(
            [sys.executable, str(TOOLS / "test_runner.py")],
            cwd=str(ROOT), capture_output=True, text=True, timeout=300
        )
        print(result.stdout[-3000:] if result.stdout else "")
        if result.returncode != 0:
            log("[WARN] Some tests failed (unexpected for doc-only change)")
        else:
            log("[PASS] All tests still green!")
    else:
        log("[DRY RUN] Skipping build/test verification.")

    print()
    print("=" * 60)
    print("  DONE. 8 documentation files updated:")
    print("    1. readme.md              — full rewrite")
    print("    2. FTC_PLAN_MANIFEST.md   — M2 ✅, M3 partial")
    print("    3. scope.md               — resolution header added")
    print("    4. how_to_use.md          — robot controls + cylinders")
    print("    5. evolution.txt          — lineage updated")
    print("    6. simulation_c_audit.md  — marked stale")
    print("    7. RELEASE_GATES.md     — FTC gate section added")
    print("    8. RELEASE_POLICY.md      — FTC cycle goal")
    print("=" * 60)
    return 0


if __name__ == "__main__":
    sys.exit(main())
