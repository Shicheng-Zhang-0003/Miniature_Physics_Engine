#!/usr/bin/env python3
"""
MFS 153: Append FTC/MFS sections to readme.md  (DOC-ONLY)
==========================================================
Appends MFS/FTC-specific sections to the existing readme WITHOUT touching
any current content, then adds the v16 modularisation roadmap.

Appended sections:
  1. MFS — FTC Robotics Simulator (capabilities)
  2. Robot controls (G/V/B/N/C/H + touch robot)
  3. Headless test suite table
  4. Physics truth gate
  5. Roadmap — v16 modularisation (MPE kernel + MFS module ecosystem)

Wrapped in MFS_FTC_SECTIONS markers so it is idempotent and removable.
Makes NO changes to source code.

Usage:
    cd <project_root>
    python3 fixes/phase_mfs/153_readme_ftc_sections.py [--dry-run] [--replace]
"""
import sys
import re
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
ROOT = SCRIPT_DIR.parent.parent
README = ROOT / "readme.md"

MARK_BEGIN = "<!-- MFS_FTC_SECTIONS_BEGIN -->"
MARK_END = "<!-- MFS_FTC_SECTIONS_END -->"

DRY_RUN = "--dry-run" in sys.argv
REPLACE = "--replace" in sys.argv


def log(msg):
    print(f"  [153] {msg}")


SECTIONS = """
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
| **v15R3** | Config system + MFS merged *(current)* |
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
"""


def strip_existing(text):
    """Remove a previously-appended MFS block if present."""
    pattern = re.compile(
        re.escape(MARK_BEGIN) + r".*?" + re.escape(MARK_END),
        re.DOTALL,
    )
    return pattern.sub("", text)


def main():
    print("=" * 60)
    print("MFS 153: Append FTC/MFS sections to readme.md (DOC-ONLY)")
    print("=" * 60)
    if DRY_RUN:
        print("  ** DRY RUN — no files modified **\n")

    if not README.exists():
        log(f"[FAIL] readme not found at {README}")
        return 1

    original = README.read_text()

    if MARK_BEGIN in original:
        if REPLACE:
            log("Existing MFS sections found — replacing (--replace)")
            base = strip_existing(original).rstrip() + "\n"
        else:
            log("[SKIP] MFS sections already present (use --replace to refresh)")
            return 0
    else:
        log("No existing MFS sections — appending")
        base = original.rstrip() + "\n"

    new_content = base + SECTIONS

    if DRY_RUN:
        log(f"[DRY RUN] Would write {len(new_content)} bytes to readme.md")
        log("[DRY RUN] Preview of appended section headers:")
        for line in SECTIONS.split("\n"):
            if line.startswith("#"):
                log("    " + line)
        return 0

    README.write_text(new_content)
    log(f"[OK] readme.md updated ({len(original)} → {len(new_content)} bytes)")
    log("[OK] Existing content left untouched; MFS sections appended at end.")
    print("=" * 60)
    return 0


if __name__ == "__main__":
    sys.exit(main())
