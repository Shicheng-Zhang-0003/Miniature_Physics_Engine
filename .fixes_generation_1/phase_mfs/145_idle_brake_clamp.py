#!/usr/bin/env python3
"""
MFS 145: Idle back-EMF brake clamp — stop the ±25 rad/s wheel oscillation
===========================================================================
Root cause (from 142/143/144):
  At command=0 the motor's back-EMF current clamps at ±17 A -> ±2.17 N·m.
  Against the light wheel inertia (I≈0.00025 kg·m²) at dt=1/60 s, that
  torque reverses the wheel every step -> limit-cycle at ±25 rad/s.
  The sign convention is CORRECT (confirmed in 144); it's an overshoot.

Physically-truthful fix:
  Back-EMF braking is a damper. It can bring a wheel to rest but can never
  reverse it (no back-EMF once stopped). So at idle, clamp the braking
  torque to I*|wheel_speed|/dt = the torque that stops the wheel within
  this timestep. Driving (command≠0) is untouched.

Usage:
    cd <project_root>
    python3 fixes/phase_mfs/145_idle_brake_clamp.py [--dry-run]
"""
import sys
import subprocess
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
ROOT = SCRIPT_DIR.parent.parent
SRC = ROOT / "v15R3" / "src"
TOOLS = ROOT / "tools"
sys.path.insert(0, str(TOOLS))
DRY_RUN = "--dry-run" in sys.argv

def log(msg): print(f"  [145] {msg}")

ROBOT_C = SRC / "robotics" / "robot.c"

CLAMP = '''{indent}/* MFS_145_IDLE_BRAKE: back-EMF braking is a damper — it brings a coasting
{indent} * wheel to rest and can never reverse it (no back-EMF once stopped).
{indent} * At idle, clamp the braking torque to the amount that stops the wheel
{indent} * within this timestep. Without this, the stall-clamped back-EMF torque
{indent} * (~2.17 N·m) reverses the light wheel every step -> ±25 rad/s idle spin. */
{indent}if ((fabsf(robot->wheel_motors[i].command) < 0.05f) && ((torque * wheel_speed) < 0.0f)) {{
{indent}    float mfs_i_axle = 0.5f * wheel->mass * wheel->radius * wheel->radius;
{indent}    if (mfs_i_axle > 0.0f) {{
{indent}        float mfs_max_brake = mfs_i_axle * fabsf(wheel_speed) / dt;
{indent}        if (fabsf(torque) > mfs_max_brake) {{
{indent}            torque = (torque > 0.0f) ? mfs_max_brake : -mfs_max_brake;
{indent}        }}
{indent}    }}
{indent}}}'''


def step_insert_clamp():
    log("Step 1: Inserting idle-brake clamp into ftc_robot_update")
    content = ROBOT_C.read_text()
    if "MFS_145_IDLE_BRAKE" in content:
        log("  [SKIP] already present")
        return True

    lines = content.split("\n")
    target = None
    for i, ln in enumerate(lines):
        if "float torque = robot->wheel_motors[i].output_torque;" in ln:
            target = i
            break
    if target is None:
        log("  [FAIL] torque assignment line not found in robot.c")
        return False

    # detect indentation of the torque line
    indent = ""
    for ch in lines[target]:
        if ch in " \t":
            indent += ch
        else:
            break

    clamp_lines = CLAMP.format(indent=indent).split("\n")
    new_lines = lines[:target + 1] + clamp_lines + lines[target + 1:]
    if not DRY_RUN:
        ROBOT_C.write_text("\n".join(new_lines))
    log(f"  [OK] clamp inserted after line {target + 1} (indent={len(indent)})")
    return True


def step_verify_idle():
    log("Step 2: Rebuild + run idle diagnostic (142)")
    r = subprocess.run(["make", "-C", str(SRC), "test_idle_spin_diag"],
                       capture_output=True, text=True, timeout=180)
    out = r.stdout
    print(out[-2200:] if out else "")
    if r.returncode != 0:
        print(r.stderr[-1500:] if r.stderr else "")
        log("[FAIL] idle diag build/run failed")
        return False
    # parse max wheel omega
    for line in out.split("\n"):
        if "max |wheel axle omega|" in line:
            log("  " + line.strip())
    if "wheels SPIN at idle" in out:
        log("[WARN] wheels STILL spinning at idle — clamp insufficient")
        return False
    log("[PASS] wheels no longer spinning at idle")
    return True


def step_truth_suite():
    log("Step 3: Run physics truth suite (regression check)")
    r = subprocess.run(["make", "-C", str(SRC), "test_physics_truth"],
                       capture_output=True, text=True, timeout=240)
    tail = r.stdout[-1400:]
    print(tail)
    if "Failed: 0" not in tail:
        log("[WARN] truth suite has failures — review")
        return False
    log("[PASS] truth suite still green")
    return True


def main():
    print("=" * 60)
    print("MFS 145: Idle Back-EMF Brake Clamp")
    print("=" * 60)
    if DRY_RUN:
        print("  ** DRY RUN — no files modified **\n")

    if not step_insert_clamp():
        return 1
    if DRY_RUN:
        log("[DRY RUN] skipping build/verify")
        return 0
    ok_idle = step_verify_idle()
    ok_truth = step_truth_suite()

    print("=" * 60)
    if ok_idle and ok_truth:
        print("  145 complete. Idle wheels now rest. Robot holds still.")
        print("  Ready for Milestone 4 (odometry) on a stable robot.")
    else:
        print("  145 needs review — see [WARN]/[FAIL] above.")
    print("=" * 60)
    return 0 if (ok_idle and ok_truth) else 1


if __name__ == "__main__":
    sys.exit(main())
