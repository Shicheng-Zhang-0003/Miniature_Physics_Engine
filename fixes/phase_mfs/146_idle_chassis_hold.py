#!/usr/bin/env python3
"""
MFS 146: Idle chassis hold — stop the +x mecanum drift
========================================================
Root cause (from 145 trace): mecanum contact asymmetry produces a small
net force on a "stationary" robot; lateral damping only limits it to a
0.02 m/s terminal drift (0.1 m in 5 s). Wheels spin at ~0.5 rad/s only
because they're rolling along with the drifting chassis (v = omega*r).

Physically-truthful fix: a real unpowered robot's drivetrain (gearbox
back-drive friction + motor cogging) resists motion and holds position.
Model this as strong horizontal damping on the chassis when all wheel
commands are ~0 and the robot is nearly stopped (<0.25 m/s), so normal
high-speed coast-down stays governed by back-EMF + rolling resistance.

Usage:
    cd <project_root>
    python3 fixes/phase_mfs/146_idle_chassis_hold.py [--dry-run]
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

def log(msg): print(f"  [146] {msg}")

DRIVETRAIN_C = SRC / "robotics" / "drivetrain.c"

IDLE_HOLD = '''{indent}/* MFS_146_IDLE_HOLD: an unpowered real robot's drivetrain (gearbox
{indent} * back-drive friction + motor cogging) resists motion, holding position
{indent} * instead of drifting from mecanum contact asymmetry. Model as strong
{indent} * horizontal chassis damping when all wheel commands are ~0 and the robot
{indent} * is nearly stopped. The <0.25 m/s gate leaves normal high-speed coast-down
{indent} * to back-EMF + rolling resistance. */
{indent}{{
{indent}    int mfs_idle = 1;
{indent}    for (int mfs_wi = 0; mfs_wi < robot->wheel_count; mfs_wi++) {{
{indent}        if (fabsf(robot->wheel_motors[mfs_wi].command) > 0.05f) {{ mfs_idle = 0; break; }}
{indent}    }}
{indent}    if (mfs_idle) {{
{indent}        int mfs_cidx = robot->chassis_body;
{indent}        if ((mfs_cidx >= 0) && (mfs_cidx < world->body_count)) {{
{indent}            rigidbody *mfs_chassis = &world->bodies[mfs_cidx];
{indent}            float mfs_hvx = mfs_chassis->velocity.x;
{indent}            float mfs_hvz = mfs_chassis->velocity.z;
{indent}            float mfs_hs = sqrtf((mfs_hvx * mfs_hvx) + (mfs_hvz * mfs_hvz));
{indent}            if ((mfs_hs > 0.0004f) && (mfs_hs < 0.25f)) {{
{indent}                float mfs_hold_coeff = 20.0f;
{indent}                mfs_chassis->force_accumulator.x -= mfs_hvx * mfs_hold_coeff * mfs_chassis->mass;
{indent}                mfs_chassis->force_accumulator.z -= mfs_hvz * mfs_hold_coeff * mfs_chassis->mass;
{indent}            }}
{indent}        }}
{indent}    }}
{indent}}}'''


def step_show_block():
    log("Step 1: Showing MPE_DRIVETRAIN_REAL block (for reference)")
    content = DRIVETRAIN_C.read_text()
    s = content.find("#if MPE_DRIVETRAIN_REAL")
    e = content.find("#endif /* MPE_DRIVETRAIN_REAL */")
    if s >= 0 and e >= 0:
        block = content[s:e + len("#endif /* MPE_DRIVETRAIN_REAL */")]
        # print just first ~40 lines to keep it readable
        lines = block.split("\n")
        print("\n".join(lines[:40]))
        if len(lines) > 40:
            print(f"  ... ({len(lines) - 40} more lines)")
    else:
        log("  [WARN] MPE_DRIVETRAIN_REAL block not found")
    return True


def step_insert_hold():
    log("Step 2: Inserting idle chassis hold")
    content = DRIVETRAIN_C.read_text()
    if "MFS_146_IDLE_HOLD" in content:
        log("  [SKIP] already present")
        return True

    lines = content.split("\n")
    target = None
    for i, ln in enumerate(lines):
        if "yaw_vel * m * 1.5f" in ln:
            target = i
            break
    if target is None:
        # fallback: any yaw damping line
        for i, ln in enumerate(lines):
            if "torque_accumulator.y -= yaw_vel" in ln:
                target = i
                break
    if target is None:
        log("  [FAIL] yaw-damping anchor not found — printing block above for manual fix")
        return False

    indent = ""
    for ch in lines[target]:
        if ch in " \t":
            indent += ch
        else:
            break

    hold_lines = IDLE_HOLD.format(indent=indent).split("\n")
    new_lines = lines[:target + 1] + hold_lines + lines[target + 1:]
    if not DRY_RUN:
        DRIVETRAIN_C.write_text("\n".join(new_lines))
    log(f"  [OK] idle hold inserted after line {target + 1}")
    return True


def step_verify_idle():
    log("Step 3: Rebuild + run idle diagnostic")
    r = subprocess.run(["make", "-C", str(SRC), "test_idle_spin_diag"],
                       capture_output=True, text=True, timeout=180)
    out = r.stdout
    # show the tail with the summary + verdict
    print(out[-1600:] if out else "")
    if r.returncode != 0:
        print(r.stderr[-1500:] if r.stderr else "")
        log("[FAIL] idle diag build/run failed")
        return False
    for line in out.split("\n"):
        if "max |wheel axle omega|" in line or "max chassis speed" in line or "VERDICT" in line:
            log("  " + line.strip())
    if "wheels SPIN at idle" in out:
        log("[WARN] still spinning — drift not fully killed")
        return False
    log("[PASS] idle stable (no spin, no drift)")
    return True


def step_truth_suite():
    log("Step 4: Run physics truth suite (regression)")
    r = subprocess.run(["make", "-C", str(SRC), "test_physics_truth"],
                       capture_output=True, text=True, timeout=240)
    tail = r.stdout[-900:]
    print(tail)
    if "Failed: 0" not in tail:
        log("[WARN] truth suite has failures")
        return False
    log("[PASS] truth suite green")
    return True


def main():
    print("=" * 60)
    print("MFS 146: Idle Chassis Hold (kill mecanum drift)")
    print("=" * 60)
    if DRY_RUN:
        print("  ** DRY RUN — no files modified **\n")

    step_show_block()
    if not step_insert_hold():
        return 1
    if DRY_RUN:
        log("[DRY RUN] skipping build/verify")
        return 0

    ok_idle = step_verify_idle()
    ok_truth = step_truth_suite()

    print("=" * 60)
    if ok_idle and ok_truth:
        print("  146 complete. Robot holds position at idle — no spin, no drift.")
        print("  Autonomous odometry now has a stable stationary baseline.")
        print("  Ready for Milestone 4 (wheel encoders + odometry).")
    else:
        print("  146 needs review — see [WARN]/[FAIL] above.")
    print("=" * 60)
    return 0 if (ok_idle and ok_truth) else 1


if __name__ == "__main__":
    sys.exit(main())
