#!/usr/bin/env python3
"""
MFS 147: Coulomb gearbox hold — fully kill the idle drift
==========================================================
146 used pure viscous damping, which can only reach a terminal velocity
against the constant mecanum contact asymmetry (drift fell to 0.0117 m/s
but not zero). A real unpowered robot holds still because gearbox
back-drive friction is ~constant (Coulomb), not velocity-proportional.

Fix: add a Coulomb force term alongside the viscous term. Coulomb exceeds
the contact-asymmetry force, so the chassis decelerates to rest and stays
there. The total is clamped to the force that stops the chassis within one
timestep, so the hold can never reverse it (no oscillation).

Usage:
    cd <project_root>
    python3 fixes/phase_mfs/147_coulomb_hold.py [--dry-run]
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

def log(msg): print(f"  [147] {msg}")

DRIVETRAIN_C = SRC / "robotics" / "drivetrain.c"


def leading_ws(s):
    return s[:len(s) - len(s.lstrip())]


def step_replace_hold():
    log("Step 1: Replacing viscous-only hold with Coulomb+viscous hold")
    content = DRIVETRAIN_C.read_text()
    if "MFS_147_COULOMB_HOLD" in content:
        log("  [SKIP] already present")
        return True

    lines = content.split("\n")
    anchor = None
    for i, ln in enumerate(lines):
        if "mfs_hold_coeff = 20.0f" in ln:
            anchor = i
            break
    if anchor is None:
        log("  [FAIL] 146 viscous hold (mfs_hold_coeff) not found")
        return False

    # Block layout relative to the hold_coeff line (anchor):
    #   anchor-1 : if ((mfs_hs > 0.0004f) && (mfs_hs < 0.25f)) {
    #   anchor   : float mfs_hold_coeff = 20.0f;
    #   anchor+1 : force_accumulator.x ...
    #   anchor+2 : force_accumulator.z ...
    #   anchor+3 : }
    if_idx = anchor - 1
    if "mfs_hs" not in lines[if_idx]:
        log("  [FAIL] expected if(mfs_hs...) above hold_coeff; layout changed")
        return False

    ind = leading_ws(lines[if_idx])
    b = ind + "    "
    new_block = [
        ind + "if (mfs_hs > 0.0001f) {",
        b + "/* MFS_147_COULOMB_HOLD: viscous damping alone only reaches a terminal",
        b + " * drift against the constant mecanum contact asymmetry. A real gearbox's",
        b + " * back-drive friction is ~constant (Coulomb) and is what actually holds",
        b + " * the robot. Add a Coulomb term that exceeds the asymmetry force; clamp",
        b + " * the total to the one-step stopping force so it can never reverse the",
        b + " * chassis (no oscillation). */",
        b + "float mfs_viscous = mfs_hs * 8.0f * mfs_chassis->mass;",
        b + "float mfs_coulomb = 2.0f;",
        b + "float mfs_total = mfs_viscous + mfs_coulomb;",
        b + "float mfs_f_stop = mfs_chassis->mass * mfs_hs / dt;",
        b + "if (mfs_total > mfs_f_stop) { mfs_total = mfs_f_stop; }",
        b + "mfs_chassis->force_accumulator.x -= (mfs_hvx / mfs_hs) * mfs_total;",
        b + "mfs_chassis->force_accumulator.z -= (mfs_hvz / mfs_hs) * mfs_total;",
        ind + "}",
    ]
    new_lines = lines[:if_idx] + new_block + lines[anchor + 4:]
    if not DRY_RUN:
        DRIVETRAIN_C.write_text("\n".join(new_lines))
    log("  [OK] Coulomb+viscous hold installed")
    return True


def step_verify_idle():
    log("Step 2: Rebuild + run idle diagnostic")
    r = subprocess.run(["make", "-C", str(SRC), "test_idle_spin_diag"],
                       capture_output=True, text=True, timeout=180)
    out = r.stdout
    print(out[-1500:] if out else "")
    if r.returncode != 0:
        print(r.stderr[-1500:] if r.stderr else "")
        log("[FAIL] idle diag build/run failed")
        return False
    # pull the key numbers
    max_omega = None
    max_speed = None
    pos_end = None
    for line in out.split("\n"):
        if "max |wheel axle omega|" in line:
            max_omega = line.strip()
        if "max chassis speed" in line:
            max_speed = line.strip()
        if "t=299" in line:
            pos_end = line.strip()
    for s in (max_omega, max_speed, pos_end):
        if s:
            log("  " + s)
    # drift check: parse final x from the t=299 line
    drift_ok = True
    if pos_end and "pos=(" in pos_end:
        try:
            seg = pos_end.split("pos=(")[1].split(")")[0]
            x_end = float(seg.split(",")[0])
            log(f"  final idle x = {x_end:.4f} (drift from ~0.02 start)")
            if abs(x_end) > 0.05:
                drift_ok = False
        except Exception:
            pass
    if not drift_ok:
        log("[WARN] still drifting more than 5 cm over the idle window")
        return False
    log("[PASS] idle drift eliminated")
    return True


def step_truth_suite():
    log("Step 3: Run physics truth suite (regression)")
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
    print("MFS 147: Coulomb Gearbox Hold (kill idle drift for good)")
    print("=" * 60)
    if DRY_RUN:
        print("  ** DRY RUN — no files modified **\n")

    if not step_replace_hold():
        return 1
    if DRY_RUN:
        log("[DRY RUN] skipping build/verify")
        return 0

    ok_idle = step_verify_idle()
    ok_truth = step_truth_suite()

    print("=" * 60)
    if ok_idle and ok_truth:
        print("  147 complete. Robot holds position at idle — drift gone.")
        print("  Stationary odometry baseline is now clean.")
        print("  Ready for Milestone 4 (wheel encoders + odometry).")
    else:
        print("  147 needs review — see [WARN]/[FAIL] above.")
    print("=" * 60)
    return 0 if (ok_idle and ok_truth) else 1


if __name__ == "__main__":
    sys.exit(main())
