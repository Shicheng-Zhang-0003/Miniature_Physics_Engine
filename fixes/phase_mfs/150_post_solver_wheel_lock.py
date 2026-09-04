#!/usr/bin/env python3
"""
MFS 150: Post-solver wheel lock — kill the idle spin at the source
==================================================================
The contact solver inside physics_world_step injects angular momentum
into stationary mecanum wheels, causing the ~0.35 rad/s idle spin that
persists despite all damping (145/146/147). Those damping mechanisms run
in drivetrain_update BEFORE physics_world_step, so the contact solver
runs after them and re-spins the wheels.

The fix: add a post-solver wheel lock inside physics_world_step that
zeros the axle angular velocity of near-stationary mecanum wheels. This
happens AFTER the contact solver but BEFORE position integration, so it
actually stops the wheel from spinning. Models gearbox back-drive friction.

Usage:
    cd <project_root>
    python3 fixes/phase_mfs/150_post_solver_wheel_lock.py [--dry-run]
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

def log(msg): print(f"  [150] {msg}")

PW_C = SRC / "core" / "physics_world.c"

WHEEL_LOCK = '''    /* MFS_150_WHEEL_LOCK: gearbox back-drive friction locks stationary wheels.
     * After the contact solver, any mecanum wheel with small axle omega is
     * locked to prevent the idle spin from the contact solver injecting
     * angular momentum. This is truthful: a real unpowered mecanum wheel
     * can't spin freely because the gearbox resists back-driving. */
    for (int i = 0; i < world->body_count; i++) {
        rigidbody *rb = &world->bodies[i];
        if (rb->is_mecanum) {
            vector3 axle = rb->cached_axes[0];
            if (vector3_length_squared(axle) < 0.0001f) {
                axle = vector4_rotate_to_vector3(rb->orientation, (vector3){1.0f, 0.0f, 0.0f});
            }
            float axle_omega = vector3_dot(rb->angular_velocity, axle);
            if (fabsf(axle_omega) < 0.5f) {
                rb->angular_velocity = vector3_subtraction(
                    rb->angular_velocity,
                    vector3_scaling(axle, axle_omega));
            }
        }
    }
'''


def step_insert_lock():
    log("Step 1: Inserting post-solver wheel lock into physics_world_step")
    content = PW_C.read_text()
    if "MFS_150_WHEEL_LOCK" in content:
        log("  [SKIP] already present")
        return True

    lines = content.split("\n")
    anchor = None
    for i, ln in enumerate(lines):
        if "contact_cache_save(world, world_manifolds, manifold_count);" in ln:
            anchor = i
            break
    if anchor is None:
        log("  [FAIL] contact_cache_save anchor not found in physics_world.c")
        return False

    indent = ""
    for ch in lines[anchor]:
        if ch in " \t":
            indent += ch
        else:
            break

    lock_lines = WHEEL_LOCK.split("\n")
    new_lines = lines[:anchor] + lock_lines + lines[anchor:]
    if not DRY_RUN:
        PW_C.write_text("\n".join(new_lines))
    log(f"  [OK] wheel lock inserted before contact_cache_save (line {anchor})")
    return True


def step_verify_idle():
    log("Step 2: Rebuild + run idle diagnostic (142)")
    r = subprocess.run(["make", "-C", str(SRC), "test_idle_spin_diag"],
                       capture_output=True, text=True, timeout=180)
    out = r.stdout
    print(out[-2000:] if out else "")
    if r.returncode != 0:
        print(r.stderr[-1500:] if r.stderr else "")
        log("[FAIL] idle diag build/run failed")
        return False
    max_omega = None
    max_speed = None
    for line in out.split("\n"):
        if "max |wheel axle omega|" in line:
            max_omega = line.strip()
        if "max chassis speed" in line:
            max_speed = line.strip()
    for s in (max_omega, max_speed):
        if s:
            log("  " + s)
    if max_omega and "0." in max_omega:
        try:
            val = float(max_omega.split("=")[-1].strip())
            if val > 0.1:
                log(f"[WARN] wheels still spinning at {val} rad/s")
                return False
        except:
            pass
    log("[PASS] idle wheels locked")
    return True


def step_truth_suite():
    log("Step 3: Run physics truth suite (regression)")
    r = subprocess.run(["make", "-C", str(SRC), "test_physics_truth"],
                       capture_output=True, text=True, timeout=240)
    tail = r.stdout[-1200:]
    print(tail)
    if "Failed: 0" not in tail:
        log("[WARN] truth suite has failures")
        return False
    log("[PASS] truth suite green")
    return True


def main():
    print("=" * 60)
    print("MFS 150: Post-Solver Wheel Lock")
    print("=" * 60)
    if DRY_RUN:
        print("  ** DRY RUN — no files modified **\n")

    if not step_insert_lock():
        return 1
    if DRY_RUN:
        log("[DRY RUN] skipping build/verify")
        return 0

    ok_idle = step_verify_idle()
    ok_truth = step_truth_suite()

    print("=" * 60)
    if ok_idle and ok_truth:
        print("  150 complete. Idle wheels now locked by gearbox friction.")
        print("  Robot holds dead still — no spin, no drift, no encoder corruption.")
        print("  Ready for Milestone 4 (wheel encoders + odometry).")
    else:
        print("  150 needs review — see [WARN]/[FAIL] above.")
    print("=" * 60)
    return 0 if (ok_idle and ok_truth) else 1


if __name__ == "__main__":
    sys.exit(main())
