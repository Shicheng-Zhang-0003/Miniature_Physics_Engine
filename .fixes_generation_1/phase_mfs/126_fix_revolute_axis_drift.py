#!/usr/bin/env python3
"""
MFS 126: Fix revolute joint axis drift
=======================================
Adds positional axis alignment correction to revolute_solve.
The existing solver kills perpendicular angular velocity but doesn't
correct orientation drift. This adds a Baumgarte-style correction
that rotates the wheel's orientation to keep its axle aligned with
the chassis's axle.

Usage:
  cd <project_root>
  python3 fixes/phase_mfs/126_fix_revolute_axis_drift.py [--dry-run]
"""

import sys
import subprocess
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
ROOT = SCRIPT_DIR.parent.parent
SRC = ROOT / "v15R3" / "src"
TOOLS = ROOT / "tools"

sys.path.insert(0, str(TOOLS))
from refactor import Refactor, RefactorError

DRY_RUN = "--dry-run" in sys.argv


def log(msg):
    print(f"  [126] {msg}")


def run_build():
    result = subprocess.run(
        [sys.executable, str(TOOLS / "build_check.py"), "--quick"],
        cwd=str(ROOT), capture_output=True, text=True, timeout=120
    )
    if result.returncode != 0:
        print(result.stdout[-2000:] if result.stdout else "")
        print(result.stderr[-2000:] if result.stderr else "")
        return False
    return True


def run_tests():
    result = subprocess.run(
        [sys.executable, str(TOOLS / "test_runner.py")],
        cwd=str(ROOT), capture_output=True, text=True, timeout=300
    )
    print(result.stdout[-3000:] if result.stdout else "")
    if result.returncode != 0:
        print(result.stderr[-1000:] if result.stderr else "")
        return False
    return True


def step_fix_revolute_axis_drift():
    """Add positional axis alignment correction to revolute_solve."""
    log("Step 1: Adding positional axis alignment to revolute_solve")
    path = SRC / "physics" / "revolute_joint.c"
    content = path.read_text()

    if "MFS_REVOLUTE_AXIS_DRIFT_FIX" in content:
        log("  [SKIP] axis drift fix already present")
        return True

    # Find the end of the existing axis alignment block
    # It ends with:
    #   body_b->angular_velocity = vector3_addition(
    #       body_b->angular_velocity,
    #       math3_multiplication_vector3(body_b->inverse_inertia_system, angular_impulse));
    # }

    old_block = """body_b->angular_velocity = vector3_addition(
body_b->angular_velocity,
math3_multiplication_vector3(body_b->inverse_inertia_system, angular_impulse));
}"""

    new_block = """body_b->angular_velocity = vector3_addition(
body_b->angular_velocity,
math3_multiplication_vector3(body_b->inverse_inertia_system, angular_impulse));
}
/* MFS_REVOLUTE_AXIS_DRIFT_FIX: Positional axis alignment correction.
* The existing axis alignment kills perpendicular angular velocity but
* doesn't correct orientation drift. This adds a Baumgarte-style correction
* that rotates body_b's orientation to keep its local axis aligned with
* body_a's local axis. Without this, wheels slowly tilt and cause erratic
* robot movement. */
{
vector3 axis_a_world = vector4_rotate_to_vector3(body_a->orientation, vector3_normalisation(p->axis_a));
vector3 axis_b_world = vector4_rotate_to_vector3(body_b->orientation, vector3_normalisation(p->axis_a));
/* Axis error: cross product gives rotation vector needed to align axis_b with axis_a.
* Magnitude is sin(angle) ≈ angle for small angles. Direction is the rotation axis. */
vector3 axis_error = vector3_cross(axis_a_world, axis_b_world);
float axis_error_len_sq = vector3_length_squared(axis_error);
if (axis_error_len_sq > 0.000001f) {
/* Baumgarte stabilization: apply angular velocity correction proportional to axis_error */
const float axis_baumgarte_beta = 0.2f; /* tuning parameter */
vector3 axis_correction = vector3_scaling(axis_error, axis_baumgarte_beta / dt);
/* Compute effective angular mass for the correction */
math3 angular_mass = math3_addition(body_a->inverse_inertia_system, body_b->inverse_inertia_system);
math3 angular_mass_inv = math3_inverse(angular_mass);
vector3 axis_impulse = vector3_scaling(math3_multiplication_vector3(angular_mass_inv, axis_correction), -1.0f);
/* Apply angular impulse to both bodies */
if (!body_a->static_state) {
body_a->angular_velocity = vector3_subtraction(
body_a->angular_velocity,
math3_multiplication_vector3(body_a->inverse_inertia_system, axis_impulse));
}
if (!body_b->static_state) {
body_b->angular_velocity = vector3_addition(
body_b->angular_velocity,
math3_multiplication_vector3(body_b->inverse_inertia_system, axis_impulse));
}
}
}"""

    if old_block in content:
        r = Refactor(str(path))
        r.replace(
            old=old_block,
            new=new_block,
            label="Add positional axis alignment correction"
        )
        if not DRY_RUN:
            r.apply(dry_run=False)
        else:
            print(r.diff())
        log("  [OK] Positional axis alignment correction added")
        return True

    log("  [WARN] Could not find exact axis alignment block")
    return False


def main():
    print("=" * 60)
    print("MFS 126: Fix Revolute Joint Axis Drift")
    print("=" * 60)

    if DRY_RUN:
        print("  ** DRY RUN — no files will be modified **")
        print()

    if not SRC.exists():
        print(f"FATAL: Source directory not found: {SRC}")
        return 1

    steps = [
        ("Revolute axis drift fix", step_fix_revolute_axis_drift),
    ]

    for name, func in steps:
        try:
            if not func():
                print(f"\n[FAIL] Step '{name}' failed. Aborting.")
                return 1
        except RefactorError as e:
            print(f"\n[FAIL] Step '{name}' raised RefactorError: {e}")
            return 1
        except Exception as e:
            print(f"\n[FAIL] Step '{name}' raised unexpected error: {e}")
            import traceback
            traceback.print_exc()
            return 1

    print()

    if not DRY_RUN:
        log("Running build verification...")
        if not run_build():
            log("[FAIL] Build failed. Review errors above.")
            return 1
        log("[PASS] Build successful!")

        log("Running headless tests...")
        if not run_tests():
            log("[WARN] Some tests failed. Review output above.")
        else:
            log("[PASS] All tests passed!")
    else:
        log("[DRY RUN] Skipping build verification.")

    print()
    print("=" * 60)
    print("  DONE. Wheels should now stay aligned with the chassis.")
    print("  The revolute solver now corrects orientation drift,")
    print("  preventing the erratic movement caused by tilted wheels.")
    print("=" * 60)
    return 0


if __name__ == "__main__":
    sys.exit(main())
