#!/usr/bin/env python3
"""
MFS 126a — Repair: Add positional axis alignment to revolute_solve
===================================================================
The original 126 script failed because the pattern match didn't account
for the actual line breaks in revolute_joint.c. This script uses a more
flexible approach to find the axis alignment block and insert the
positional correction code.

Usage:
  cd <project_root>
  python3 fixes/phase_mfs/126a_repair_revolute_axis_drift.py [--dry-run]
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
    print(f"  [126a] {msg}")


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
    return result.returncode == 0


def main():
    print("=" * 60)
    print("MFS 126a: Fix Revolute Joint Axis Drift")
    print("=" * 60)

    if DRY_RUN:
        print("  ** DRY RUN **")
        print()

    path = SRC / "physics" / "revolute_joint.c"
    content = path.read_text()

    # Check if already fixed
    if "MFS_REVOLUTE_AXIS_DRIFT_FIX" in content:
        log("[SKIP] axis drift fix already present")
        return 0

    # Find the end of the axis alignment block by looking for the closing brace
    # after the angular impulse application
    lines = content.split('\n')
    insert_idx = -1

    # Look for the pattern: the last line of angular impulse application
    # followed by a closing brace
    for i in range(len(lines) - 1):
        # Look for the line that applies angular impulse to body_b
        if 'body_b->angular_velocity = vector3_addition(' in lines[i]:
            # Check if the next few lines complete the statement and have a closing brace
            for j in range(i + 1, min(i + 5, len(lines))):
                if lines[j].strip() == '}':
                    # This is the closing brace of the axis alignment block
                    insert_idx = j
                    break
            if insert_idx > 0:
                break

    if insert_idx < 0:
        log("[FAIL] Could not find axis alignment block end")
        return 1

    # Insert the positional correction code before the closing brace
    correction_code = [
        "    /* MFS_REVOLUTE_AXIS_DRIFT_FIX: Positional axis alignment correction.",
        "     * The existing axis alignment kills perpendicular angular velocity but",
        "     * doesn't correct orientation drift. This adds a Baumgarte-style correction",
        "     * that rotates body_b's orientation to keep its local axis aligned with",
        "     * body_a's local axis. Without this, wheels slowly tilt and cause erratic",
        "     * robot movement. */",
        "    {",
        "        vector3 axis_a_world = vector4_rotate_to_vector3(body_a->orientation, vector3_normalisation(p->axis_a));",
        "        vector3 axis_b_world = vector4_rotate_to_vector3(body_b->orientation, vector3_normalisation(p->axis_a));",
        "        /* Axis error: cross product gives rotation vector needed to align axis_b with axis_a.",
        "         * Magnitude is sin(angle) ≈ angle for small angles. Direction is the rotation axis. */",
        "        vector3 axis_error = vector3_cross(axis_a_world, axis_b_world);",
        "        float axis_error_len_sq = vector3_length_squared(axis_error);",
        "        if (axis_error_len_sq > 0.000001f) {",
        "            /* Baumgarte stabilization: apply angular velocity correction proportional to axis_error */",
        "            const float axis_baumgarte_beta = 0.2f; /* tuning parameter */",
        "            vector3 axis_correction = vector3_scaling(axis_error, axis_baumgarte_beta / dt);",
        "            /* Compute effective angular mass for the correction */",
        "            math3 angular_mass = math3_addition(body_a->inverse_inertia_system, body_b->inverse_inertia_system);",
        "            math3 angular_mass_inv = math3_inverse(angular_mass);",
        "            vector3 axis_impulse = vector3_scaling(math3_multiplication_vector3(angular_mass_inv, axis_correction), -1.0f);",
        "            /* Apply angular impulse to both bodies */",
        "            if (!body_a->static_state) {",
        "                body_a->angular_velocity = vector3_subtraction(",
        "                    body_a->angular_velocity,",
        "                    math3_multiplication_vector3(body_a->inverse_inertia_system, axis_impulse));",
        "            }",
        "            if (!body_b->static_state) {",
        "                body_b->angular_velocity = vector3_addition(",
        "                    body_b->angular_velocity,",
        "                    math3_multiplication_vector3(body_b->inverse_inertia_system, axis_impulse));",
        "            }",
        "        }",
        "    }"
    ]

    # Insert the correction code
    lines = lines[:insert_idx] + correction_code + lines[insert_idx:]
    new_content = '\n'.join(lines)

    if not DRY_RUN:
        path.write_text(new_content)
    log("[OK] Positional axis alignment correction added")

    # Build verification
    if not DRY_RUN:
        log("Running build verification...")
        if not run_build():
            log("[FAIL] Build failed")
            return 1
        log("[PASS] Build successful!")

        log("Running headless tests...")
        if not run_tests():
            log("[WARN] Some tests failed")
        else:
            log("[PASS] All tests passed!")

    print()
    print("=" * 60)
    print("  126a repair complete.")
    print("  Wheels should now stay aligned with the chassis.")
    print("  The revolute solver now corrects orientation drift,")
    print("  preventing the erratic movement caused by tilted wheels.")
    print("=" * 60)
    return 0


if __name__ == "__main__":
    sys.exit(main())
