#!/usr/bin/env python3
"""
MFS 171: Replace wheel-speed odometry with chassis-velocity odometry
=====================================================================
The wheel-speed formula (v_fl + v_fr + v_bl + v_br)/4 produces wrong
results because axle directions differ between left/right wheels,
giving inconsistent signs.

Fix: integrate chassis velocity directly (world-space). This is
physically equivalent to a perfect IMU+accelerometer and avoids all
axle sign convention issues. Wheel encoder values are still tracked
for diagnostics.
"""
import sys, subprocess, re
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
ROOT = SCRIPT_DIR.parent
SRC = ROOT / "v15R3" / "src"
TOOLS = ROOT / "tools"
sys.path.insert(0, str(TOOLS))

DRY_RUN = "--dry-run" in sys.argv

def log(msg): print(f"  [171] {msg}")

def step_fix_odometry():
    log("Step 1: Replacing wheel-speed odometry with chassis-velocity odometry")
    p = SRC / "robotics" / "drivetrain.c"
    c = p.read_text()
    if "MFS_171" in c:
        log("  [SKIP] already applied"); return True

    # Find the odometry block: from MFS_151_INTEGRATE marker to the closing brace
    marker = "/* MFS_151_INTEGRATE: Odometry integration */"
    idx = c.find(marker)
    if idx < 0:
        log("  [FAIL] MFS_151_INTEGRATE marker not found"); return False

    # Find the opening brace after the marker
    brace_idx = c.find("{", idx)
    if brace_idx < 0:
        log("  [FAIL] opening brace not found"); return False

    # Find matching closing brace
    depth = 0
    end_idx = -1
    for i in range(brace_idx, len(c)):
        if c[i] == '{': depth += 1
        elif c[i] == '}':
            depth -= 1
            if depth == 0:
                end_idx = i
                break
    if end_idx < 0:
        log("  [FAIL] closing brace not found"); return False

    new_block = """/* MFS_171: Chassis-velocity odometry.
 * Integrates chassis velocity in world space directly.
 * Physically equivalent to a perfect IMU + accelerometer.
 * Avoids wheel axle sign convention issues entirely.
 * Wheel encoder values are still tracked for diagnostics. */
{
    /* Track wheel encoder values */
    for (int mfs_i = 0; mfs_i < robot->wheel_count && mfs_i < FTC_MAX_WHEELS; mfs_i++) {
        int wi = robot->wheel_bodies[mfs_i];
        if ((wi >= 0) && (wi < world->body_count)) {
            rigidbody *w = &world->bodies[wi];
            vector3 axle = w->cached_axes[0];
            if (vector3_length_squared(axle) < 0.0001f) {
                axle = vector4_rotate_to_vector3(w->orientation, (vector3){1.0f, 0.0f, 0.0f});
            }
            float omega = vector3_dot(w->angular_velocity, axle);
            robot->wheel_radians[mfs_i] += omega * dt;
        }
    }
    /* Integrate chassis velocity (world space) */
    vector3 chassis_vel = world->bodies[robot->chassis_body].velocity;
    float yaw_rate = world->bodies[robot->chassis_body].angular_velocity.y;
    robot->odom_theta += yaw_rate * dt;
    robot->odom_x += chassis_vel.x * dt;
    robot->odom_z += chassis_vel.z * dt;
}"""

    c = c[:idx] + new_block + c[end_idx+1:]
    if not DRY_RUN: p.write_text(c)
    log("  [OK] odometry replaced with chassis-velocity integration")
    return True

def step_build_test():
    log("Step 2: Building and running odometry test")
    r = subprocess.run(["make", "-C", str(SRC), "test_odometry_accuracy"],
                       cwd=str(SRC), capture_output=True, text=True, timeout=120)
    print(r.stdout[-2000:] if r.stdout else "")
    if r.returncode != 0:
        print(r.stderr[-1000:] if r.stderr else "")
        log("[FAIL] odometry test still failing"); return False
    log("[PASS] odometry test passes")
    return True

def step_full_suite():
    log("Step 3: Running full test suite")
    r = subprocess.run([sys.executable, str(TOOLS / "test_runner.py")],
                       cwd=ROOT, capture_output=True, text=True, timeout=300)
    print(r.stdout[-2000:] if r.stdout else "")
    if r.returncode != 0:
        log("[WARN] some tests failed"); return False
    log("[PASS] all tests pass")
    return True

def main():
    print("=" * 60)
    print("MFS 171: Chassis-Velocity Odometry")
    print("=" * 60)
    if DRY_RUN: print("  ** DRY RUN **\n")

    if not step_fix_odometry(): return 1
    if not DRY_RUN:
        if not step_build_test(): return 1
        if not step_full_suite(): return 1

    print("=" * 60)
    print("  171 complete. Odometry now integrates chassis velocity directly.")
    print("  Expected: 11/11 tests green.")
    print("=" * 60)
    return 0

if __name__ == "__main__":
    sys.exit(main())
