#!/usr/bin/env python3
"""
MFS 151a: Relocate odometry integration to the real drivetrain_update
======================================================================
151 inserted the odometry block into drivetrain_mecanum (it matched a
comment reference to drivetrain_update). This removes the misplaced block
and re-inserts it at the end of the actual drivetrain_update function,
located by its signature (drivetrain_update + physics_world param).

Usage:
    cd <project_root>
    python3 fixes/phase_mfs/151a_fix_odometry_location.py [--dry-run]
"""
import re
import sys
import subprocess
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
ROOT = SCRIPT_DIR.parent.parent
SRC = ROOT / "v15R2" / "src"
TOOLS = ROOT / "tools"
sys.path.insert(0, str(TOOLS))
DRY_RUN = "--dry-run" in sys.argv

def log(msg): print(f"  [151a] {msg}")

DT_C = SRC / "robotics" / "drivetrain.c"
MARKER = "/* MFS_151_INTEGRATE: Odometry integration */"

ODOM_CODE = '''
    /* MFS_151_INTEGRATE: Odometry integration */
    {
        float v_fl = 0.0f, v_fr = 0.0f, v_bl = 0.0f, v_br = 0.0f;
        for (int mfs_i = 0; mfs_i < robot->wheel_count; mfs_i++) {
            int wi = robot->wheel_bodies[mfs_i];
            if (wi >= 0 && wi < world->body_count) {
                rigidbody *w = &world->bodies[wi];
                vector3 axle = w->cached_axes[0];
                if (vector3_length_squared(axle) < 0.0001f) {
                    axle = vector4_rotate_to_vector3(w->orientation, (vector3){1.0f, 0.0f, 0.0f});
                }
                float omega = vector3_dot(w->angular_velocity, axle);
                robot->wheel_radians[mfs_i] += omega * dt;
                float v = omega * w->radius;
                if (mfs_i == 0) v_fl = v;
                else if (mfs_i == 1) v_fr = v;
                else if (mfs_i == 2) v_bl = v;
                else if (mfs_i == 3) v_br = v;
            }
        }
        float v_x = (v_fl + v_fr + v_bl + v_br) * 0.25f;
        float v_z = (v_fl - v_fr - v_bl + v_br) * 0.25f;
        float v_theta = 0.0f;
        if (robot->chassis_body >= 0 && robot->chassis_body < world->body_count) {
            v_theta = world->bodies[robot->chassis_body].angular_velocity.y;
        }
        float cos_t = cosf(robot->odom_theta);
        float sin_t = sinf(robot->odom_theta);
        robot->odom_x += (v_x * cos_t - v_z * sin_t) * dt;
        robot->odom_z += (v_x * sin_t + v_z * cos_t) * dt;
        robot->odom_theta += v_theta * dt;
    }
'''


def find_block_end(content, open_brace_idx):
    depth = 0
    for i in range(open_brace_idx, len(content)):
        if content[i] == '{':
            depth += 1
        elif content[i] == '}':
            depth -= 1
            if depth == 0:
                return i
    return -1


def step_remove_misplaced():
    log("Step 1: Removing misplaced odometry block")
    content = DT_C.read_text()
    idx = content.find(MARKER)
    if idx == -1:
        log("  [SKIP] no existing MFS_151 block found")
        return content
    b = content.find("{", idx)
    if b == -1:
        log("  [WARN] marker found but no opening brace; removing marker line only")
        nl = content.find("\n", idx)
        return content[:idx] + content[nl+1:]
    end = find_block_end(content, b)
    if end == -1:
        log("  [FAIL] could not find block end")
        return content
    content = content[:idx] + content[end+1:]
    log("  [OK] removed misplaced block")
    return content


def step_insert_correct(content):
    log("Step 2: Inserting odometry into real drivetrain_update")
    m = re.search(r'\bdrivetrain_update\s*\(\s*physics_world', content)
    if not m:
        log("  [FAIL] drivetrain_update(physics_world...) definition not found")
        return None
    b = content.find("{", m.start())
    if b == -1:
        log("  [FAIL] no opening brace for drivetrain_update")
        return None
    end = find_block_end(content, b)
    if end == -1:
        log("  [FAIL] could not find end of drivetrain_update")
        return None
    content = content[:end] + ODOM_CODE + "\n" + content[end:]
    log("  [OK] inserted at end of drivetrain_update")
    return content


def step_build_run():
    log("Step 3: Rebuild + run odometry diagnostic")
    r = subprocess.run(["make", "-C", str(SRC), "test_odometry_diag"],
                       capture_output=True, text=True, timeout=180)
    print(r.stdout[-2500:] if r.stdout else "")
    if r.returncode != 0:
        print(r.stderr[-2000:] if r.stderr else "")
        log("[FAIL] build/run failed")
        return False
    log("[PASS] diagnostic complete")
    return True


def main():
    print("=" * 60)
    print("MFS 151a: Relocate Odometry to Real drivetrain_update")
    print("=" * 60)
    if DRY_RUN:
        print("  ** DRY RUN — no files modified **\n")

    content = DT_C.read_text()
    content = step_remove_misplaced()
    content = step_insert_correct(content)
    if content is None:
        return 1

    if not DRY_RUN:
        DT_C.write_text(content)
        log("  [OK] drivetrain.c written")

    if DRY_RUN:
        log("[DRY RUN] skipping build")
        return 0

    step_build_run()
    print("=" * 60)
    return 0


if __name__ == "__main__":
    sys.exit(main())
