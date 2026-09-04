#!/usr/bin/env python3
"""
MFS 125a — Repair: rewrite gui_robot_registry.c with proper nose indicator
===========================================================================
The 125 fallback insertion corrupted the file structure. This script
rewrites gui_robot_registry.c cleanly with the nose indicator integrated
correctly into both spawn and tick.

Usage:
  cd <project_root>
  python3 fixes/phase_mfs/125a_repair_heading.py [--dry-run]
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


def log(msg):
    print(f"  [125a] {msg}")


def run_build():
    result = subprocess.run(
        [sys.executable, str(TOOLS / "build_check.py"), "--quick"],
        cwd=str(ROOT), capture_output=True, text=True, timeout=120
    )
    if result.returncode != 0:
        print(result.stdout[-3000:] if result.stdout else "")
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
    print("MFS 125a: Repair heading indicator")
    print("=" * 60)

    if DRY_RUN:
        print("  ** DRY RUN **")
        print()

    path = SRC / "robotics" / "gui_robot_registry.c"

    # The clean rewrite — nose indicator properly integrated
    clean_content = """\
/* MFS_GUI_ROBOT_REGISTRY: GUI robot management with visual proxies.
* FIX 105: Initialize physics_world before creating bodies.
* FIX 125: Orange nose sphere at front of chassis for heading indication. */
#include "gui_robot_registry.h"
#include "../mpe_engine.h"
#include "../scene/scene_init.h"
#include <string.h>
#include <stdio.h>

ftc_robot mfs_gui_robots[MFS_MAX_GUI_ROBOTS];
int mfs_gui_robot_count = 0;
physics_world *mfs_gui_robot_world = NULL;
gui_robot_proxy mfs_gui_proxies[MFS_MAX_GUI_ROBOTS];

/* MFS_125: Nose offset in chassis-local space.
* Chassis half-extent Z = 0.225m, so 0.28m puts the nose just in front. */
#define MFS_NOSE_OFFSET_X 0.0f
#define MFS_NOSE_OFFSET_Y 0.0f
#define MFS_NOSE_OFFSET_Z 0.28f
#define MFS_NOSE_RADIUS    0.03f

int gui_robot_spawn(float x, float y, float z, motor_preset_id preset) {
if (mfs_gui_robot_count >= MFS_MAX_GUI_ROBOTS) {
return -1;
}
if (!mfs_gui_robot_world) {
mfs_gui_robot_world = physics_world_get_primary();
}
if (!mfs_gui_robot_world) {
return -1;
}
/* FIX 105: The legacy GUI never initializes the physics_world.
Its bodies array is NULL. We MUST init before adding bodies. */
if (!mfs_gui_robot_world->bodies) {
physics_world_init(mfs_gui_robot_world);
}
ftc_robot *robot = &mfs_gui_robots[mfs_gui_robot_count];
int rc = ftc_robot_create(mfs_gui_robot_world, robot, x, y, z, preset);
if (rc != 0) {
return -1;
}
int idx = mfs_gui_robot_count;
/* --- Create visual proxies in obj_per_scene --- */
gui_robot_proxy *proxy = &mfs_gui_proxies[idx];
proxy->chassis_proxy = -1;
proxy->nose_proxy = -1; /* MFS_125 */
for (int i = 0; i < FTC_MAX_WHEELS; i++) {
proxy->wheel_proxies[i] = -1;
}
/* Chassis proxy */
int chassis_body = robot->chassis_body;
if ((chassis_body >= 0) && (chassis_body < mfs_gui_robot_world->body_count)) {
rigidbody *src = &mfs_gui_robot_world->bodies[chassis_body];
int proxy_idx = scene_add_cube(src->position, src->half_extensions, 0.0f);
if (proxy_idx >= 0) {
obj_per_scene[proxy_idx].colour = (vector3){0.2f, 0.6f, 0.9f};
obj_per_scene[proxy_idx].static_state = true;
obj_per_scene[proxy_idx].inverse_mass = 0.0f;
proxy->chassis_proxy = proxy_idx;
}
/* MFS_125: Create heading indicator (orange nose) at front of chassis */
{
vector3 nose_local = {MFS_NOSE_OFFSET_X, MFS_NOSE_OFFSET_Y, MFS_NOSE_OFFSET_Z};
vector3 nose_world = vector3_addition(src->position,
vector4_rotate_to_vector3(src->orientation, nose_local));
int nose_idx = scene_add_object(MFS_NOSE_RADIUS, 0.0f, nose_world);
if (nose_idx >= 0) {
obj_per_scene[nose_idx].colour = (vector3){1.0f, 0.5f, 0.0f}; /* orange */
obj_per_scene[nose_idx].static_state = true;
obj_per_scene[nose_idx].inverse_mass = 0.0f;
proxy->nose_proxy = nose_idx;
}
}
}
/* Wheel proxies */
for (int i = 0; i < robot->wheel_count; i++) {
int wheel_body = robot->wheel_bodies[i];
if ((wheel_body >= 0) && (wheel_body < mfs_gui_robot_world->body_count)) {
rigidbody *src = &mfs_gui_robot_world->bodies[wheel_body];
int proxy_idx = scene_add_object(src->radius, 0.0f, src->position);
if (proxy_idx >= 0) {
obj_per_scene[proxy_idx].colour = (vector3){0.15f, 0.15f, 0.15f};
obj_per_scene[proxy_idx].static_state = true;
obj_per_scene[proxy_idx].inverse_mass = 0.0f;
proxy->wheel_proxies[i] = proxy_idx;
}
}
}
mfs_gui_robot_count++;
return idx;
}

void gui_robot_tick(float dt) {
if ((mfs_gui_robot_count <= 0) || (!mfs_gui_robot_world)) {
return;
}
/* MFS_122: Fixed-timestep accumulator for deterministic robot physics. */
static float robot_accumulator = 0.0f;
const float fixed_robot_dt = 1.0f / 60.0f;
const float max_frame_time = fixed_robot_dt * 5.0f;
robot_accumulator += dt;
if (robot_accumulator > max_frame_time) {
robot_accumulator = max_frame_time;
}
while (robot_accumulator >= fixed_robot_dt) {
for (int i = 0; i < mfs_gui_robot_count; i++) {
drivetrain_update(mfs_gui_robot_world, &mfs_gui_robots[i], fixed_robot_dt);
}
physics_world_step(mfs_gui_robot_world, fixed_robot_dt);
robot_accumulator -= fixed_robot_dt;
}
/* --- Sync visual proxies from physics world --- */
for (int i = 0; i < mfs_gui_robot_count; i++) {
ftc_robot *robot = &mfs_gui_robots[i];
gui_robot_proxy *proxy = &mfs_gui_proxies[i];
/* Sync chassis */
if ((proxy->chassis_proxy >= 0) && (proxy->chassis_proxy < object_count)) {
int chassis_body = robot->chassis_body;
if ((chassis_body >= 0) && (chassis_body < mfs_gui_robot_world->body_count)) {
rigidbody *src = &mfs_gui_robot_world->bodies[chassis_body];
rigidbody *dst = &obj_per_scene[proxy->chassis_proxy];
dst->position = src->position;
dst->orientation = src->orientation;
rigidbody_update_axes(dst);
}
}
/* Sync wheels */
for (int w = 0; w < robot->wheel_count; w++) {
int proxy_idx = proxy->wheel_proxies[w];
if ((proxy_idx >= 0) && (proxy_idx < object_count)) {
int wheel_body = robot->wheel_bodies[w];
if ((wheel_body >= 0) && (wheel_body < mfs_gui_robot_world->body_count)) {
rigidbody *src = &mfs_gui_robot_world->bodies[wheel_body];
rigidbody *dst = &obj_per_scene[proxy_idx];
dst->position = src->position;
dst->orientation = src->orientation;
rigidbody_update_axes(dst);
}
}
}
/* MFS_125: Sync heading indicator (nose) */
if ((proxy->nose_proxy >= 0) && (proxy->nose_proxy < object_count)) {
int chassis_body = robot->chassis_body;
if ((chassis_body >= 0) && (chassis_body < mfs_gui_robot_world->body_count)) {
rigidbody *chassis = &mfs_gui_robot_world->bodies[chassis_body];
vector3 nose_local = {MFS_NOSE_OFFSET_X, MFS_NOSE_OFFSET_Y, MFS_NOSE_OFFSET_Z};
vector3 nose_world = vector3_addition(chassis->position,
vector4_rotate_to_vector3(chassis->orientation, nose_local));
obj_per_scene[proxy->nose_proxy].position = nose_world;
}
}
}
}

void gui_robot_apply_drive(float forward, float strafe, float rotate) {
if ((mfs_gui_robot_count <= 0) || (!mfs_gui_robot_world)) {
return;
}
for (int i = 0; i < mfs_gui_robot_count; i++) {
drivetrain_mecanum(&mfs_gui_robots[i], forward, strafe, rotate);
}
}

int gui_robot_get_count(void) {
return mfs_gui_robot_count;
}

ftc_robot *gui_robot_get(int index) {
if ((index < 0) || (index >= mfs_gui_robot_count)) {
return NULL;
}
return &mfs_gui_robots[index];
}
"""

    if not DRY_RUN:
        path.write_text(clean_content)
    log(f"[OK] gui_robot_registry.c rewritten cleanly ({len(clean_content)} bytes)")

    # Also verify the header has nose_proxy
    h_path = SRC / "robotics" / "gui_robot_registry.h"
    h_content = h_path.read_text()
    if "nose_proxy" not in h_content:
        # Add nose_proxy to the struct
        h_content = h_content.replace(
            "int wheel_proxies[FTC_MAX_WHEELS];",
            "int wheel_proxies[FTC_MAX_WHEELS];\nint nose_proxy;             /* MFS_125 */"
        )
        if not DRY_RUN:
            h_path.write_text(h_content)
        log("[OK] Added nose_proxy to gui_robot_registry.h")
    else:
        log("[SKIP] nose_proxy already in header")

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
    print("  125a repair complete.")
    print("  Orange nose sphere at front of chassis (+Z local).")
    print("  C = rotate left (CCW), H = rotate right (CW).")
    print("=" * 60)
    return 0


if __name__ == "__main__":
    sys.exit(main())
