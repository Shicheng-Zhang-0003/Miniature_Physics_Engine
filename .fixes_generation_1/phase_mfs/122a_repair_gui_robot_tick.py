#!/usr/bin/env python3
"""
MFS 122a — Repair: gui_robot_tick fixed-timestep scope error
=============================================================
The 122 fallback declared fixed_robot_dt inside a block wrapping only
physics_world_step, but drivetrain_update is called BEFORE that block.
This rewrites the entire tick function so both drivetrain_update AND
physics_world_step live inside the accumulator loop, with fixed_robot_dt
declared at function scope.

Usage:
  cd <project_root>
  python3 fixes/phase_mfs/122a_repair_gui_robot_tick.py [--dry-run]
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
    print(f"  [122a] {msg}")


def main():
    print("=" * 60)
    print("MFS 122a: Repair gui_robot_tick scope error")
    print("=" * 60)

    if DRY_RUN:
        print("  ** DRY RUN **")
        print()

    path = SRC / "robotics" / "gui_robot_registry.c"
    content = path.read_text()

    # The broken fallback produced something like:
    #   drivetrain_update(..., fixed_robot_dt);   <-- fixed_robot_dt not in scope
    #   {
    #   static float robot_accumulator = ...
    #   const float fixed_robot_dt = ...
    #   while (...) { physics_world_step(...); }
    #   }
    #
    # Fix: rewrite the entire gui_robot_tick function body to put
    # everything inside the accumulator loop.

    # Detect the broken state
    if "fixed_robot_dt" not in content:
        log("[SKIP] fixed_robot_dt not found — nothing to repair")
        return 0

    # Strategy: find gui_robot_tick and rewrite it entirely
    # Find the function boundaries
    func_start_marker = "void gui_robot_tick(float dt) {"
    if func_start_marker not in content:
        func_start_marker = "void gui_robot_tick (float dt) {"
    if func_start_marker not in content:
        log("[FAIL] Could not find gui_robot_tick function")
        return 1

    func_start_idx = content.index(func_start_marker)

    # Find the end of the function by counting braces
    brace_count = 0
    func_end_idx = func_start_idx
    found_open = False
    for i in range(func_start_idx, len(content)):
        if content[i] == '{':
            brace_count += 1
            found_open = True
        elif content[i] == '}':
            brace_count -= 1
            if found_open and brace_count == 0:
                func_end_idx = i + 1
                break

    old_func = content[func_start_idx:func_end_idx]
    log(f"Found gui_robot_tick ({len(old_func)} chars)")

    # Check if it contains proxy sync code (from fix 104/105)
    has_proxy_sync = "gui_robot_proxy" in old_func or "chassis_proxy" in old_func

    if has_proxy_sync:
        new_func = """void gui_robot_tick(float dt) {
if ((mfs_gui_robot_count <= 0) || (!mfs_gui_robot_world)) {
return;
}
/* MFS_122: Fixed-timestep accumulator for deterministic robot physics.
* The robot world steps at 60 Hz regardless of render framerate. */
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
}
}"""
    else:
        new_func = """void gui_robot_tick(float dt) {
if ((mfs_gui_robot_count <= 0) || (!mfs_gui_robot_world)) {
return;
}
/* MFS_122: Fixed-timestep accumulator for deterministic robot physics.
* The robot world steps at 60 Hz regardless of render framerate. */
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
}"""

    new_content = content[:func_start_idx] + new_func + content[func_end_idx:]

    if DRY_RUN:
        log("[DRY RUN] Would rewrite gui_robot_tick")
        print(f"  Old function: {len(old_func)} chars")
        print(f"  New function: {len(new_func)} chars")
        return 0

    path.write_text(new_content)
    log("[OK] gui_robot_tick rewritten with correct scope")

    # Build verification
    log("Running build verification...")
    result = subprocess.run(
        [sys.executable, str(TOOLS / "build_check.py"), "--quick"],
        cwd=str(ROOT), capture_output=True, text=True, timeout=120
    )
    print(result.stdout[-2000:] if result.stdout else "")
    if result.returncode != 0:
        print(result.stderr[-2000:] if result.stderr else "")
        log("[FAIL] Build failed after repair")
        return 1

    log("[PASS] Build successful!")
    print()
    print("=" * 60)
    print("  122a repair complete.")
    print("  fixed_robot_dt is now at function scope, visible to")
    print("  both drivetrain_update and physics_world_step.")
    print("=" * 60)
    return 0


if __name__ == "__main__":
    sys.exit(main())
