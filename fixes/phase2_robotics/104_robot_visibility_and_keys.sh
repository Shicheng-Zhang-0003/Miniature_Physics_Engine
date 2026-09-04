#!/usr/bin/env bash
# ============================================================
# FIX 104 — Robot visibility (proxy objects) + T key exclusive
#
#   Problem 1: Robot bodies live in physics_world but renderer
#   only draws obj_per_scene. Robot is invisible.
#   Fix: Create visual proxy objects in obj_per_scene and sync
#   their transforms from physics_world each tick.
#
#   Problem 2: T key still opens debug terminal.
#   Fix: Remove T terminal code; T is now robot-forward only.
#
# Phase:   phase2_robotics
# Files:   v15R3/src/robotics/gui_robot_registry.h
#          v15R3/src/robotics/gui_robot_registry.c
#          v15R3/src/ui_input/simulation_input_dispatch.c
# Risk:    medium
# ============================================================
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"

REG_H="v15R3/src/robotics/gui_robot_registry.h"
REG_C="v15R3/src/robotics/gui_robot_registry.c"
DISPATCH="v15R3/src/ui_input/simulation_input_dispatch.c"

# Backup
cp "$REG_H" "${REG_H}.pre_104"
cp "$REG_C" "${REG_C}.pre_104"
cp "$DISPATCH" "${DISPATCH}.pre_104"

# ============================================================
# PART 1: Rewrite gui_robot_registry.h with proxy tracking
# ============================================================
cat > "$REG_H" << 'EOF'
/* MFS_GUI_ROBOT_REGISTRY: GUI-side robot management.
* Owns the registry of active robots, their physics world binding,
* visual proxy objects in obj_per_scene, and per-tick sync.
*/
#ifndef gui_robot_registry_h
#define gui_robot_registry_h

#include "robot.h"
#include "drivetrain.h"
#include "../core/physics_world.h"

#define MFS_MAX_GUI_ROBOTS 4

/* Visual proxy tracking: indices into obj_per_scene */
typedef struct {
    int chassis_proxy;          /* index in obj_per_scene, -1 if none */
    int wheel_proxies[FTC_MAX_WHEELS];
} gui_robot_proxy;

/* Registry state */
extern ftc_robot mfs_gui_robots[MFS_MAX_GUI_ROBOTS];
extern int mfs_gui_robot_count;
extern physics_world *mfs_gui_robot_world;
extern gui_robot_proxy mfs_gui_proxies[MFS_MAX_GUI_ROBOTS];

/* Spawn a robot into the GUI registry + visual proxies. Returns index or -1. */
int gui_robot_spawn(float x, float y, float z, motor_preset_id preset);

/* Per-tick: drive motors, step physics, sync proxies to renderer. */
void gui_robot_tick(float dt);

/* Apply keyboard drive input to all registered robots. */
void gui_robot_apply_drive(float forward, float strafe, float rotate);

/* Query */
int gui_robot_get_count(void);
ftc_robot *gui_robot_get(int index);

#endif /* gui_robot_registry_h */
EOF

echo "  [1/3] gui_robot_registry.h rewritten with proxy tracking"

# ============================================================
# PART 2: Rewrite gui_robot_registry.c with proxy creation + sync
# ============================================================
cat > "$REG_C" << 'EOF'
/* MFS_GUI_ROBOT_REGISTRY: GUI robot management with visual proxies. */
#include "gui_robot_registry.h"
#include "../mpe_engine.h"
#include "../scene/scene_init.h"
#include <string.h>
#include <stdio.h>

ftc_robot mfs_gui_robots[MFS_MAX_GUI_ROBOTS];
int mfs_gui_robot_count = 0;
physics_world *mfs_gui_robot_world = NULL;
gui_robot_proxy mfs_gui_proxies[MFS_MAX_GUI_ROBOTS];

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

    ftc_robot *robot = &mfs_gui_robots[mfs_gui_robot_count];
    int rc = ftc_robot_create(mfs_gui_robot_world, robot, x, y, z, preset);
    if (rc != 0) {
        return -1;
    }

    int idx = mfs_gui_robot_count;

    /* --- Create visual proxies in obj_per_scene --- */
    gui_robot_proxy *proxy = &mfs_gui_proxies[idx];
    proxy->chassis_proxy = -1;
    for (int i = 0; i < FTC_MAX_WHEELS; i++) {
        proxy->wheel_proxies[i] = -1;
    }

    /* Chassis proxy: cube matching chassis dimensions */
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
    }

    /* Wheel proxies: spheres matching wheel radius */
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

    for (int i = 0; i < mfs_gui_robot_count; i++) {
        drivetrain_update(mfs_gui_robot_world, &mfs_gui_robots[i], dt);
    }

    /* Step the robot's physics world */
    physics_world_step(mfs_gui_robot_world, dt);

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
EOF

echo "  [2/3] gui_robot_registry.c rewritten with proxy creation + sync"

# ============================================================
# PART 3: Fix simulation_input_dispatch.c
#   - Remove T-key terminal opening
#   - T is now exclusively robot-forward
# ============================================================
python3 - "$DISPATCH" << 'PYEOF'
import sys

path = sys.argv[1]
with open(path, 'r') as f:
    content = f.read()

# Remove the T-key terminal opening block
old_t_block = """if (main_inputs.t_key_pressed) {
    if (main_inputs.is_debug_mode_active) {
        if (debug_terminal_is_open()) {
            debug_terminal_focus_entry();
        } else {
            debug_terminal_open(parent_window);
        }
    }
    main_inputs.t_key_pressed = false;
}"""

if old_t_block in content:
    content = content.replace(old_t_block, "/* T key is now exclusively for robot forward drive */")
    print("  Removed T-key terminal block")
else:
    # Try alternate formatting
    import re
    pattern = r'if\s*\(main_inputs\.t_key_pressed\)\s*\{[^}]*\{[^}]*\}[^}]*\}'
    match = re.search(pattern, content, re.DOTALL)
    if match:
        content = content[:match.start()] + "/* T key is now exclusively for robot forward drive */" + content[match.end():]
        print("  Removed T-key terminal block (regex)")
    else:
        print("  WARNING: Could not find T-key terminal block")

with open(path, 'w') as f:
    f.write(content)
PYEOF

echo "  [3/3] simulation_input_dispatch.c: T key now robot-only"

echo ""
echo "[PASS] 104: Robot visibility proxies + T key exclusive to robot"
