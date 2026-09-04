#!/usr/bin/env bash
# ============================================================
# FIX 105 — ROOT FIX: Robot spawn fails because physics_world
# is never initialized. The legacy GUI uses obj_per_scene and
# never calls physics_world_init(). When gui_robot_spawn calls
# ftc_robot_create -> physics_world_add_cube, it sees
# world->bodies == NULL and returns -1.
#
# Fix: Initialize the world before creating robot bodies.
# Also: Fix any stray braces in simulation_input_dispatch.c
# ============================================================
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"

REG_C="v15R2/src/robotics/gui_robot_registry.c"
DISPATCH="v15R2/src/ui_input/simulation_input_dispatch.c"

# Backup
cp "$REG_C" "${REG_C}.pre_105"
cp "$DISPATCH" "${DISPATCH}.pre_105"

# ============================================================
# PART 1: Rewrite gui_robot_registry.c with INIT FIX
# ============================================================
cat > "$REG_C" << 'EOF'
/* MFS_GUI_ROBOT_REGISTRY: GUI robot management with visual proxies.
 * FIX 105: Initialize physics_world before creating bodies. */
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

echo "  [1/2] gui_robot_registry.c rewritten with physics_world_init fix"

# ============================================================
# PART 2: Fix simulation_input_dispatch.c — remove stray brace
# ============================================================
python3 - "$DISPATCH" << 'PYEOF'
import sys, re

path = sys.argv[1]
with open(path, 'r') as f:
    lines = f.readlines()

# Find and remove orphaned closing braces that are not inside any block
# Strategy: look for lines that are just "}" or "        }" immediately
# after a comment line about T key
fixed_lines = []
i = 0
while i < len(lines):
    line = lines[i]
    stripped = line.strip()
    
    # Check for pattern: comment about T key followed by orphaned }
    if (stripped.startswith('/* T key is now exclusively') and 
        i + 1 < len(lines) and 
        lines[i + 1].strip() == '}'):
        # Keep the comment, skip the orphaned }
        fixed_lines.append(line)
        i += 2  # skip the }
        # Also skip any blank line after
        if i < len(lines) and lines[i].strip() == '':
            fixed_lines.append(lines[i])
            i += 1
        continue
    
    fixed_lines.append(line)
    i += 1

with open(path, 'w') as f:
    f.writelines(fixed_lines)

print("  Fixed stray braces in simulation_input_dispatch.c")
PYEOF

echo "  [2/2] simulation_input_dispatch.c cleaned"
echo ""
echo "[PASS] 105: Root fix applied — physics_world_init before body creation"
