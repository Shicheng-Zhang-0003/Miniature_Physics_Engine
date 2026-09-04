#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"

python3 -c "
h_path = 'v15R2/src/robotics/gui_robot_registry.h'
with open(h_path, 'r') as f: h_content = f.read()
proxy_struct = '''
typedef struct {
    int chassis_proxy;
    int wheel_proxies[8];
} gui_robot_proxies;

extern gui_robot_proxies mfs_gui_proxies[MFS_MAX_GUI_ROBOTS];
'''
if 'gui_robot_proxies' not in h_content:
    h_content = h_content.replace('#define MFS_MAX_GUI_ROBOTS 4', '#define MFS_MAX_GUI_ROBOTS 4\n' + proxy_struct)
with open(h_path, 'w') as f: f.write(h_content)

c_path = 'v15R2/src/robotics/gui_robot_registry.c'
with open(c_path, 'r') as f: c_content = f.read()
includes = '''#include <string.h>
#include \"../scene/scene_init.h\"
extern rigidbody *obj_per_scene;
extern int object_count;

gui_robot_proxies mfs_gui_proxies[MFS_MAX_GUI_ROBOTS];
'''
c_content = c_content.replace('#include <string.h>', includes)

spawn_marker = 'mfs_gui_robot_count++;'
proxy_creation = '''mfs_gui_robot_count++;
    int idx = mfs_gui_robot_count - 1;
    vector3 c_pos = mfs_gui_robot_world->bodies[robot->chassis_body].position;
    int c_proxy = scene_add_cube(c_pos, (vector3){0.225f, 0.075f, 0.225f}, 0.0f);
    if (c_proxy >= 0) {
        obj_per_scene[c_proxy].colour = (vector3){0.2f, 0.8f, 0.2f};
        obj_per_scene[c_proxy].static_state = true;
    }
    mfs_gui_proxies[idx].chassis_proxy = c_proxy;
    for (int i = 0; i < robot->wheel_count; i++) {
        vector3 w_pos = mfs_gui_robot_world->bodies[robot->wheel_bodies[i]].position;
        int w_proxy = scene_add_object(0.05f, 0.0f, w_pos);
        if (w_proxy >= 0) {
            obj_per_scene[w_proxy].colour = (vector3){0.1f, 0.1f, 0.1f};
            obj_per_scene[w_proxy].static_state = true;
        }
        mfs_gui_proxies[idx].wheel_proxies[i] = w_proxy;
    }'''
c_content = c_content.replace(spawn_marker, proxy_creation)

tick_marker = 'physics_world_step(mfs_gui_robot_world, dt);'
proxy_sync = '''physics_world_step(mfs_gui_robot_world, dt);
    for (int i = 0; i < mfs_gui_robot_count; i++) {
        ftc_robot *robot = &mfs_gui_robots[i];
        int c_proxy = mfs_gui_proxies[i].chassis_proxy;
        if (c_proxy >= 0 && c_proxy < object_count) {
            rigidbody *src = &mfs_gui_robot_world->bodies[robot->chassis_body];
            rigidbody *dst = &obj_per_scene[c_proxy];
            dst->position = src->position;
            dst->orientation = src->orientation;
            dst->velocity = vector3_zero();
            dst->angular_velocity = vector3_zero();
        }
        for (int w = 0; w < robot->wheel_count; w++) {
            int w_proxy = mfs_gui_proxies[i].wheel_proxies[w];
            if (w_proxy >= 0 && w_proxy < object_count) {
                rigidbody *src = &mfs_gui_robot_world->bodies[robot->wheel_bodies[w]];
                rigidbody *dst = &obj_per_scene[w_proxy];
                dst->position = src->position;
                dst->orientation = src->orientation;
                dst->velocity = vector3_zero();
                dst->angular_velocity = vector3_zero();
            }
        }
    }'''
c_content = c_content.replace(tick_marker, proxy_sync)

with open(c_path, 'w') as f: f.write(c_content)
"
echo "[PASS] 103: GUI robot proxy sync added for visibility"
