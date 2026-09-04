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
