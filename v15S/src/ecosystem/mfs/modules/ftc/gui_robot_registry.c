/* MFS_GUI_ROBOT_REGISTRY: GUI robot management with visual proxies.
* FIX 105: Initialize physics_world before creating bodies.
* FIX 125: Orange nose sphere at front of chassis for heading indication. */
#include "gui_robot_registry.h"
#include "mpe_engine.h"
#include "scene/scene_init.h"
#include <string.h>
#include <stdio.h>

ftc_robot mfs_gui_robots[MFS_MAX_GUI_ROBOTS];
int mfs_gui_robot_count = 0;
physics_world *mfs_gui_robot_world = NULL;
gui_robot_proxy mfs_gui_proxies[MFS_MAX_GUI_ROBOTS];

/* MFS_125: Nose offset in chassis-local space.
 * FIX-AUDIT: front wheels are at -Z (robot.c), so the nose must be -Z too.
 * Was +0.28 (rear), contradicting wheel labels and drive direction. */
#define MFS_NOSE_OFFSET_X 0.0f
#define MFS_NOSE_OFFSET_Y 0.0f
#define MFS_NOSE_OFFSET_Z -0.28f
#define MFS_NOSE_RADIUS    0.03f

int gui_robot_spawn(float x, float y, float z, motor_preset_id preset) {
if (mfs_gui_robot_count >= MFS_MAX_GUI_ROBOTS) {
return -1;
}
/* DESPOT-FIX: old code bound the primary world once and never re-checked —
 * if the host swapped primary worlds, robots spawned into the stale world
 * while rendering read the new one (silent split-brain). Pin to the first
 * bound world and refuse cross-world spawns loudly instead of mixing. */
physics_world *primary = physics_world_get_primary();
if (!mfs_gui_robot_world) {
mfs_gui_robot_world = primary;
} else if (primary && primary != mfs_gui_robot_world) {
fprintf(stderr, "gui_robot_registry: already bound to a different primary world; refusing spawn (clear first)\n");
return -1;
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
/* MFS_PORT_V15S: visual proxies are static bodies in the same world
 * (the retired obj_per_scene/object_count globals are gone). The engine
 * renders world bodies directly; proxies preserve the legacy
 * duplicate-body render path for GUI code that reads them. */
int proxy_idx = physics_world_add_cube(mfs_gui_robot_world, src->position, src->half_extensions, 0.0f);
if (proxy_idx >= 0) {
mfs_gui_robot_world->bodies[proxy_idx].colour = (vector3){0.2f, 0.6f, 0.9f};
mfs_gui_robot_world->bodies[proxy_idx].static_state = true;
mfs_gui_robot_world->bodies[proxy_idx].inverse_mass = 0.0f;
/* Proxies overlap the real bodies by design: exclude from ALL contact
 * (broadphase/dispatch/floor/CCD) or the solver fights the robot. */
mfs_gui_robot_world->bodies[proxy_idx].no_collide = true;
proxy->chassis_proxy = proxy_idx;
}
/* MFS_125: Create heading indicator (orange nose) at front of chassis */
{
vector3 nose_local = {MFS_NOSE_OFFSET_X, MFS_NOSE_OFFSET_Y, MFS_NOSE_OFFSET_Z};
vector3 nose_world = vector3_addition(src->position,
vector4_rotate_to_vector3(src->orientation, nose_local));
int nose_idx = physics_world_add_sphere(mfs_gui_robot_world, MFS_NOSE_RADIUS, 0.0f, nose_world);
if (nose_idx >= 0) {
mfs_gui_robot_world->bodies[nose_idx].colour = (vector3){1.0f, 0.5f, 0.0f}; /* orange */
mfs_gui_robot_world->bodies[nose_idx].static_state = true;
mfs_gui_robot_world->bodies[nose_idx].inverse_mass = 0.0f;
mfs_gui_robot_world->bodies[nose_idx].no_collide = true;
proxy->nose_proxy = nose_idx;
}
}
}
/* Wheel proxies */
for (int i = 0; i < robot->wheel_count; i++) {
int wheel_body = robot->wheel_bodies[i];
if ((wheel_body >= 0) && (wheel_body < mfs_gui_robot_world->body_count)) {
rigidbody *src = &mfs_gui_robot_world->bodies[wheel_body];
int proxy_idx = physics_world_add_sphere(mfs_gui_robot_world, src->radius, 0.0f, src->position);
if (proxy_idx >= 0) {
mfs_gui_robot_world->bodies[proxy_idx].colour = (vector3){0.15f, 0.15f, 0.15f};
mfs_gui_robot_world->bodies[proxy_idx].static_state = true;
mfs_gui_robot_world->bodies[proxy_idx].inverse_mass = 0.0f;
mfs_gui_robot_world->bodies[proxy_idx].no_collide = true;
proxy->wheel_proxies[i] = proxy_idx;
}
}
}
mfs_gui_robot_count++;
return idx;
}

/* gui_robot_tick OWNS stepping mfs_gui_robot_world (fixed 60 Hz
 * accumulator). Do NOT step that world from the engine loop as well:
 * double-stepping integrates forces twice per tick. Use this tick OR
 * the engine loop, never both, for any bound world. */
/* FIX-AUDIT-DESPOT: set by gui_robot_clear so the next tick drops the
 * stale time debt instead of burst-stepping freshly spawned robots. */
static int s_tick_accumulator_reset = 0;
void gui_robot_tick(float dt) {
if ((mfs_gui_robot_count <= 0) || (!mfs_gui_robot_world)) {
return;
}
/* MFS_122: Fixed-timestep accumulator for deterministic robot physics. */
static float robot_accumulator = 0.0f;
if (s_tick_accumulator_reset) {
s_tick_accumulator_reset = 0;
robot_accumulator = 0.0f;
}
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
if ((proxy->chassis_proxy >= 0) && (proxy->chassis_proxy < mfs_gui_robot_world->body_count)) {
int chassis_body = robot->chassis_body;
if ((chassis_body >= 0) && (chassis_body < mfs_gui_robot_world->body_count)) {
rigidbody *src = &mfs_gui_robot_world->bodies[chassis_body];
rigidbody *dst = &mfs_gui_robot_world->bodies[proxy->chassis_proxy];
dst->position = src->position;
dst->orientation = src->orientation;
rigidbody_update_axes(dst);
}
}
/* Sync wheels */
for (int w = 0; w < robot->wheel_count; w++) {
int proxy_idx = proxy->wheel_proxies[w];
if ((proxy_idx >= 0) && (proxy_idx < mfs_gui_robot_world->body_count)) {
int wheel_body = robot->wheel_bodies[w];
if ((wheel_body >= 0) && (wheel_body < mfs_gui_robot_world->body_count)) {
rigidbody *src = &mfs_gui_robot_world->bodies[wheel_body];
rigidbody *dst = &mfs_gui_robot_world->bodies[proxy_idx];
dst->position = src->position;
dst->orientation = src->orientation;
rigidbody_update_axes(dst);
}
}
}
/* MFS_125: Sync heading indicator (nose) */
if ((proxy->nose_proxy >= 0) && (proxy->nose_proxy < mfs_gui_robot_world->body_count)) {
int chassis_body = robot->chassis_body;
if ((chassis_body >= 0) && (chassis_body < mfs_gui_robot_world->body_count)) {
rigidbody *chassis = &mfs_gui_robot_world->bodies[chassis_body];
vector3 nose_local = {MFS_NOSE_OFFSET_X, MFS_NOSE_OFFSET_Y, MFS_NOSE_OFFSET_Z};
vector3 nose_world = vector3_addition(chassis->position,
vector4_rotate_to_vector3(chassis->orientation, nose_local));
mfs_gui_robot_world->bodies[proxy->nose_proxy].position = nose_world;
}
}
}
}

void gui_robot_apply_drive(float forward, float strafe, float rotate) {
if ((mfs_gui_robot_count <= 0) || (!mfs_gui_robot_world)) {
return;
}
for (int i = 0; i < mfs_gui_robot_count; i++) {
/* MFS_164_DRIVE_DISPATCH: dispatch based on drivetrain type */
if (mfs_gui_robots[i].drivetrain_type == FTC_DRIVETRAIN_TANK) {
drivetrain_tank(&mfs_gui_robots[i], forward - rotate, forward + rotate);
} else {
drivetrain_mecanum(&mfs_gui_robots[i], forward, strafe, rotate);
}
}
}

/* Release a registry slot. Bodies persist until scene_clear (no
 * mid-array removal exists); proxies are already non-colliding so the
 * leftovers are render-only. */
void gui_robot_despawn(int index) {
if ((index < 0) || (index >= mfs_gui_robot_count)) {
return;
}
for (int i = index; i + 1 < mfs_gui_robot_count; i++) {
mfs_gui_robots[i] = mfs_gui_robots[i + 1];
mfs_gui_proxies[i] = mfs_gui_proxies[i + 1];
}
mfs_gui_robot_count--;
}

void gui_robot_clear(void) {
mfs_gui_robot_count = 0;
/* FIX-AUDIT-DESPOT: the fixed-step accumulator in gui_robot_tick is static
 * and survived clear, so the first tick after a clear+respawn consumed a
 * stale time debt (up to 5/60 s) and burst-stepped the new robots. Publish
 * a reset that the next tick consumes. */
s_tick_accumulator_reset = 1;
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
