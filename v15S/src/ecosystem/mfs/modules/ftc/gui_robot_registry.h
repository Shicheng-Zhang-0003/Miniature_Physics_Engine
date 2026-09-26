/* MFS_GUI_ROBOT_REGISTRY: GUI-side robot management.
* Owns the registry of active robots, their physics world binding,
* visual proxy objects in obj_per_scene, and per-tick sync.
*
* FIX-AUDIT-DESPOT contract (was undocumented):
* - CAPACITY: at most MFS_MAX_GUI_ROBOTS (4) robots; spawn returns -1 past
*   it. The 4-cap mirrors the practical fleet limit (see FTC_FLEET_MAX doc
*   in ftc_fleet.c): GUI robots are full roller assemblies, not ghosts.
* - WORLD BINDING: the registry binds to the PRIMARY world on first spawn
*   (physics_world_get_primary) and never rebinds; all later spawns land in
*   that world even if the primary changes. Call gui_robot_clear before
*   switching worlds.
* - DRIVE TYPE: spawn builds MECANUM robots only (ftc_robot_create); the
*   tank branch in gui_robot_apply_drive serves robots whose
*   drivetrain_type a host flips afterwards, not anything spawn produces.
*/
#ifndef gui_robot_registry_h
#define gui_robot_registry_h

#include "submodules/robot.h"
#include "submodules/drivetrain.h"
#include "core/physics_world.h"

#define MFS_MAX_GUI_ROBOTS 4

/* Visual proxy tracking: indices into obj_per_scene */
typedef struct {
    int chassis_proxy;          /* index in obj_per_scene, -1 if none */
    int wheel_proxies[FTC_MAX_WHEELS];
int nose_proxy;             /* MFS_125: heading indicator */
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

/* Release a slot / release all slots (bodies persist until scene_clear). */
void gui_robot_despawn(int index);
void gui_robot_clear(void);

/* Query */
int gui_robot_get_count(void);
ftc_robot *gui_robot_get(int index);

#endif /* gui_robot_registry_h */
