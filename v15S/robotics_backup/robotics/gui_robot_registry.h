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

/* Query */
int gui_robot_get_count(void);
ftc_robot *gui_robot_get(int index);

#endif /* gui_robot_registry_h */
