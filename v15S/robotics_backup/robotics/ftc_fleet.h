/* FTC robot fleet: per-world container that makes robots steppable as a
 * kernel tick module (see ftc_module.c).
 *
 * Manual (static-link) flow keeps working untouched: create an ftc_robot,
 * call drivetrain_update() yourself, step the world. The fleet exists so
 * a HOT-LOADED plugin can own robots instead: attach "ftc-fleet" to a
 * world, spawn through this API, then step the world only — the module's
 * pre_step runs drivetrain_update() for every fleet member before the
 * engine integrates velocity (same tick position as the manual flow, so
 * behavior is identical).
 *
 * The fleet holds plain ftc_robot structs (no pointers: memcpy-safe).
 * World bodies/joints created at spawn belong to the world, NOT the
 * fleet: detaching frees the fleet array only; bodies persist until the
 * world is cleared (same ownership rule as manual creation).
 *
 * LIFETIME RULE (matches bodies after physics_world_cleanup): robot
 * pointers from ftc_fleet_get() dangle after detach. Re-fetch after
 * re-attaching; body INDICES stay valid (detach never touches bodies).
 */
#ifndef ftc_fleet_h
#define ftc_fleet_h

#include "robot.h"

struct physics_world;

/* Spawn a robot into the fleet attached to `world`. Returns the fleet
 * index (>= 0), or -1 if no ftc-fleet module is attached, the fleet is
 * full, or creation fails. Robots are simulated in spawn order. */
int ftc_fleet_spawn(struct physics_world *world, float x, float y, float z,
                    motor_preset_id preset, ftc_drivetrain_type drivetrain_type);

/* Live robot count in the world's fleet (0 if none attached). */
int ftc_fleet_count(struct physics_world *world);

/* Mutable robot access for commanding (drivetrain_tank/mecanum) and
 * inspection (odometry, battery). NULL on bad world/index. */
ftc_robot *ftc_fleet_get(struct physics_world *world, int index);

/* ---- module plumbing (used by ftc_module.c; not application API) ---- */
void *ftc_fleet_create(void);
void ftc_fleet_destroy(void *fleet_state);
void ftc_fleet_step_all(struct physics_world *world, void *fleet_state, float dt);

#endif /* ftc_fleet_h */
