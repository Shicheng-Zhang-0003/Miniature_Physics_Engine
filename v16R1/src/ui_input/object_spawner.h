#ifndef object_spawner_h
#define object_spawner_h
#include "../core/math3d.h"
#include "../core/rigidbody.h"
#include "../core/physics_world.h" /* mpe_hull_shape_kind */
void spawner_launch_sphere (float spherical_radius, float physical_mass, float launch_speed);
void spawner_static_sphere (float spherical_radius, float physical_mass, vector3 static_position);
void spawner_launch_cube (vector3 position, vector3 half_extensions, float physical_mass);
void spawner_static_cube (vector3 position, vector3 half_extensions, float physical_mass);
void spawner_launch_cylinder (float radius, float half_length, float physical_mass, float launch_speed);
void spawner_static_cylinder (float radius, float half_length, float physical_mass, vector3 static_position);
/* MIG WP2.6 spawn-menu: launch a catalog hull kind (prism|pyr4|pyr3|
 * icosa|ell) on the shared hull dials (spawner.hull_{mass,size,height});
 * name is only for the event-log refusal message. */
void spawner_launch_hull (mpe_hull_shape_kind kind, const char *name, float launch_speed);
/* Dispatch for the spawn gun on current_spawn_type 0..7 (sphere, cube,
 * cylinder, prism, pyr4, pyr3, icosa, ell) — the one call both the hold
 * and repeat sites make. */
void spawner_launch_current_type (void);
#endif
