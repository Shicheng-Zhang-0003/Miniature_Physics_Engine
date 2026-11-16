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
/* MIG WP4 4.6p: spawn a SAVED MIG/CAD spec (spec-store order) as a
 * composite body in front of the camera — the user's ruling "as soon
 * as a new object type is saved that the object can be spawned in the
 * main engine". Validates through the gated build seam
 * (cad_design_from_spec + cad_build), translates the parts so the
 * assembly's COM lands at the gun's reach, physics_world_add_composite
 * composes it, and the gun's launch velocity rides the body. */
void spawner_launch_spec (int store_index);
/* MIG WP4 4.6p: spec-spawn refusals (the event log is terminal-only,
 * so the GUI HUD carries this count like the hull one). Lifetime
 * counter, never reset. */
extern unsigned long spawner_spec_refusal_count;
/* Dispatch for the spawn gun on current_spawn_type 0..7 (sphere, cube,
 * cylinder, prism, pyr4, pyr3, icosa, ell) — the one call both the hold
 * and repeat sites make. */
void spawner_launch_current_type (void);
/* MIG WP2.6 2.6.6a: hull-spawn refusal counter (flag-0 + window/pool
 * refusals). The event log is terminal-only, so the GUI HUD surfaces
 * this count. Lifetime counter, never reset. */
extern unsigned long spawner_hull_refusal_count;
#endif
