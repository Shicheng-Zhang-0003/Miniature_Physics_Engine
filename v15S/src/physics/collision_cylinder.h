/* Cylinder narrowphase (extracted from collision_mechanics.c).
 * TRUTH status (P1 audit fixes):
 * - cyl-sphere: EXACT solid-cylinder SDF (flat caps, rim, inside).
 * - cyl-cube: EXACT segment-OBB convex ternary + face-parallel line support.
 * - cyl-cyl: coaxial face-gap truth + parallel 2-point support.
 * - cyl-floor: exact vertical half-extent + 2-point wheel support. */
#ifndef collision_cylinder_h
#define collision_cylinder_h
#include "../core/rigidbody.h"
#include "collision_mechanics.h"

/* Shared floor proxy (defined in collision_mechanics.c, thread-local). */
rigidbody *collision_static_plane_body_proxy(float plane_y);

bool collision_static_plane_cylinder(rigidbody *cyl, float plane_y, collision_data *out);
bool collision_cylinder_sphere(rigidbody *cyl, rigidbody *sph, collision_data *out);
bool collision_cylinder_cube(rigidbody *cyl, rigidbody *cube, collision_data *out);
bool collision_cylinder_cylinder(rigidbody *cyl_a, rigidbody *cyl_b, collision_data *out);
#endif
