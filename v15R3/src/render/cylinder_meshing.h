#ifndef cylinder_mesh_h
#define cylinder_mesh_h
#include <epoxy/gl.h>
#include "../core/math3d.h"
#include "../core/math4_special.h"
#include "../core/rigidbody.h"
#include "sphere_meshing.h" /* mesh struct */

extern mesh cylinder_mesh;
/* Unit cylinder: axle along local X, radius 1, half-length 1.
 * Render scale (half_length, radius, radius) maps it to any body. */
void init_cylinder_system(mesh *mesh_object, int radial_segments);
#endif
