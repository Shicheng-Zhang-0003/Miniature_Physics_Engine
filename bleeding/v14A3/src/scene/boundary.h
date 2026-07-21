#ifndef boundary_h
#define boundary_h
#include "../core/math3D.h"
#include "../core/buffer.h"

void boundary_apply_floor (rigidbody *rigid_body, float floor_y_coordinate);
void boundary_apply_box (rigidbody *rigid_body, vector3 minimum_bounds, vector3 maximum_bounds);
#endif
