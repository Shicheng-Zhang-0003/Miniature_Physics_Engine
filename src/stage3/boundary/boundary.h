#ifndef boundary_h
#define boundary_h
#include "../../stage1/master_header.h"
//Add a floor to the container system
//V and POS additions if a object is detected below boundary
void boundary_apply_floor (rigidbody *rigid_body, float floor_y_coordinate);
//Axis Aligned Box
void boundary_apply_box (rigidbody *rigid_body, vector3 minimum_bounds, vector3 maximum_bounds);
#endif
