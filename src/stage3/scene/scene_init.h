#ifndef scene_init_h
#define scene_init_h
#include "../../stage1/master_header.h"
#include "../../stage2/master_header_2.h"
#include "../../stage5/master_header_5.h"
extern int object_capacity;
//Add a initial object for referencing (sphere, properties outlined below) (index of object is returned after, -1 if overflow)
int scene_add_object (float radius, float mass, vector3 initial_position);
//Add a cube object
int scene_add_cube (vector3 position, vector3 half_extensions, float mass);
//Init Scene, default object set
void scene_init_default (void);
//Clear Objects
void scene_clear (void);
#endif
