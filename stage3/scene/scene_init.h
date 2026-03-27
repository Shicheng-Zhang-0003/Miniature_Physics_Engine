#ifndef scene_init_h
#define scene_init_h
#include "../../stage1/master_header.h"
#include "../../stage2/master_header_2.h"
//Add a initial object for referencing (sphere, properties outlined below) (index of object is returned after, -1 if overflow)
int scene_add_object (float radius, float mass, vector3 position);
//Init Scene, default object set
void scene_init_default (void);
//Clear Objects
void scene_clear (void);
#endif
