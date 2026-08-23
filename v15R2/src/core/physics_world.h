#ifndef physics_world_h
#define physics_world_h

#include "rigidbody.h"
#include "../ui_input/camera.h"
#include "../ui_input/input_control.h"

typedef struct {
    rigidbody *obj_per_scene;
    int object_count;
    int object_capacity;
    int selected_object;
    camera main_camera;
    input_status main_inputs;
} physics_world;

extern physics_world g_physics_world;

void physics_world_init (physics_world *world);
void physics_world_cleanup (physics_world *world);
physics_world *physics_world_get_primary (void);

#endif
