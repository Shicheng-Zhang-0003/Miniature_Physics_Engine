#include "physics_world.h"
#include "../config/mpe_constants.h"
#include <stdlib.h>

physics_world g_physics_world = {
    .obj_per_scene = NULL,
    .object_count = 0,
    .object_capacity = 0,
    .selected_object = -1
};

void physics_world_init (physics_world *world) {
    if (!world) {return;}
    if (!world -> obj_per_scene) {
        world -> obj_per_scene = (rigidbody *) malloc (mpe_max_bodies * sizeof (rigidbody));
        world -> object_capacity = mpe_max_bodies;
    }
    world -> object_count = 0;
    world -> selected_object = -1;
}

void physics_world_cleanup (physics_world *world) {
    if (!world) {return;}
    if (world -> obj_per_scene) {
        free (world -> obj_per_scene);
        world -> obj_per_scene = NULL;
    }
    world -> object_count = 0;
    world -> object_capacity = 0;
    world -> selected_object = -1;
}

physics_world *physics_world_get_primary (void) {
    return &g_physics_world;
}
