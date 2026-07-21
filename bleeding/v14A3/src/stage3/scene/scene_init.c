#include "scene_init.h"
#include "../../stage5/constraints/spring_joint.h"
#include <stdlib.h>
#include <stdio.h>

static bool pool_is_initialized = false;

void scene_allocate_pool (void) {
    if (!pool_is_initialized) {
        obj_per_scene = (rigidbody *) malloc (MPE_MAX_BODIES * sizeof (rigidbody));
        if (!obj_per_scene) {
            fprintf (stderr, "Error POOL00: Failed to allocate physics heap.\n");
            exit (1);
        }
        object_capacity = MPE_MAX_BODIES;
        pool_is_initialized = true;
    }
}

int scene_add_object (float radius, float mass, vector3 initial_position) {
    scene_allocate_pool ();
    if (object_count >= MPE_MAX_BODIES) { fprintf (stderr, "Error POOL01: Maximum object capacity reached.\n"); return -1; }
    rigidbody_initialisation_sphere (&obj_per_scene [object_count], radius, mass, initial_position);
    int current_object_index = object_count;
    object_count += 1;
    return current_object_index;
}

int scene_add_cube (vector3 position, vector3 half_extensions, float mass) {
    scene_allocate_pool ();
    if (object_count >= MPE_MAX_BODIES) { fprintf (stderr, "Error POOL01: Maximum object capacity reached.\n"); return -1; }
    rigidbody_initialisation_cube (&obj_per_scene [object_count], position, half_extensions, mass);
    int current_object_index = object_count;
    object_count += 1;
    return current_object_index;
}

void scene_init_default (void) {
    scene_clear ();
    int object_grey_index = scene_add_object (2.0f, 0.0f, (vector3) {0.0f, 2.0f, 0.0f});
    obj_per_scene [object_grey_index].colour = (vector3) {0.8f, 0.8f, 0.8f};
}

void scene_clear (void) {
    object_count = 0;
    joint_init_pool ();
}
