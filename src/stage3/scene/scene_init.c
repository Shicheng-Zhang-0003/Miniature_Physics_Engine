#include "scene_init.h"
#include <stdlib.h>
#include <stdio.h>
//Maximum Object count
int scene_add_object (float radius, float mass, vector3 initial_position) {
    if (object_count >= object_capacity) {
        int new_object_capacity = 0;
        if (object_capacity == 0) {new_object_capacity = 16;}
        else {new_object_capacity = object_capacity * 2;}
        rigidbody *new_objects_array = realloc (obj_per_scene, new_object_capacity * sizeof (rigidbody));
        if (!new_objects_array) {fprintf (stderr, "Maximum object count reached.\n"); return -1;}
        obj_per_scene = new_objects_array;
        object_capacity = new_object_capacity;
    } rigidbody_initialisation_sphere (&obj_per_scene [object_count], radius, mass, initial_position);
    int current_object_index = object_count;
    object_count += 1;
    return current_object_index;
} int scene_add_cube (vector3 position, vector3 half_extensions, float mass) {
    if (object_count >= object_capacity) {
        int new_object_capacity = 0;
        if (object_capacity == 0) {new_object_capacity = 16;}
        else {new_object_capacity = object_capacity * 2;}
        rigidbody *new_objects_array = realloc (obj_per_scene, new_object_capacity * sizeof (rigidbody));
        if (!new_objects_array) {fprintf (stderr, "Maximum object count reached.\n"); return -1;}
        obj_per_scene = new_objects_array;
        object_capacity = new_object_capacity;
    } rigidbody_initialisation_cube (&obj_per_scene [object_count], position, half_extensions, mass);
    obj_per_scene [object_count].type = object_cube;
    int current_object_index = object_count;
    object_count += 1;
    return current_object_index;
} void scene_init_default (void) {
    scene_clear ();
    // Centered Immobile Sphere
    int object_grey_index = scene_add_object (2.0f, 0.0f, (vector3) {0.0f, 2.0f, 0.0f});
    obj_per_scene [object_grey_index].colour = (vector3) {0.8f, 0.8f, 0.8f}; // GY
} void scene_clear (void) {
    object_count = 0;
    joint_init_pool ();
}
