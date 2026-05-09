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
} void scene_init_default (void) {
    scene_clear ();
    //Define Object Colours
    int object_red_index = scene_add_object (1.0, 1.0, (vector3) {0.0, 20.0, 0.0});
    int object_green_index = scene_add_object (1.0, 1.0, (vector3) {0.5, 15.0, 0.0});
    int object_blue_index = scene_add_object (1.5, 2.0, (vector3) {0.0, 10.0, 0.0});
    int object_grey_index = scene_add_object (2.0, 0.0, (vector3) {0.0, 2.0, 0.0});
    obj_per_scene [object_red_index].colour = (vector3) {1.0, 0.3, 0.3}; // R
    obj_per_scene [object_green_index].colour = (vector3) {0.3, 1.0, 0.3}; // G
    obj_per_scene [object_blue_index].colour = (vector3) {0.3, 0.3, 1.0}; // B
    obj_per_scene [object_grey_index].colour = (vector3) {0.8, 0.8, 0.8}; // GY
} void scene_clear (void) {
    object_count = 0;
}
