#include "scene_init.h"
#include <stdlib.h>
//Maximum Object count
int scene_add_object (float radius, float mass, vector3 position) {
    if (object_count >= object_capacity) {
        int new_capacity = 0;
        if (object_capacity == 0) {new_capacity == 16;}
        else {new_capacity = object_capacity * 2;}
        rigidbody *new_array_objects = realloc (obj_per_scene, new_capacity * sizeof (rigidbody));
        if (!new_array_objects) {fprintf (stderr, "Maximum object count reached.\n"); return -1;}
        obj_per_scene = new_array_objects;
        object_capacity = new_capacity;
    } rigidbody_initialisation_sphere (&obj_per_scene [object_count], radius, mass, position);
    int index = object_count;
    object_count += 1;
    return index;
} void scene_init_default (void) {
    scene_clear ();
    //Define Object Colours
    int a = scene_add_object (1.0, 1.0, (vector3) {0.0, 20.0, 0.0});
    int b = scene_add_object (1.0, 1.0, (vector3) {0.5, 15.0, 0.0});
    int c = scene_add_object (1.5, 2.0, (vector3) {0.0, 10.0, 0.0});
    int d = scene_add_object (2.0, 0.0, (vector3) {0.0, 2.0, 0.0});
    obj_per_scene[a].colour = (vector3) {1.0, 0.3, 0.3}; // R
    obj_per_scene[b].colour = (vector3) {0.3, 1.0, 0.3}; // G
    obj_per_scene[c].colour = (vector3) {0.3, 0.3, 1.0}; // B
    obj_per_scene[d].colour = (vector3) {0.8, 0.8, 0.8}; // GY
    //3 Spheres for initial render, testing gravity and collision premises (different height spawn)
    scene_add_object (1.0, 1.0, (vector3) {0.0, 20.0, 0.0});
    scene_add_object (1.0, 1.0, (vector3) {0.5, 15.0, 0.0});
    scene_add_object (1.5, 2.0, (vector3) {0.0, 10.0, 0.0});
    //Add a high-mass sphere as a collision bumper testing device at the bottom
    scene_add_object (2.0, 0.0, (vector3) {0.0, 2.0, 0.0});
} void scene_clear (void) {
    object_count = 0;
}
