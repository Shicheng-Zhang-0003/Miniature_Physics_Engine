#include "scene_init.h"
//Maximum Object count
#define max_objects 10
int scene_add_object (float radius, float mass, vector3 position) {
    if (object_count >= max_objects) {fprintf (stderr, "Maximum object count reached.\n"); return -1;}
    rigidbody_initialisation_sphere (&obj_per_scene [object_count], radius, mass, position);
    int index = object_count;
    object_count += 1;
    return index;
} void scene_init_default (void) {
    scene_clear ();
    //3 Spheres for initial render, testing gravity and collision premises (different height spawn)
    scene_add_object (1.0, 1.0, (vector3) {0.0, 20.0, 0.0});
    scene_add_object (1.0, 1.0, (vector3) {0.5, 15.0, 0.0});
    scene_add_object (1.5, 2.0, (vector3) {0.0, 10.0, 0.0});
    //Add a high-mass sphere as a collision bumper testing device at the bottom
    scene_add_object (2.0, 0.0, (vector3) {0.0, 2.0, 0.0});
} void scene_clear (void) {
    object_count = 0;
}
