#include "scene_load.h"
#include "../../stage3/scene/scene_init.h"
#include <stdio.h>
int scene_loading (const char *file_src) {
    FILE *source = fopen (file_src, "r");
    if (!source) {fprintf (stderr, "Error LDF01: Loading File Error %s\n", file_src); return 0;}
    int count_objects = 0;
    fscanf (source, "%d\n", &count_objects);
    //Clear the current scene before rendering saved objects
    scene_clear ();
    for (int step = 0; step < count_objects; step++) {
        float radius, mass, restitution;
        int static_state;
        vector3 position, velocity, angular_velocity, colour;
        vector4 orientation;
        fscanf (source, "%f %f %d\n", &radius, &mass, &static_state);
        fscanf (source, "%f %f %f\n", &position.x, &position.y, &position.z);
        fscanf (source, "%f %f %f\n", &velocity.x, &velocity.y, &velocity.z);
        fscanf (source, "%f %f %f\n", &angular_velocity.x, &angular_velocity.y, &angular_velocity.z);
        fscanf (source, "%f %f %f %f\n", &orientation.w, &orientation.x, &orientation.y, &orientation.z);
        fscanf (source, "%f %f %f\n", &colour.x, &colour.y, &colour.z);
        fscanf (source, "%f\n", &restitution);
        // Use mass 0 if static to preserve static_state behaviour
        float loading_mass;
        if (static_state) {loading_mass = 0;}
        else {loading_mass = mass;}
        int idx = scene_add_object (radius, loading_mass, position);
        if (idx < 0) {fclose (source); return 0;}
        obj_per_scene [idx].velocity = velocity;
        obj_per_scene [idx].angular_velocity = angular_velocity;
        obj_per_scene [idx].orientation = orientation;
        obj_per_scene [idx].colour = colour;
        obj_per_scene [idx].restitution = restitution;
    } fclose (source);
    return 1;
}
