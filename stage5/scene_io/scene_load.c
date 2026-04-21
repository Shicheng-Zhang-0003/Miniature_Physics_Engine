#include "scene_load.h"
#include "../../stage3/scene/scene_init.h"
#include <stdio.h>
int scene_loading (const char *file_src) {
    FILE *source = fopen (file_src, "r");
    if (!source) {fprintf (stderr, "Error LDF01: Loading File Error %s\n", file_src); return 0;}
    int count_objects = 0;
    fscanf (file_src, "%d\n", count);
    //Clear the current scene before rendering saved objects
    scene_clear ();
    for (int step = 0; step < count; step++) {
        float radius, mass, restitution;
        int static_state;
        vector3 position, velocity, angular_velocity, colour;
        vector4 orientation;
        fscanf (file_src, "%f %f %d\n", &radius, &mass, &static_state);
        fscanf (file_src, "%f %f %f\n", &position.x, &position.y, &position.z);
        fscanf (file_src, "%f %f %f\n", &velocity.x, &velocity.y, &velocity.z);
        fscanf (file_src, "%f %f %f\n", &angular_velocity.x, &angular_velocity.y, &angular_velocity.z);
        fscanf (file_src, "%f %f %f %f\n", &orientation.w, &orientation.x, &orientation.y, &orientation.z);
        fscanf (file_src, "%f %f %f\n", &colour.x, &colour.y, &colour.z);
        fscanf (file_src, "%f\n", &restitution);
        // Use mass 0 if static to preserve static_state behaviour
        int idx = scene_add_object (radius, static_state ? 0 : mass, position);
        if (idx < 0) {fclose (f); return 0;}
        obj_per_scene [idx].velocity = velocity;
        obj_per_scene [idx].angular_velocity = angular_velocity;
        obj_per_scene [idx].orientation = orientation;
        obj_per_scene [idx].colour = colour;
        obj_per_scene [idx].restitution = restitution;
    } fclose (file_src);
    return 1;
}
