#include "scene_saving.h"
#include <stdio.h>
int save_scene (const char *filepath) {
    FILE *source = fopen (filepath, "w");
    if (!source) {fprintf (stderr, "Error SVF01, could not open %s\n", filepath); return 0;}
    //List number of objects first
    fprintf (source, "%d\n", object_count);
    for (int step = 0; step < object_count; step++) {
        rigidbody *rb = &obj_per_scene [step];
        //Each object listed on each line
        //radius, mass, static_status
        //position.x, position.y, position.z
        //velocity.x, velocity.y, velocity.z
        //angular_velocity.x, angular_velocity.y, angular_velocity.z
        //orientation.w, orientation.x, orientation.y, orientation.z
        //colour.x, colour.y, colour.z
        //restitution value
        fprintf (source, "%.6f %.6f %d\n", rb -> radius, rb -> mass, (int) rb -> static_state);
        fprintf (source, "%.6f %.6f %.6f\n", rb -> position.x, rb -> position.y, rb -> position.z);
        fprintf (source, "%.6f %.6f %.6f\n", rb -> velocity.x, rb -> velocity.y, rb -> velocity.z);
        fprintf (source, "%.6f %.6f %.6f\n", rb -> angular_velocity.x, rb -> angular_velocity.y, rb -> angular_velocity.z);
        fprintf (source, "%.6f %.6f %.6f %.6f\n", rb -> orientation.w, rb -> orientation.x, rb -> orientation.y, rb -> orientation.z);
        fprintf (source, "%.6f %.6f %.6f\n", rb -> colour.x, rb -> colour.y, rb -> colour.z);
        fprintf (source, "%.6f\n", rb -> restitution);
    } fclose (source);
    return 1;
}
