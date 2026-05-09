#include "scene_saving.h"
#include <stdio.h>
int save_scene (const char *file_destination_path) {
    FILE *file_destination = fopen (file_destination_path, "w");
    if (!file_destination) {fprintf (stderr, "Error SVF01, could not open %s\n", file_destination_path); return 0;}
    //List number of objects first
    fprintf (file_destination, "%d\n", object_count);
    for (int object_index = 0; object_index < object_count; object_index++) {
        rigidbody *rigid_body_pointer = &obj_per_scene [object_index];
        //Each object listed on each line
        //radius, mass, static_status
        //position.x, position.y, position.z
        //velocity.x, velocity.y, velocity.z
        //angular_velocity.x, angular_velocity.y, angular_velocity.z
        //orientation.w, orientation.x, orientation.y, orientation.z
        //colour.x, colour.y, colour.z
        //restitution value
        fprintf (file_destination, "%.6f %.6f %d\n", rigid_body_pointer -> radius, rigid_body_pointer -> mass, (int) rigid_body_pointer -> static_state);
        fprintf (file_destination, "%.6f %.6f %.6f\n", rigid_body_pointer -> position.x, rigid_body_pointer -> position.y, rigid_body_pointer -> position.z);
        fprintf (file_destination, "%.6f %.6f %.6f\n", rigid_body_pointer -> velocity.x, rigid_body_pointer -> velocity.y, rigid_body_pointer -> velocity.z);
        fprintf (file_destination, "%.6f %.6f %.6f\n", rigid_body_pointer -> angular_velocity.x, rigid_body_pointer -> angular_velocity.y, rigid_body_pointer -> angular_velocity.z);
        fprintf (file_destination, "%.6f %.6f %.6f %.6f\n", rigid_body_pointer -> orientation.w, rigid_body_pointer -> orientation.x, rigid_body_pointer -> orientation.y, rigid_body_pointer -> orientation.z);
        fprintf (file_destination, "%.6f %.6f %.6f\n", rigid_body_pointer -> colour.x, rigid_body_pointer -> colour.y, rigid_body_pointer -> colour.z);
        fprintf (file_destination, "%.6f\n", rigid_body_pointer -> restitution);
    } fclose (file_destination);
    return 1;
}
