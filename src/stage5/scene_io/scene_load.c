#include "scene_load.h"
#include "../../stage3/scene/scene_init.h"
#include <stdio.h>
int scene_loading (const char *file_source_path) {
    FILE *file_source = fopen (file_source_path, "r");
    if (!file_source) {fprintf (stderr, "Error LDF01: Loading File Error %s\n", file_source_path); return 0;}
    int scene_object_count = 0;
    fscanf (file_source, "%d\n", &scene_object_count);
    //Clear the current scene before rendering saved objects
    scene_clear ();
    for (int object_index = 0; object_index < scene_object_count; object_index++) {
        float object_radius, object_mass, object_restitution;
        int object_static_state, object_type_int;
        vector3 object_half_extensions;
        vector3 object_position, object_velocity, object_angular_velocity, object_colour;
        vector4 object_orientation;
        fscanf (file_source, "%f %f %d %d %f %f %f\n", &object_radius, &object_mass, &object_static_state, &object_type_int, &object_half_extensions.x, &object_half_extensions.y, &object_half_extensions.z);
        fscanf (file_source, "%f %f %f\n", &object_position.x, &object_position.y, &object_position.z);
        fscanf (file_source, "%f %f %f\n", &object_velocity.x, &object_velocity.y, &object_velocity.z);
        fscanf (file_source, "%f %f %f\n", &object_angular_velocity.x, &object_angular_velocity.y, &object_angular_velocity.z);
        fscanf (file_source, "%f %f %f %f\n", &object_orientation.w, &object_orientation.x, &object_orientation.y, &object_orientation.z);
        fscanf (file_source, "%f %f %f\n", &object_colour.x, &object_colour.y, &object_colour.z);
        fscanf (file_source, "%f\n", &object_restitution);
        // Use mass 0 if static to preserve static_state behaviour
        float loaded_physical_mass;
        if (object_static_state) {loaded_physical_mass = 0.0f;}
        else {loaded_physical_mass = object_mass;}
        object_type loaded_type = (object_type) object_type_int;
        int loaded_object_index;
        if (loaded_type == object_cube) {
            if (object_count >= object_capacity) {
                int new_capacity;
                if (object_capacity == 0) {new_capacity = 16;}
                else {new_capacity = object_capacity * 2;}
                rigidbody *new_object_array = realloc (obj_per_scene, new_capacity * sizeof (rigidbody));
                if (!new_object_array) {fclose (file_source); return 0;}
                obj_per_scene = new_object_array;
                object_capacity = new_capacity;
            }
 rigidbody_initialisation_cube (&obj_per_scene [object_count], object_position, object_half_extensions, loaded_physical_mass);
            obj_per_scene [object_count].type = object_cube;
            loaded_object_index = object_count;
            object_count += 1;
        } else {loaded_object_index = scene_add_object (object_radius, loaded_physical_mass, object_position);}
        if (loaded_object_index < 0) {fclose (file_source); return 0;}
        obj_per_scene [loaded_object_index].static_state = (bool) object_static_state;
        obj_per_scene [loaded_object_index].velocity = object_velocity;
        obj_per_scene [loaded_object_index].angular_velocity = object_angular_velocity;
        obj_per_scene [loaded_object_index].orientation = object_orientation;
        obj_per_scene [loaded_object_index].colour = object_colour;
        obj_per_scene [loaded_object_index].restitution = object_restitution;
    } fclose (file_source);
    return 1;
}
