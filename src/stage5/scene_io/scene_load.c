#include "scene_load.h"
#include "../../stage3/scene/scene_init.h"
#include <stdio.h>
#include <stdlib.h>
int scene_loading (const char *file_source_path) {
    FILE *file_source = fopen (file_source_path, "r");
    if (!file_source) {fprintf (stderr, "Error LDF01: Loading File Error %s\n", file_source_path); return 0;}
    int scene_object_count = 0;
    if ((fscanf (file_source, "%d\n", &scene_object_count) != 1) || (scene_object_count < 0)) {fclose (file_source); return 0;}
    rigidbody *loaded_scene_objects = NULL;
    if (scene_object_count > 0) {
        loaded_scene_objects = malloc (scene_object_count * sizeof (rigidbody));
        if (!loaded_scene_objects) {fclose (file_source); return 0;}
    } for (int object_index = 0; object_index < scene_object_count; object_index++) {
        float object_radius, object_mass, object_restitution;
        int object_static_state, object_type_int;
        vector3 object_half_extensions;
        vector3 object_position, object_velocity, object_angular_velocity, object_colour;
        vector4 object_orientation;
        if (fscanf (file_source, "%f %f %d %d %f %f %f\n", &object_radius, &object_mass, &object_static_state, &object_type_int, &object_half_extensions.x, &object_half_extensions.y, &object_half_extensions.z) != 7) {free (loaded_scene_objects); fclose (file_source); return 0;}
        if (fscanf (file_source, "%f %f %f\n", &object_position.x, &object_position.y, &object_position.z) != 3) {free (loaded_scene_objects); fclose (file_source); return 0;}
        if (fscanf (file_source, "%f %f %f\n", &object_velocity.x, &object_velocity.y, &object_velocity.z) != 3) {free (loaded_scene_objects); fclose (file_source); return 0;}
        if (fscanf (file_source, "%f %f %f\n", &object_angular_velocity.x, &object_angular_velocity.y, &object_angular_velocity.z) != 3) {free (loaded_scene_objects); fclose (file_source); return 0;}
        if (fscanf (file_source, "%f %f %f %f\n", &object_orientation.w, &object_orientation.x, &object_orientation.y, &object_orientation.z) != 4) {free (loaded_scene_objects); fclose (file_source); return 0;}
        if (fscanf (file_source, "%f %f %f\n", &object_colour.x, &object_colour.y, &object_colour.z) != 3) {free (loaded_scene_objects); fclose (file_source); return 0;}
        if (fscanf (file_source, "%f\n", &object_restitution) != 1) {free (loaded_scene_objects); fclose (file_source); return 0;}
        object_type loaded_type = (object_type) object_type_int;
        if (loaded_type == object_cube) {rigidbody_initialisation_cube (&loaded_scene_objects [object_index], object_position, object_half_extensions, object_mass);}
        else if (loaded_type == object_sphere) {rigidbody_initialisation_sphere (&loaded_scene_objects [object_index], object_radius, object_mass, object_position);}
        else {free (loaded_scene_objects); fclose (file_source); return 0;}
        loaded_scene_objects [object_index].static_state = (bool) object_static_state;
        if (loaded_scene_objects [object_index].static_state) {loaded_scene_objects [object_index].inverse_mass = 0.0f;}
        else if (loaded_scene_objects [object_index].mass > 0.0f) {loaded_scene_objects [object_index].inverse_mass = 1.0f / loaded_scene_objects [object_index].mass;}
        else {loaded_scene_objects [object_index].inverse_mass = 0.0f; loaded_scene_objects [object_index].static_state = true;}
        loaded_scene_objects [object_index].velocity = object_velocity;
        loaded_scene_objects [object_index].angular_velocity = object_angular_velocity;
        loaded_scene_objects [object_index].orientation = vector4_normalisation (object_orientation);
        loaded_scene_objects [object_index].colour = object_colour;
        loaded_scene_objects [object_index].restitution = object_restitution;
        rigidbody_update_axes (&loaded_scene_objects [object_index]);
    } fclose (file_source);
    scene_clear ();
    if (scene_object_count > object_capacity) {
        rigidbody *new_object_array = realloc (obj_per_scene, scene_object_count * sizeof (rigidbody));
        if (!new_object_array) {free (loaded_scene_objects); return 0;}
        obj_per_scene = new_object_array;
        object_capacity = scene_object_count;
    } for (int object_index = 0; object_index < scene_object_count; object_index++) {obj_per_scene [object_index] = loaded_scene_objects [object_index];}
    object_count = scene_object_count;
    free (loaded_scene_objects);
    return 1;
}
