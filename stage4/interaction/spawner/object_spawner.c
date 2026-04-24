#include "object_spawner.h"
void spawner_launch_sphere (float spherical_object_radius, float physical_mass_value, float initial_launch_velocity_magnitude) {
    //Spawn the object just very slightly in front of the camera (no collision)
    vector3 initial_spawn_position_vector = vector3_addition (main_camera_fov.position, vector3_scaling (main_camera_fov.front, spherical_object_radius + 1.0));
    int spawned_object_scene_index = scene_add_object (spherical_object_radius, physical_mass_value, initial_spawn_position_vector);
    if (spawned_object_scene_index < 0) {return;} //Scene already occupied, or stack failure (SAO/SKF)
    //Velocity to the object, camera direction
    obj_per_scene [spawned_object_scene_index].velocity = vector3_scaling (main_camera_fov.front, initial_launch_velocity_magnitude);
    //Give the object a random colour for now, distinguish objects
    obj_per_scene [spawned_object_scene_index].colour = (vector3) {
        0.4 + 0.6 * ((float) (spawned_object_scene_index % 3) / 2.0),
        0.4 + 0.6 * ((float) ((spawned_object_scene_index + 1) % 3) / 2.0),
        0.4 + 0.6 * ((float) ((spawned_object_scene_index + 2) % 3) / 2.0)
    };
} void spawner_static_sphere (float spherical_object_radius, float physical_mass_value, vector3 static_spawn_position_vector) {
    int spawned_object_scene_index = scene_add_object (spherical_object_radius, physical_mass_value, static_spawn_position_vector);
    if (spawned_object_scene_index < 0) {return;} //SAO/SKF
    obj_per_scene [spawned_object_scene_index].colour = (vector3) {0.8, 0.8, 0.8};
}
