#include "object_spawner.h"
void spawner_launch_sphere (float spherical_radius, float physical_mass, float launch_speed) {
    //Spawn the object just very slightly in front of the camera (no collision)
    vector3 initial_spawn_position = vector3_addition (main_camera_fov.position, vector3_scaling (main_camera_fov.forward_vector, spherical_radius + 1.0));
    int newly_spawned_object_index = scene_add_object (spherical_radius, physical_mass, initial_spawn_position);
    if (newly_spawned_object_index < 0) {return;} //Scene already occupied, or stack failure (SAO/SKF)
    //Velocity to the object, camera direction
    obj_per_scene [newly_spawned_object_index].velocity = vector3_scaling (main_camera_fov.forward_vector, launch_speed);
    //Give the object a random colour for now, distinguish objects
    obj_per_scene [newly_spawned_object_index].colour = (vector3) {
        0.4 + 0.6 * ((float) (newly_spawned_object_index % 3) / 2.0),
        0.4 + 0.6 * ((float) ((newly_spawned_object_index + 1) % 3) / 2.0),
        0.4 + 0.6 * ((float) ((newly_spawned_object_index + 2) % 3) / 2.0)
    };
} void spawner_static_sphere (float spherical_radius, float physical_mass, vector3 static_position) {
    int newly_spawned_object_index = scene_add_object (spherical_radius, physical_mass, static_position);
    if (newly_spawned_object_index < 0) {return;} //SAO/SKF
    obj_per_scene [newly_spawned_object_index].colour = (vector3) {0.8, 0.8, 0.8};
}
