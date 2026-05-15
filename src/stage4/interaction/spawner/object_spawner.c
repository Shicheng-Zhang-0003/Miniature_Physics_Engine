#include "object_spawner.h"
float spawn_mass = 1.0f;
float spawn_radius = 0.5f;
float spawn_cube_mass = 2.0f;
float spawn_cube_extent = 0.5f;
float spawn_speed = 20.0f;
float friction_static = 0.3f;
float friction_kinetic = 0.2f;
void spawner_launch_sphere (float spherical_radius, float physical_mass, float launch_speed) {
    //Spawn the object just very slightly in front of the camera (no collision)
    vector3 initial_spawn_position = vector3_addition (main_camera_fov.position, vector3_scaling (main_camera_fov.forward_vector, spherical_radius + 1.0f));
    int newly_spawned_object_index = scene_add_object (spherical_radius, physical_mass, initial_spawn_position);
    if (newly_spawned_object_index < 0) {return;} //Scene already occupied, or stack failure (SAO/SKF)
    //Velocity to the object, camera direction
    obj_per_scene [newly_spawned_object_index].velocity = vector3_scaling (main_camera_fov.forward_vector, launch_speed);
    // Friction
    obj_per_scene [newly_spawned_object_index].friction_static = friction_static;
    obj_per_scene [newly_spawned_object_index].friction_kinetic = friction_kinetic;
    //Give the object a random colour for now, distinguish objects
    obj_per_scene [newly_spawned_object_index].colour = (vector3) {0.4f + 0.6f * ((float) (newly_spawned_object_index % 3) / 2.0f), 0.4f + 0.6f * ((float) ((newly_spawned_object_index + 1) % 3) / 2.0f), 0.4f + 0.6f * ((float) ((newly_spawned_object_index + 2) % 3) / 2.0f)};
} void spawner_static_sphere (float spherical_radius, float physical_mass, vector3 static_position) {
    int newly_spawned_object_index = scene_add_object (spherical_radius, physical_mass, static_position);
    if (newly_spawned_object_index < 0) {return;} //SAO/SKF
    obj_per_scene [newly_spawned_object_index].friction_static = friction_static;
    obj_per_scene [newly_spawned_object_index].friction_kinetic = friction_kinetic;
    obj_per_scene [newly_spawned_object_index].colour = (vector3) {0.8f, 0.8f, 0.8f};
} void spawner_static_cube (vector3 position, vector3 half_extensions, float physical_mass) {
    if (object_count >= object_capacity) {return;}
    rigidbody *cube = &obj_per_scene [object_count];
    rigidbody_initialisation_cube (cube, position, half_extensions, physical_mass);
    cube -> type = object_cube;
    object_count += 1;
} void spawner_launch_cube (vector3 position, vector3 half_extensions, float physical_mass) {
    if (object_count >= object_capacity) {return;}
    rigidbody *cube = &obj_per_scene [object_count];
    rigidbody_initialisation_cube (cube, position, half_extensions, physical_mass);
    cube -> type = object_cube;
    cube -> velocity = vector3_scaling (main_camera_fov.forward_vector, spawn_speed);
    cube -> friction_static = friction_static;
    cube -> friction_kinetic = friction_kinetic;
    cube -> colour = (vector3) {0.6f + 0.4f * ((float) (object_count % 3) / 2.0f), 0.4f + 0.6f * ((float) ((object_count + 1) % 3) / 2.0f), 0.2f + 0.8f * ((float) ((object_count + 2) % 3) / 2.0f)};
    object_count += 1;
}
