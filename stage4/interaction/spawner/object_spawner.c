#include "object_spawner.h"
void spawner_launch_sphere (float radius, float mass, float speed) {
    //Spawn the object just very slightly in front of the camera (no collision)
    vector3 spawn_position = vector3_addition (main_camera_fov.position, vector3_scaling (main_camera_fov.front, radius + 1.0));
    int idx = scene_add_object (radius, mass, spawn_position);
    if (idx < 0) {return;} //Scene already occupied, or stack failure (SAO/SKF)
    //Velocity to the object, camera direction
    obj_per_scene [idx].velocity = vector3_scaling (main_camera_fov.front, speed);
    //Give the object a random colour for now, distinguish objects
    obj_per_scene [idx].colour = (vector3) {
        0.4 + 0.6 * ((float) (idx % 3) / 2.0),
        0.4 + 0.6 * ((float) ((idx + 1) % 3) / 2.0),
        0.4 + 0.6 * ((float) ((idx + 2) % 3) / 2.0)
    };
} void spawner_static_sphere (float radius, float mass, vector3 position) {
    int idx = scene_add_object (radius, mass, position);
    if (idx < 0) {return;} //SAO/SKF
    obj_per_scene [idx].colour = (vector3) {0.8, 0.8, 0.8};
}
