#include "object_selector.h"
int selected_object = -1;
int selector_ray_tracing (void) {
    vector3 ray_origin_position = main_camera_fov.position;
    vector3 ray_direction_vector = vector3_normalisation (main_camera_fov.forward_vector);
    float closest_hit_distance = 1e30;
    int closest_object_index = -1;
    for (int object_index = 0; object_index < object_count; object_index++) {
        rigidbody *rigid_body_pointer = &obj_per_scene [object_index];
        //Vector: Ray Origin to Sphere Centre
        vector3 origin_to_center_vector = vector3_subtraction (rigid_body_pointer -> position, ray_origin_position);
        //Project origin_centre onto ray direction
        float projection_length_along_ray = vector3_dot (origin_to_center_vector, ray_direction_vector);
        if (projection_length_along_ray < 0) {continue;} //Behind Camera
        //Perpendicular distance squared
        vector3 closest_point_on_ray_vector = vector3_scaling (ray_direction_vector, projection_length_along_ray);
        vector3 perpendicular_displacement_vector = vector3_subtraction (origin_to_center_vector, closest_point_on_ray_vector);
        float perpendicular_distance_squared = vector3_length_squared (perpendicular_displacement_vector);
        //Collision Check (Radius)
        if (perpendicular_distance_squared <= (rigid_body_pointer -> radius * rigid_body_pointer -> radius)) {
            if (projection_length_along_ray < closest_hit_distance) {
                closest_hit_distance = projection_length_along_ray;
                closest_object_index = object_index;
            }
        }
    } selected_object = closest_object_index;
    return closest_object_index;
} void clear_selection (void) {
    selected_object = -1;
} void selector_apply_force_impulse (float impulse_magnitude) {
    if ((selected_object < 0) || (selected_object >= object_count)) {return;}
    rigidbody *selected_rigid_body = &obj_per_scene [selected_object];
    vector3 applied_impulse_vector = vector3_scaling (main_camera_fov.forward_vector, impulse_magnitude);
    rb_apply_forces_perfect (selected_rigid_body, applied_impulse_vector);
}
