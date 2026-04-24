#include "object_selector.h"
int selected_object = -1;
int selector_ray_tracing (void) {
    vector3 ray_origin_camera_position_vector = main_camera_fov.position;
    vector3 ray_direction_normalised_vector = vector3_normalisation (main_camera_fov.front);
    float closest_intersection_distance_to_camera = 1e30;
    int closest_hit_rigidbody_object_index = -1;
    for (int current_object_iterator_index = 0; current_object_iterator_index < object_count; current_object_iterator_index++) {
        rigidbody *rigidbody_object_pointer = &obj_per_scene [current_object_iterator_index];
        //Vector: Ray Origin to Sphere Centre
        vector3 ray_origin_to_sphere_centre_vector = vector3_subtraction (rigidbody_object_pointer->position, ray_origin_camera_position_vector);
        //Project origin_centre onto ray direction
        float distance_along_ray_direction_projection = vector3_dot (ray_origin_to_sphere_centre_vector, ray_direction_normalised_vector);
        if (distance_along_ray_direction_projection < 0) {continue;} //Behind Camera
        //Perpendicular distance squared
        vector3 closest_point_on_ray_vector = vector3_scaling (ray_direction_normalised_vector, distance_along_ray_direction_projection);
        vector3 perpendicular_offset_vector = vector3_subtraction (ray_origin_to_sphere_centre_vector, closest_point_on_ray_vector);
        float perpendicular_distance_to_ray_squared = vector3_length_squared (perpendicular_offset_vector);
        //Collision Check (Radius)
        if (perpendicular_distance_to_ray_squared <= (rigidbody_object_pointer->radius * rigidbody_object_pointer->radius)) {
            if (distance_along_ray_direction_projection < closest_intersection_distance_to_camera) {
                closest_intersection_distance_to_camera = distance_along_ray_direction_projection;
                closest_hit_rigidbody_object_index = current_object_iterator_index;
            }
        }
    } selected_object = closest_hit_rigidbody_object_index;
    return closest_hit_rigidbody_object_index;
} void clear_selection (void) {
    selected_object = -1;
} void selector_apply_force_impulse (float impulse_magnitude_value) {
    if ((selected_object < 0) || (selected_object >= object_count)) {return;}
    rigidbody *selected_rigidbody_object_pointer = &obj_per_scene [selected_object];
    vector3 impulse_application_vector = vector3_scaling (main_camera_fov.front, impulse_magnitude_value);
    rb_apply_forces_perfect (selected_rigidbody_object_pointer, impulse_application_vector);
}
