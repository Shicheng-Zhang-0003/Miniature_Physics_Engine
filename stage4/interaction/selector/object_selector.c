#include "object_selector.h"
int selected_object = -1;
int selector_ray_tracing (void) {
    vector3 origin_ray_view = main_camera_fov.position;
    vector3 ray_direction = vector3_normalisation (main_camera_fov.front);
    float closest_distance = 1e30;
    int closest_index = -1;
    for (int iteration = 0; iteration < object_count; iteration++) {
        rigidbody *rb = &obj_per_scene [iteration];
        //Vector from the origin of the ray to centre of the sphere
        vector3 origin_centre = vector3_subtraction (rb->position, origin_ray_view);
        //Project origin_centre onto the direction of the ray to get the closest point on the vector
        float t = vector3_dot (origin_centre, ray_direction);
        if (t < 0) {continue;} //Spehre in question is behind the camera view
        //Perpendicular distance squared, from centre of the sphere to the ray
        vector3 projected = vector3_scaling (ray_direction, t);
        vector3 perspective = vector3_subtraction (origin_centre, projected);
        float distance_squared = vector3_length_squared (perspective);
        //If the ray passes through within the radius of the sphere object, hit is registered
        if (distance_squared <= (rb->radius * rb->radius)) {
            if (t < closest_distance) {
                closest_distance = t;
                closest_index = iteration;
            }
        }
    } selected_object = closest_index;
    return closest_index;
} void clear_selection (void) {
    selected_object = -1;
} void selector_apply_force_impulse (float magnitude) {
    if ((selected_object < 0) || (selected_object >= object_count)) {return;}
    rigidbody *rb = &obj_per_scene [selected_object];
    vector3 impulse = vector3_scaling (main_camera_fov.front, magnitude);
    rb_apply_forces_perfect (rb, impulse);
}
