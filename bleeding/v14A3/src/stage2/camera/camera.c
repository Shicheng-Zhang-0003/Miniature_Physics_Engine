#include "camera.h"
#include <math.h>
void camera_update_vectors (camera *camera_object) {
    //Front Vector --> Pitch and Yaw
    //Deg to Rad
    float yaw_radians = camera_object -> yaw * degrad;
    float pitch_radians = camera_object -> pitch * degrad;
    vector3 updated_forward_vector;
    updated_forward_vector.x = cosf (yaw_radians) * cosf (pitch_radians);
    updated_forward_vector.y = sinf (pitch_radians);
    updated_forward_vector.z = sinf (yaw_radians) * cosf (pitch_radians);
    //Normalise Frontal Vector
    camera_object -> forward_vector = vector3_normalisation (updated_forward_vector);
    //Calculate Right Side and Vertical Vectors
    //Cross of Frontal and Up view {0, 1, 0} --> Right Axis
    vector3 global_up_vector = {0.0f, 1.0f, 0.0f};
    camera_object -> side_vector = vector3_normalisation (vector3_cross (camera_object -> forward_vector, global_up_vector));
    //Cross right and front gives the UP axis
    camera_object -> vertical_vector = vector3_normalisation (vector3_cross (camera_object -> side_vector, camera_object -> forward_vector));
} void initialize_camera (camera *camera_object, vector3 starting_position) {
    camera_object -> position = starting_position;
    camera_object -> yaw = -90.0f; //Straight Forwards (Negative Z Axis)
    camera_object -> pitch = 0.0f; //Flat Horizon View
    camera_object -> movement_speed = 25.0f; //25 Units of Movement * s ^ -1
    camera_object -> mouse_sensitivity = 0.1f;
    camera_object -> vertical_velocity = 0.0f;
    camera_object -> horizontal_velocity = (vector3) {0.0f, 0.0f, 0.0f};
    camera_update_vectors (camera_object);
} //Movement Vectoring
void camera_move_forward (camera *camera_object, float delta_time) {
    vector3 flat_forward = vector3_normalisation ((vector3) {camera_object -> forward_vector.x, 0.0f, camera_object -> forward_vector.z});
    camera_object -> horizontal_velocity = vector3_addition (camera_object -> horizontal_velocity, vector3_scaling (flat_forward, camera_object -> movement_speed * 8.0f * delta_time));
} void camera_move_backward (camera *camera_object, float delta_time) {
    vector3 flat_forward = vector3_normalisation ((vector3) {camera_object -> forward_vector.x, 0.0f, camera_object -> forward_vector.z});
    camera_object -> horizontal_velocity = vector3_subtraction (camera_object -> horizontal_velocity, vector3_scaling (flat_forward, camera_object -> movement_speed * 8.0f * delta_time));
} //Strafe uses the Right/Side Vector
void camera_move_left (camera *camera_object, float delta_time) {
    camera_object -> horizontal_velocity = vector3_subtraction (camera_object -> horizontal_velocity, vector3_scaling (camera_object -> side_vector, camera_object -> movement_speed * 8.0f * delta_time));
} void camera_move_right (camera *camera_object, float delta_time) {
    camera_object -> horizontal_velocity = vector3_addition (camera_object -> horizontal_velocity, vector3_scaling (camera_object -> side_vector, camera_object -> movement_speed * 8.0f * delta_time));
}
