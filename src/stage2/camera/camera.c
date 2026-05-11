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
} void initalise_camera (camera *camera_object, vector3 starting_position) {
    camera_object -> position = starting_position;
    camera_object -> yaw = -90.0f; //Straight Forwards (Negative Z Axis)
    camera_object -> pitch = 0.0f; //Flat Horizon View
    camera_object -> movement_speed = 25.0f; //25 Units of Movement * s ^ -1
    camera_object -> mouse_sensitivity = 0.1f;
    camera_update_vectors (camera_object);
} /* void camera_view_matrix (camera *camera_object, float *matrix_output) {
    //Target is where we will be standing --> In ADDITION to where we are currently looking
    vector3 target_position = vector3_addition (camera_object -> position, camera_object -> forward_vector);
    //Calculate orthogonal axis
    vector3 forward_axis = vector3_normalisation (vector3_subtraction (target_position, camera_object -> position));
    vector3 vertical_axis = vector3_normalisation (camera_object -> vertical_vector);
    vector3 right_axis = vector3_normalisation (vector3_cross (forward_axis, vertical_axis));
    //U-axis is directly orthogonal to F-axis and R-axis
    vertical_axis = vector3_cross (right_axis, forward_axis);
    //OpenGL matrices:
    //Top left 3 ^ 3 lattice is for defining rotational values (Right/Side, Up/Vertical, Front)
    //Right Most Column is for defining translational motion (moving world view away from the camera fov)

    //C0
    matrix_output [0] = right_axis.x, matrix_output [1] = vertical_axis.x, matrix_output [2] = -forward_axis.x, matrix_output [3] = 0.0f;
    //C1
    matrix_output [4] = right_axis.y, matrix_output [5] = vertical_axis.y, matrix_output [6] = -forward_axis.y, matrix_output [7] = 0.0f;
    //C2
    matrix_output [8] = right_axis.z, matrix_output [9] = vertical_axis.z, matrix_output [10] = -forward_axis.z, matrix_output [11] = 0.0f;
    //C3
    matrix_output [12] = -vector3_dot (right_axis, camera_object -> position);
    matrix_output [13] = -vector3_dot (vertical_axis, camera_object -> position);
    matrix_output [14] = -vector3_dot (forward_axis, camera_object -> position);
    matrix_output [15] = 1.0f;
} //Movement */
void camera_move_forward (camera *camera_object, float delta_time) {
    vector3 camera_velocity = vector3_scaling (camera_object -> forward_vector, camera_object -> movement_speed * delta_time);
    camera_object -> position = vector3_addition (camera_object -> position, camera_velocity);
} void camera_move_backward (camera *camera_object, float delta_time) {
    vector3 camera_velocity = vector3_scaling (camera_object -> forward_vector, camera_object -> movement_speed * delta_time);
    camera_object -> position = vector3_subtraction (camera_object -> position, camera_velocity);
} //Strafe uses the Right/Side Vector
void camera_move_left (camera *camera_object, float delta_time) {
    vector3 camera_velocity = vector3_scaling (camera_object -> side_vector, camera_object -> movement_speed * delta_time);
    camera_object -> position = vector3_subtraction (camera_object -> position, camera_velocity);
} void camera_move_right (camera *camera_object, float delta_time) {
    vector3 camera_velocity = vector3_scaling (camera_object -> side_vector, camera_object -> movement_speed * delta_time);
    camera_object -> position = vector3_addition (camera_object -> position, camera_velocity);
}
