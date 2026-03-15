#include "camera.h"
#include <math.h>
void camera_update_vector_input (camera *cam) {
    //Front Vector --> Pitch and Yaw
    //Deg to Rad
    float yaw_radians = cam->yaw * degrad;
    float pitch_radians = cam->pitch * degrad;
    vector3 new_frontal;
    new_frontal.x = cosf (yaw_radians) * cosf (pitch_radians);
    new_frontal.y = sinf (pitch_radians);
    new_frontal.z = sinf (yaw_radians) * cosf (pitch_radians);
    //Normalise Frontal Vector
    cam->front = vector3_normalisation (new_frontal);
    //Calculate Right Side and Vertical Vectors
    //Cross of Frontal and Up view {0, 1, 0} --> Right Axis
    vector3 world_up_view = {0.0, 1.0, 0.0};
    cam->side = vector3_normalisation (vector3_cross (cam->front, world_up_view));
    //Cross right and front gives the UP axis
    cam->vertical = vector3_normalisation (vector3_cross (cam->side, cam->front));
} void initalise_camera (camera *cam, vector3 start_position) {
    cam->position = start_position;
    cam->yaw = -90.0; //Straight Forwards (Negative Z Axis)
    cam->pitch = 0.0; //Flat Horizon View
    cam->speed_movement_camera = 25.0; //25 Units of Movement * s ^ -1
    cam->sensitivity_mouse = 0.1;
    camera_update_vector_input (cam);
} void camera_view_matrix (camera *cam, float *matrix_out) {
    //Target is where we will be standing --> In ADDITION to where we are currently looking
    vector3 target = vector3_addition (cam->position, cam->front);
    //Calculate orthogonal axis 
    vector3 f_axis = vector3_normalisation (vector3_subtraction (target, cam->position));
    vector3 u_axis = vector3_normalisation (cam->vertical);
    vector3 r_axis = vector3_normalisation (vector3_cross (f_axis, u_axis));
    //U-axis is directly orthogonal to F-axis and R-axis
    u_axis = vector3_cross(r_axis, f_axis);
    //OpenGL matrices:
    //Top left 3 ^ 3 lattice is for defining rotational values (Right/Side, Up/Vertical, Front)
    //Right Most Column is for defining translational motion (moving world view away from the camera fov)

    //C0
    matrix_out [0] = r_axis.x, matrix_out [1] = u_axis.x, matrix_out [2] = -f_axis.x, matrix_out [3] = 0.0;
    //C1
    matrix_out [4] = r_axis.y, matrix_out [5] = u_axis.y, matrix_out [6] = -f_axis.y, matrix_out [7] = 0.0;
    //C2
    matrix_out [8] = r_axis.z, matrix_out [9] = u_axis.z, matrix_out [10] = -f_axis.z, matrix_out [11] = 0.0;
    //C3
    matrix_out [12] = -vector3_dot (r_axis, cam->position);
    matrix_out [13] = -vector3_dot (u_axis, cam->position);
    matrix_out [14] = -vector3_dot (f_axis, cam->position);
    matrix_out [15] = 1.0;
} //Movement
void camera_move_w (camera *cam, float delta_time) {
    vector3 velocity_camera = vector3_scaling (cam->front, cam->speed_movement_camera * delta_time);
    cam->position = vector3_addition (cam->position, velocity_camera);
} void camera_move_s (camera *cam, float delta_time) {
    vector3 velocity_camera = vector3_scaling (cam->front, cam->speed_movement_camera * delta_time);
    cam->position = vector3_subtraction (cam->position, velocity_camera);
} //Strafe uses the Right/Side Vector
void camera_move_a (camera *cam, float delta_time) {
    vector3 velocity_camera = vector3_scaling (cam->side, cam->speed_movement_camera * delta_time);
    cam->position = vector3_subtraction (cam->position, velocity_camera);
} void camera_move_d (camera *cam, float delta_time) {
    vector3 velocity_camera = vector3_scaling (cam->side, cam->speed_movement_camera * delta_time);
    cam->position = vector3_addition (cam->position, velocity_camera);
}
