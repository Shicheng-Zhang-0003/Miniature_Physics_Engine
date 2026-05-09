#ifndef camera_h
#define camera_h
#include "../../stage1/master_header.h"
typedef struct {
    vector3 position; //Where Camera is in 3D field
    vector3 forward_vector; //Vector that the camera is currently facing
    vector3 vertical_vector; //Which vector defines moving up
    vector3 side_vector; //Axis from Horizontal Planar Entry
    //Euler Angles for rotating motion
    float yaw; //Left, Right
    float pitch; //Up, Down (Degrees)
    //Base Settings
    float movement_speed; //How Fast the Camera POV moves
    float mouse_sensitivity; //How sensitive the mouse is
} camera;
//Init Camera (starting values)
void initalise_camera (camera *camera_object, vector3 starting_position);
//4 ^ 4 view matrix --> OpenGL shaders
//void camera_view_matrix (camera *camera_object, float *matrix_output);
//Input and Motion Functions
void camera_update_vectors (camera *camera_object);
void camera_move_forward (camera *camera_object, float delta_time);
void camera_move_backward (camera *camera_object, float delta_time);
void camera_move_left (camera *camera_object, float delta_time);
void camera_move_right (camera *camera_object, float delta_time);

#endif
