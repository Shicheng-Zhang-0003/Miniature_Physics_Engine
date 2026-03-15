#ifndef camera_h
#define camera_h
#include "../master_header_2.h"
#include "../../stage1/master_header.h"
typedef struct {
    vector3 position; //Where Camera is in 3D field
    vector3 front; //Vector that the camera is currently facing
    vector3 vertical; //Which vector defines moving up
    vector3 side; //Axis from Horizontal Planar Entry
    //Euler Angles for rotating motion
    float yaw; //Left, Right
    float pitch; //Up, Down (Degrees)
    //Base Settings
    float speed_movement_camera; //How Fast the Camera POV moves
    float sensitivity_mouse; //How sensitive the mouse is
} camera;
//Init Camera (starting values)
void initalise_camera (camera *cam, vector3 start_position);
//4 ^ 4 view matrix --> OpenGL shaders
void camera_view_matrix (camera *cam, float *matrix_out);
//Input and Motion Functions
void camera_update_vector_input (camera *cam);
void camera_move_w (camera *cam, float delta_time);
void camera_move_s (camera *cam, float delta_time);
void camera_move_a (camera *cam, float delta_time);
void camera_move_d (camera * cam, float delta_time);

#endif
