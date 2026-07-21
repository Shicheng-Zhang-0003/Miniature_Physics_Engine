#ifndef camera_h
#define camera_h
#include "../core/math3D.h"

typedef struct {
    vector3 position;
    vector3 forward_vector;
    vector3 vertical_vector;
    vector3 side_vector;
    float yaw;
    float pitch;
    float movement_speed;
    float mouse_sensitivity;
    float vertical_velocity;
    vector3 horizontal_velocity;
} camera;

void initialize_camera (camera *camera_object, vector3 starting_position);
void camera_update_vectors (camera *camera_object);
void camera_move_forward (camera *camera_object, float delta_time);
void camera_move_backward (camera *camera_object, float delta_time);
void camera_move_left (camera *camera_object, float delta_time);
void camera_move_right (camera *camera_object, float delta_time);
#endif
