#ifndef camera_h
#define camera_h
#include "../core/math3d.h"

typedef struct {
    vector3 position;
    vector3 forward_vector;
    vector3 vertical_vector;
    vector3 side_vector;
    float yaw;
    float pitch;
    /* FIX-AUDIT-DESPOT: movement_speed semantics (was undocumented dual-use).
     * Unit is m/s (= camera.move_speed, default 25). Debug fly mode
     * integrates it DIRECTLY as velocity (pos += dir * speed * dt in
     * simulation_camera_tick). Game grounded mode instead feeds it as an
     * ACCELERATION gain (camera_move_*: horizontal_velocity += dir * speed *
     * 8.0 * dt, with the 8.0 game-mode accel gain), bled each tick by
     * camera.horizontal_friction towards a terminal glide. Both paths share
     * this one field; do not retune one mode without the other. */
    float movement_speed;
    float mouse_sensitivity;
    float vertical_velocity;
    vector3 horizontal_velocity;
} camera;

void initialize_camera(camera *camera_object, vector3 starting_position);
void camera_update_vectors(camera *camera_object);
void camera_move_forward(camera *camera_object, float delta_time);
void camera_move_backward(camera *camera_object, float delta_time);
void camera_move_left(camera *camera_object, float delta_time);
void camera_move_right(camera *camera_object, float delta_time);
#endif
