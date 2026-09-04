/* MFS_INCREMENT_SPLIT_1: Camera and character movement tick.
* Extracted from physics_step_increment in simulation.c.
* Owns: game-mode grounded movement, debug-mode fly, IJKL steer,
*        pitch clamp, character gravity/jump/boundary.
*/
#include "../mpe_engine.h"
#include <math.h>

void simulation_camera_tick(float frame_delta_time) {
    /* Game Mode: grounded WASD */
    if (!main_inputs.is_debug_mode_active) {
        if (main_inputs.w_key_pressed) {
            camera_move_forward(&main_camera_fov, frame_delta_time);
        }
        if (main_inputs.a_key_pressed) {
            camera_move_left(&main_camera_fov, frame_delta_time);
        }
        if (main_inputs.s_key_pressed) {
            camera_move_backward(&main_camera_fov, frame_delta_time);
        }
        if (main_inputs.d_key_pressed) {
            camera_move_right(&main_camera_fov, frame_delta_time);
        }
    }

    /* Perspective steering (mouse look) */
    float perspective_steering_sensitivity = g_cfg.camera.steer_sensitivity;
    if (main_inputs.is_mouse_locked) {
        main_camera_fov.yaw += main_inputs.mouse_delta_x * perspective_steering_sensitivity;
        main_camera_fov.pitch += main_inputs.mouse_delta_y * perspective_steering_sensitivity;
        main_inputs.mouse_delta_x = 0.0f;
        main_inputs.mouse_delta_y = 0.0f;
    }

    /* IJKL emulation (Debug Mode) */
    /* MFS_127_CAMERA_FLOAT_FIX: Reset vertical velocity when entering debug mode
     * to prevent camera from floating upward due to residual game-mode velocity. */
    if (main_inputs.is_debug_mode_active) {
        /* Reset vertical velocity if not actively controlled */
        if (!main_inputs.space_key_pressed && !main_inputs.shift_key_pressed) {
            main_camera_fov.vertical_velocity = 0.0f;
        }
        float debug_speed = main_camera_fov.movement_speed * frame_delta_time;
        if (main_inputs.w_key_pressed) {
            main_camera_fov.position = vector3_addition(
                main_camera_fov.position,
                vector3_scaling(main_camera_fov.forward_vector, debug_speed));
        }
        if (main_inputs.s_key_pressed) {
            main_camera_fov.position = vector3_subtraction(
                main_camera_fov.position,
                vector3_scaling(main_camera_fov.forward_vector, debug_speed));
        }
        if (main_inputs.a_key_pressed) {
            main_camera_fov.position = vector3_subtraction(
                main_camera_fov.position,
                vector3_scaling(main_camera_fov.side_vector, debug_speed));
        }
        if (main_inputs.d_key_pressed) {
            main_camera_fov.position = vector3_addition(
                main_camera_fov.position,
                vector3_scaling(main_camera_fov.side_vector, debug_speed));
        }
        if (main_inputs.space_key_pressed) {
            main_camera_fov.position.y += debug_speed;
        }
        if (main_inputs.shift_key_pressed) {
            main_camera_fov.position.y -= debug_speed;
        }
        float ijkl_speed = g_cfg.camera.ijkl_speed * frame_delta_time;
        if (main_inputs.i_key_pressed) { main_camera_fov.pitch += ijkl_speed; }
        if (main_inputs.k_key_pressed) { main_camera_fov.pitch -= ijkl_speed; }
        if (main_inputs.j_key_pressed) { main_camera_fov.yaw -= ijkl_speed; }
        if (main_inputs.l_key_pressed) { main_camera_fov.yaw += ijkl_speed; }
    }

    /* Pitch clamp */
    if (main_camera_fov.pitch > 89.0f) { main_camera_fov.pitch = 89.0f; }
    if (main_camera_fov.pitch < -89.0f) { main_camera_fov.pitch = -89.0f; }
    camera_update_vectors(&main_camera_fov);

    /* Character logic (Game Mode only) */
    if (!main_inputs.is_debug_mode_active) {
        float horizontal_friction = g_cfg.camera.horizontal_friction;
        main_camera_fov.horizontal_velocity.x -=
            main_camera_fov.horizontal_velocity.x * horizontal_friction * frame_delta_time;
        main_camera_fov.horizontal_velocity.z -=
            main_camera_fov.horizontal_velocity.z * horizontal_friction * frame_delta_time;
        main_camera_fov.position.x += main_camera_fov.horizontal_velocity.x * frame_delta_time;
        main_camera_fov.position.z += main_camera_fov.horizontal_velocity.z * frame_delta_time;
        main_camera_fov.vertical_velocity += g_cfg.world.gravity * frame_delta_time;
        main_camera_fov.position.y += main_camera_fov.vertical_velocity * frame_delta_time;
        if (main_camera_fov.position.y <= 2.0f) {
            main_camera_fov.position.y = 2.0f;
            main_camera_fov.vertical_velocity = 0.0f;
            if (main_inputs.space_key_pressed) {
                float jump_velocity = sqrtf(2.0f * fabsf(g_cfg.world.gravity) * g_cfg.camera.jump_height);
                main_camera_fov.vertical_velocity = jump_velocity;
                main_inputs.space_key_pressed = false;
            }
        }
        if (main_camera_fov.position.x < -250.0f) { main_camera_fov.position.x = -250.0f; }
        if (main_camera_fov.position.x > 250.0f) { main_camera_fov.position.x = 250.0f; }
        if (main_camera_fov.position.z < -250.0f) { main_camera_fov.position.z = -250.0f; }
        if (main_camera_fov.position.z > 250.0f) { main_camera_fov.position.z = 250.0f; }
    }
}
