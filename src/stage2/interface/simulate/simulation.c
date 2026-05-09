#include "../../master_header_2.h"
#include "../../../stage3/master_header_3.h"
#include "../../../stage1/master_header.h"
#include "../../../stage4/master_header_4.h"
#include "../../../stage5/master_header_5.h"
#include <gtk/gtk.h>
#include <stdbool.h>
//World Status right now
frame_timer main_timer;
rigidbody *obj_per_scene = NULL;
int object_count = 0;
int object_capacity = 0;
gboolean physics_step_increment (gpointer user_data_pointer) {
    frame_timer_update (&main_timer);
    float frame_delta_time = main_timer.delta_time;
    if (main_inputs.w_key_pressed) {camera_move_forward (&main_camera_fov, frame_delta_time);}
    if (main_inputs.a_key_pressed) {camera_move_left (&main_camera_fov, frame_delta_time);}
    if (main_inputs.s_key_pressed) {camera_move_backward (&main_camera_fov, frame_delta_time);}
    if (main_inputs.d_key_pressed) {camera_move_right (&main_camera_fov, frame_delta_time);}
    // Mouse Perspective Steering
    if (main_inputs.is_mouse_locked) {
        float perspective_steering_sensitivity = 0.0035f;
        main_camera_fov.yaw += main_inputs.mouse_delta_x * perspective_steering_sensitivity;
        main_camera_fov.pitch += main_inputs.mouse_delta_y * perspective_steering_sensitivity;
        // Reset deltas
        main_inputs.mouse_delta_x = 0.0;
        main_inputs.mouse_delta_y = 0.0;
    } if (main_camera_fov.pitch > 89.0f) {main_camera_fov.pitch = 89.0f;}
    if (main_camera_fov.pitch < -89.0f) {main_camera_fov.pitch = -89.0f;}
    camera_update_vectors (&main_camera_fov);
    if (!main_inputs.is_debug_mode_active) {
        if (main_camera_fov.position.x < -250.0f) {main_camera_fov.position.x = -250.0f;}
        if (main_camera_fov.position.x > 250.0f) {main_camera_fov.position.x = 250.0f;}
        if (main_camera_fov.position.z < -250.0f) {main_camera_fov.position.z = -250.0f;}
        if (main_camera_fov.position.z > 250.0f) {main_camera_fov.position.z = 250.0f;}
    } if (main_inputs.escape_key_pressed) {
        if (main_inputs.is_mouse_locked) {
            mouse_lock_disable (gtk_widget_get_toplevel (GTK_WIDGET (user_data_pointer)));
            main_inputs.is_mouse_locked = false;
        } main_inputs.escape_key_pressed = false;
    } if (main_inputs.right_mouse_button_clicked) {
        selector_ray_tracing ();
        main_inputs.right_mouse_button_clicked = false;
    } if (main_inputs.space_key_pressed) {
        if (selected_object >= 0) {selector_apply_force_impulse (50.0);}
        main_inputs.space_key_pressed = false;
    } //Spawn Logic (Shift Press/Hold)
    static float shift_hold_timer = 0.0;
    static bool shift_previously_held = false;
    if (main_inputs.shift_key_pressed) {
        if (! shift_previously_held) {
            // Initial simple spawn on press
            spawner_launch_sphere (0.5, 1.0, 20.0);
            shift_hold_timer = 0.0;
        } else {
            // Key is being held
            shift_hold_timer += frame_delta_time;
            if (shift_hold_timer > 2.0) {
                // "Floor it" - continuous spawning
                spawner_launch_sphere (0.5, 1.0, 20.0);
            }
        } shift_previously_held = true;
    } else {
        shift_hold_timer = 0.0;
        shift_previously_held = false;
    }

    if (main_inputs.save_key_pressed) {
        save_scene ("scene.dat");
        main_inputs.save_key_pressed = false;
    } if (main_inputs.load_key_pressed) {
        scene_loading ("scene.dat");
        main_inputs.load_key_pressed = false;
    } if (main_inputs.left_mouse_button_clicked) {
        if (selected_object >= 0) {
            remove_joints_from_object (selected_object);
            // Shift array (fill gap)
            for (int object_shift_index = selected_object; object_shift_index < (object_count - 1); object_shift_index++) {
                obj_per_scene [object_shift_index] = obj_per_scene [object_shift_index + 1];
            } object_count -= 1;
            // Update joint indices
            for (int joint_index = 0; joint_index < maximum_joint_count; joint_index++) {
                if (! (joint_pool [joint_index].is_active)) {continue;}
                if (joint_pool [joint_index].object_index_a > selected_object) {joint_pool [joint_index].object_index_a -= 1;}
                if (joint_pool [joint_index].object_index_b > selected_object) {joint_pool [joint_index].object_index_b -= 1;}
            } clear_selection ();
        } main_inputs.left_mouse_button_clicked = false;
    } //Apply Force (All Joints)
    apply_force_all_joints ();
    for (int object_iterator_index = 0; object_iterator_index < object_count; object_iterator_index++) {
        vector3 constant_gravity_acceleration = {0, -9.81, 0};
        //Normal Force (Floor check y=0)
        if (obj_per_scene [object_iterator_index].position.y <= (obj_per_scene [object_iterator_index].radius + 0.01)) {
            force_applicant_gravity_normal (&obj_per_scene [object_iterator_index], constant_gravity_acceleration, (vector3) {0.0, 1.0, 0.0});
            //Apply Floor Friction (Induce Rotation)
            force_applicant_friction_rolling (&obj_per_scene [object_iterator_index], (vector3) {0.0, 1.0, 0.0}, 0.3, 0.2);
        } else {
            // Gravity (Freefall)
            rb_apply_forces_perfect (&obj_per_scene [object_iterator_index], vector3_scaling (constant_gravity_acceleration, obj_per_scene [object_iterator_index].mass));
        }
    } //Resolve Collisions
    broadphase_pair broadphase_collision_pairs [256];
    int detected_collision_count = broadphase_generate_pairing (broadphase_collision_pairs, 256);
    for (int collision_index = 0; collision_index < detected_collision_count; collision_index++) {
        collision_data narrowphase_collision;
        if (collision_dual_sphere (&obj_per_scene [broadphase_collision_pairs [collision_index].object_index_a], &obj_per_scene [broadphase_collision_pairs [collision_index].object_index_b], &narrowphase_collision)) {collision_resolve (&narrowphase_collision);}
    } //Integrate and Redraw
    for (int object_iterator_index = 0; object_iterator_index < object_count; object_iterator_index++) {rb_integrate (&obj_per_scene [object_iterator_index], frame_delta_time);}
    if (!main_inputs.is_debug_mode_active) {
        vector3 boundary_minimum_bounds = {-250.0, 0.0, -250.0};
        vector3 boundary_maximum_bounds = {250.0, 500.0, 250.0};
        for (int object_iterator_index = 0; object_iterator_index < object_count; object_iterator_index++) {
            boundary_apply_box (&obj_per_scene [object_iterator_index], boundary_minimum_bounds, boundary_maximum_bounds);
        }
    } else {
        for (int object_iterator_index = 0; object_iterator_index < object_count; object_iterator_index++) {
            boundary_apply_floor (&obj_per_scene [object_iterator_index], 0.0);
        }
    } gtk_widget_queue_draw (GTK_WIDGET (user_data_pointer));
    overlay_update ();
    return TRUE;
}
