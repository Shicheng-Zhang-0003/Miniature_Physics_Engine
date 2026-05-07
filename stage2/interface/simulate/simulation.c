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
gboolean physics_step_increment (gpointer user_data_stored_pointer) {
    static int previously_selected_object_index = -1;
    frame_timer_update (&main_timer);
    float current_frame_delta_time = main_timer.delta_time;
    if (main_inputs.w_key) {camera_move_w (&main_camera_fov, current_frame_delta_time);}
    if (main_inputs.a_key) {camera_move_a (&main_camera_fov, current_frame_delta_time);}
    if (main_inputs.s_key) {camera_move_s (&main_camera_fov, current_frame_delta_time);}
    if (main_inputs.d_key) {camera_move_d (&main_camera_fov, current_frame_delta_time);}
    //IJKL Viewframe
    float look_speed = 60.0f; // degrees per second, tune to taste
    if (main_inputs.cam_up) {main_camera_fov.pitch += look_speed * current_frame_delta_time;}
    if (main_inputs.cam_down) {main_camera_fov.pitch -= look_speed * current_frame_delta_time;}
    if (main_inputs.cam_left) {main_camera_fov.yaw -= look_speed * current_frame_delta_time;}
    if (main_inputs.cam_right) {main_camera_fov.yaw += look_speed * current_frame_delta_time;}
    if (main_camera_fov.pitch > 89.0f) {main_camera_fov.pitch = 89.0f;}
    if (main_camera_fov.pitch < -89.0f) {main_camera_fov.pitch = -89.0f;}
    if ((main_inputs.cam_up) || (main_inputs.cam_down) || (main_inputs.cam_left) || (main_inputs.cam_right)) {
        camera_update_vector_input (&main_camera_fov);
    } if (main_inputs.e_key) {
        selector_ray_tracing ();
        main_inputs.e_key = false;
    } if (main_inputs.f_key) {
        selector_apply_force_impulse (50.0);
        main_inputs.f_key = false;
    } if (main_inputs.space_key) {
        spawner_launch_sphere (0.5, 1.0, 20.0);
        main_inputs.space_key = false;
    } if (main_inputs.r_key) {
        save_scene ("scene.dat");
        main_inputs.r_key = false;
    } if (main_inputs.l_key) {
        scene_loading ("scene.dat");
        main_inputs.l_key = false;
    } if (main_inputs.j_key) {
        if ((selected_object >= 0) && (previously_selected_object_index >= 0) && (selected_object != previously_selected_object_index)) {
            // Rest length is current distance between the two objects
            vector3 position_difference_vector = vector3_subtraction (obj_per_scene [selected_object].position, obj_per_scene [previously_selected_object_index].position);
            float spring_resting_length_value = vector3_length (position_difference_vector);
            add_joint (previously_selected_object_index, selected_object, spring_resting_length_value, 10.0, 0.5);
        } previously_selected_object_index = selected_object;
        main_inputs.j_key = false;
    } if (main_inputs.x_key) {
        if (selected_object >= 0) {
            remove_joints_from_object (selected_object);
            // Shift array (fill gap)
            for (int object_shift_iterator_index = selected_object; object_shift_iterator_index < object_count - 1; object_shift_iterator_index++) {
                obj_per_scene [object_shift_iterator_index] = obj_per_scene [object_shift_iterator_index + 1];
            } object_count -= 1;
            // Update joint indices
            for (int joint_pool_iterator_index = 0; joint_pool_iterator_index < max_joint_count; joint_pool_iterator_index++) {
                if (!joint_pool [joint_pool_iterator_index].activation) {continue;}
                if (joint_pool [joint_pool_iterator_index].index_av > selected_object) {joint_pool [joint_pool_iterator_index].index_av -= 1;}
                if (joint_pool [joint_pool_iterator_index].index_bv > selected_object) {joint_pool [joint_pool_iterator_index].index_bv -= 1;}
            } clear_selection ();
            previously_selected_object_index = -1;
        } main_inputs.x_key = false;
    } //Apply Force (All Joints)
    apply_force_all_joints ();
    for (int force_application_iterator_index = 0; force_application_iterator_index < object_count; force_application_iterator_index++) {
        vector3 constant_gravity_acceleration_vector = {0, -9.81, 0};
        //Normal Force (Floor check y=0)
        if (obj_per_scene [force_application_iterator_index].position.y <= obj_per_scene [force_application_iterator_index].radius + 0.01) {
            force_applicant_gravity_normal (&obj_per_scene [force_application_iterator_index], constant_gravity_acceleration_vector, (vector3) {0.0, 1.0, 0.0});
        } else {
            // Gravity (Freefall)
            rb_apply_forces_perfect (&obj_per_scene [force_application_iterator_index], vector3_scaling (constant_gravity_acceleration_vector, obj_per_scene [force_application_iterator_index].mass));
        }
    } //Resolve Collisions
    broadphase_pair collision_pairs_detected_array [256];
    int detected_collision_pair_count = broadphase_generate_pairing (collision_pairs_detected_array, 256);
    for (int collision_resolution_iterator_index = 0; collision_resolution_iterator_index < detected_collision_pair_count; collision_resolution_iterator_index++) {
        collision_data narrowphase_collision_information;
        if (collision_dual_sphere(&obj_per_scene [collision_pairs_detected_array [collision_resolution_iterator_index].a], &obj_per_scene [collision_pairs_detected_array [collision_resolution_iterator_index].b], &narrowphase_collision_information)) {collision_resolve (&narrowphase_collision_information);}
    } //Integrate and Redraw
    for (int physics_integration_iterator_index = 0; physics_integration_iterator_index < object_count; physics_integration_iterator_index++) {rb_integrate (&obj_per_scene [physics_integration_iterator_index], current_frame_delta_time);}
    for (int boundary_check_iterator_index = 0; boundary_check_iterator_index < object_count; boundary_check_iterator_index++) {boundary_apply_floor (&obj_per_scene [boundary_check_iterator_index], 0.0);}
    gtk_widget_queue_draw (GTK_WIDGET (user_data_stored_pointer));
    overlay_update ();
    return TRUE;
}
