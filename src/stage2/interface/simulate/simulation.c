#include "../../master_header_2.h"
#include "../../../stage3/master_header_3.h"
#include "../../../stage1/master_header.h"
#include "../../../stage4/master_header_4.h"
#include "../../../stage5/master_header_5.h"
#include "../../../stage5/constraints/spring_joint.h"
#include <gtk/gtk.h>
#include <stdbool.h>
#include <stdlib.h>
#include <sys/stat.h>
//World Status right now
frame_timer main_timer;
rigidbody *obj_per_scene = NULL;
int object_count = 0;
int object_capacity = 0;
// World Physics Globals
float world_gravity_y = -9.81f;
float world_drag_coefficient = 0.99f; // Used in pow (drag, delta)
float world_surface_friction_static = 0.2f;
float world_surface_friction_kinetic = 0.1f;
float variable_change_rate = 0.2f;
float character_weight = 70.0f;
float jump_height = 1.0f;
gboolean physics_step_increment (gpointer user_data_pointer) {
    if (main_inputs.is_debug_mode_active) {
        if (main_inputs.left_arrow_pressed)  {variable_change_rate -= 0.01f; main_inputs.left_arrow_pressed = false;}
        if (main_inputs.right_arrow_pressed) {variable_change_rate += 0.01f; main_inputs.right_arrow_pressed = false;}
    } else {
        if (main_inputs.left_arrow_pressed)  {variable_change_rate -= 0.2f; main_inputs.left_arrow_pressed = false;}
        if (main_inputs.right_arrow_pressed) {variable_change_rate += 0.2f; main_inputs.right_arrow_pressed = false;}
    } float adjustment_increment = variable_change_rate;
    static int status_dir_checked = 0;
    if (!status_dir_checked) {mkdir ("status", 0755); status_dir_checked = 1;}
    frame_timer_update (&main_timer);
    float frame_delta_time = main_timer.delta_time;
    // Camera Movements
    if (main_inputs.w_key_pressed) {camera_move_forward (&main_camera_fov, frame_delta_time);}
    if (main_inputs.a_key_pressed) {camera_move_left (&main_camera_fov, frame_delta_time);}
    if (main_inputs.s_key_pressed) {camera_move_backward (&main_camera_fov, frame_delta_time);}
    if (main_inputs.d_key_pressed) {camera_move_right (&main_camera_fov, frame_delta_time);}
    // Perspective Steering
    float perspective_steering_sensitivity = 0.12f;
    if (main_inputs.is_mouse_locked) {
        main_camera_fov.yaw += main_inputs.mouse_delta_x * perspective_steering_sensitivity;
        main_camera_fov.pitch += main_inputs.mouse_delta_y * perspective_steering_sensitivity;
        main_inputs.mouse_delta_x = 0.0f;
        main_inputs.mouse_delta_y = 0.0f;
    } // IJKL Emulation (Debug Mode)
    if (main_inputs.is_debug_mode_active) {
        float ijkl_speed = 35.0f * frame_delta_time;
        if (main_inputs.i_key_pressed) {main_camera_fov.pitch += ijkl_speed;}
        if (main_inputs.k_key_pressed) {main_camera_fov.pitch -= ijkl_speed;}
        if (main_inputs.j_key_pressed) {main_camera_fov.yaw -= ijkl_speed;}
        if (main_inputs.l_key_pressed) {main_camera_fov.yaw += ijkl_speed;}
    } if (main_camera_fov.pitch > 89.0f) {main_camera_fov.pitch = 89.0f;}
    if (main_camera_fov.pitch < -89.0f) {main_camera_fov.pitch = -89.0f;}
    camera_update_vectors (&main_camera_fov);
    // Character Logic
    if (!main_inputs.is_debug_mode_active) {
        main_camera_fov.vertical_velocity += world_gravity_y * frame_delta_time;
        main_camera_fov.position.y += main_camera_fov.vertical_velocity * frame_delta_time;
        if (main_camera_fov.position.y <= 2.0f) {
            main_camera_fov.position.y = 2.0f;
            main_camera_fov.vertical_velocity = 0.0f;
            if (main_inputs.space_key_pressed) {
                float jump_velocity = sqrtf (2.0f * fabsf (world_gravity_y) * jump_height);
                main_camera_fov.vertical_velocity = jump_velocity;
                main_inputs.space_key_pressed = false;
            }
        } if (main_camera_fov.position.x < -250.0f) {main_camera_fov.position.x = -250.0f;}
        if (main_camera_fov.position.x > 250.0f) {main_camera_fov.position.x = 250.0f;}
        if (main_camera_fov.position.z < -250.0f) {main_camera_fov.position.z = -250.0f;}
        if (main_camera_fov.position.z > 250.0f) {main_camera_fov.position.z = 250.0f;}
    } //Mouse, Escape, E, F key bindings and actions
    if (main_inputs.escape_key_pressed) {
        if (main_inputs.is_mouse_locked) {
            mouse_lock_disable (gtk_widget_get_toplevel (GTK_WIDGET (user_data_pointer)));
            main_inputs.is_mouse_locked = false;
        } main_inputs.escape_key_pressed = false;
    } if (main_inputs.right_mouse_button_clicked) {
        selector_ray_tracing ();
        main_inputs.right_mouse_button_clicked = false;
    } if (main_inputs.middle_mouse_button_clicked) {
        //Scroll wheel click removes object
        if (selected_object >= 0) {
            // Shift objects down to fill hole
            for (int object_index = selected_object; object_index < object_count - 1; object_index++) {obj_per_scene [object_index] = obj_per_scene [object_index + 1];}
            object_count -= 1;
            selected_object = -1;
        } main_inputs.middle_mouse_button_clicked = false;
    } if (main_inputs.e_key_pressed) {
        if (selected_object >= 0) {
            if (main_inputs.object_menu_level > 0) {main_inputs.object_menu_level = 0;}
            else {main_inputs.object_menu_level = 1;}
        } main_inputs.e_key_pressed = false;
    } if (main_inputs.f_key_pressed) {
        if (selected_object >= 0) {selector_apply_force_impulse (250.0f);} //Increased as cube friction is far higher
        main_inputs.f_key_pressed = false;
    } //Holding down shift, spawn gun
    static float shift_hold_timer = 0.0f;
    static float shift_spawn_interval_timer = 0.0f;
    static bool shift_previously_held = false;
    if (main_inputs.shift_key_pressed) {
        if (!shift_previously_held) {
            if (main_inputs.current_spawn_type == 0) {spawner_launch_sphere (spawn_radius, spawn_mass, spawn_speed);}
            else {
                vector3 cube_spawn_position = vector3_addition (main_camera_fov.position, vector3_scaling (main_camera_fov.forward_vector, spawn_cube_extent + 1.0f));
                spawner_launch_cube (cube_spawn_position, (vector3) {spawn_cube_extent, spawn_cube_extent, spawn_cube_extent}, spawn_cube_mass);
            } shift_hold_timer = 0.0f;
            shift_spawn_interval_timer = 0.0f;
        } else {
            shift_hold_timer += frame_delta_time;
            if (shift_hold_timer > 0.3f) {
                shift_spawn_interval_timer += frame_delta_time;
                if (shift_spawn_interval_timer >= 0.02f) {
                    if (main_inputs.current_spawn_type == 0) {spawner_launch_sphere (spawn_radius, spawn_mass, spawn_speed);}
                    else {
                        vector3 cube_spawn_position = vector3_addition (main_camera_fov.position, vector3_scaling (main_camera_fov.forward_vector, spawn_cube_extent + 1.0f));
                        spawner_launch_cube (cube_spawn_position, (vector3) {spawn_cube_extent, spawn_cube_extent, spawn_cube_extent}, spawn_cube_mass);
                    } shift_spawn_interval_timer = 0.0f;
                }
            }
        } shift_previously_held = true;
    } else {
        shift_hold_timer = 0.0f;
        shift_previously_held = false;
    } //Scene Saving, 9 Key bindings
    if (main_inputs.menu_1_pressed) {save_scene ("status/scene.dat"); main_inputs.menu_1_pressed = false; main_inputs.is_menu_open = false;}
    if (main_inputs.menu_2_pressed) {scene_loading ("status/scene.dat"); main_inputs.menu_2_pressed = false; main_inputs.is_menu_open = false;}
    if (main_inputs.menu_3_pressed) {main_inputs.menu_3_pressed = false; gtk_main_quit ();}
    // Spawner Menu Logic
    if (main_inputs.spawner_menu_level == 3) {
        if (main_inputs.up_arrow_pressed)   {spawn_mass += adjustment_increment; main_inputs.up_arrow_pressed = false;}
        if (main_inputs.down_arrow_pressed) {spawn_mass -= adjustment_increment; if (spawn_mass < 0.01f) {spawn_mass = 0.01f;} main_inputs.down_arrow_pressed = false;}
        if (main_inputs.enter_key_pressed)  {main_inputs.spawner_menu_level = 0; main_inputs.enter_key_pressed = false;}
    } if (main_inputs.spawner_menu_level == 4) {
        if (main_inputs.up_arrow_pressed)   {spawn_radius += adjustment_increment; main_inputs.up_arrow_pressed = false;}
        if (main_inputs.down_arrow_pressed) {spawn_radius -= adjustment_increment; if (spawn_radius < 0.01f) {spawn_radius = 0.01f;} main_inputs.down_arrow_pressed = false;}
        if (main_inputs.enter_key_pressed)  {main_inputs.spawner_menu_level = 0; main_inputs.enter_key_pressed = false;}
    } if (main_inputs.spawner_menu_level == 6) {
        if (main_inputs.up_arrow_pressed)   {spawn_cube_mass += adjustment_increment; main_inputs.up_arrow_pressed = false;}
        if (main_inputs.down_arrow_pressed) {spawn_cube_mass -= adjustment_increment; if (spawn_cube_mass < 0.01f) {spawn_cube_mass = 0.01f;} main_inputs.down_arrow_pressed = false;}
        if (main_inputs.enter_key_pressed)  {main_inputs.spawner_menu_level = 0; main_inputs.enter_key_pressed = false;}
    } if (main_inputs.spawner_menu_level == 7) {
        if (main_inputs.up_arrow_pressed)   {spawn_cube_extent += adjustment_increment; main_inputs.up_arrow_pressed = false;}
        if (main_inputs.down_arrow_pressed) {spawn_cube_extent -= adjustment_increment; if (spawn_cube_extent < 0.01f) {spawn_cube_extent = 0.01f;} main_inputs.down_arrow_pressed = false;}
        if (main_inputs.enter_key_pressed)  {main_inputs.spawner_menu_level = 0; main_inputs.enter_key_pressed = false;}
    } if (main_inputs.spawner_menu_level == 8) {
        if ((main_inputs.up_arrow_pressed) || (main_inputs.down_arrow_pressed)) {
            if (main_inputs.current_spawn_type == 0) {main_inputs.current_spawn_type = 1;}
            else {main_inputs.current_spawn_type = 0;}
            main_inputs.up_arrow_pressed = false;
            main_inputs.down_arrow_pressed = false;
        } if (main_inputs.enter_key_pressed) {main_inputs.spawner_menu_level = 0; main_inputs.enter_key_pressed = false;}
    } // User Mechanics Menu Logic
    if (main_inputs.velocity_menu_level == 3) {
        if (main_inputs.up_arrow_pressed)   {spawn_speed += adjustment_increment; main_inputs.up_arrow_pressed = false;}
        if (main_inputs.down_arrow_pressed) {spawn_speed -= adjustment_increment; if (spawn_speed < 0.0f) {spawn_speed = 0.0f;} main_inputs.down_arrow_pressed = false;}
        if (main_inputs.enter_key_pressed)  {main_inputs.velocity_menu_level = 0; main_inputs.enter_key_pressed = false;}
    } if (main_inputs.velocity_menu_level == 4) {
        if (main_inputs.up_arrow_pressed)   {world_surface_friction_kinetic += adjustment_increment; world_surface_friction_static = world_surface_friction_kinetic + 0.1f; main_inputs.up_arrow_pressed = false;}
        if (main_inputs.down_arrow_pressed) {world_surface_friction_kinetic -= adjustment_increment; if (world_surface_friction_kinetic < 0.0f) {world_surface_friction_kinetic = 0.0f;} world_surface_friction_static = world_surface_friction_kinetic + 0.1f; main_inputs.down_arrow_pressed = false;}
        if (main_inputs.enter_key_pressed)  {main_inputs.velocity_menu_level = 0; main_inputs.enter_key_pressed = false;}
    } if (main_inputs.velocity_menu_level == 11) {
        if (main_inputs.up_arrow_pressed)   {main_camera_fov.movement_speed += adjustment_increment; main_inputs.up_arrow_pressed = false;}
        if (main_inputs.down_arrow_pressed) {main_camera_fov.movement_speed -= adjustment_increment; if (main_camera_fov.movement_speed < 0.01f) {main_camera_fov.movement_speed = 0.01f;} main_inputs.down_arrow_pressed = false;}
        if (main_inputs.enter_key_pressed)  {main_inputs.velocity_menu_level = 0; main_inputs.enter_key_pressed = false;}
    } if (main_inputs.velocity_menu_level == 12) { // Jump Height
        if (main_inputs.up_arrow_pressed)   {jump_height += adjustment_increment; main_inputs.up_arrow_pressed = false;}
        if (main_inputs.down_arrow_pressed) {jump_height -= adjustment_increment; if (jump_height < 0.1f) {jump_height = 0.1f;} main_inputs.down_arrow_pressed = false;}
        if (main_inputs.enter_key_pressed)  {main_inputs.velocity_menu_level = 0; main_inputs.enter_key_pressed = false;}
    } if (main_inputs.velocity_menu_level == 21) {
        if (main_inputs.up_arrow_pressed)   {world_gravity_y += adjustment_increment; main_inputs.up_arrow_pressed = false;}
        if (main_inputs.down_arrow_pressed) {world_gravity_y -= adjustment_increment; main_inputs.down_arrow_pressed = false;}
        if (main_inputs.enter_key_pressed)  {main_inputs.velocity_menu_level = 0; main_inputs.enter_key_pressed = false;}
    } if (main_inputs.velocity_menu_level == 22) {
        if (main_inputs.up_arrow_pressed)   {world_drag_coefficient += adjustment_increment * 0.01f; if (world_drag_coefficient > 1.0f) {world_drag_coefficient = 1.0f;} main_inputs.up_arrow_pressed = false;}
        if (main_inputs.down_arrow_pressed) {world_drag_coefficient -= adjustment_increment * 0.01f; if (world_drag_coefficient < 0.1f) {world_drag_coefficient = 0.1f;} main_inputs.down_arrow_pressed = false;}
        if (main_inputs.enter_key_pressed)  {main_inputs.velocity_menu_level = 0; main_inputs.enter_key_pressed = false;}
    } if (main_inputs.velocity_menu_level == 23) {
        if (main_inputs.up_arrow_pressed)   {world_surface_friction_kinetic += adjustment_increment; world_surface_friction_static = world_surface_friction_kinetic + 0.1f; main_inputs.up_arrow_pressed = false;}
        if (main_inputs.down_arrow_pressed) {world_surface_friction_kinetic -= adjustment_increment; if (world_surface_friction_kinetic < 0.0f) {world_surface_friction_kinetic = 0.0f;} world_surface_friction_static = world_surface_friction_kinetic + 0.1f; main_inputs.down_arrow_pressed = false;}
        if (main_inputs.enter_key_pressed)  {main_inputs.velocity_menu_level = 0; main_inputs.enter_key_pressed = false;}
    } // Selected Object Menu Logic
    if (main_inputs.object_menu_level == 2) {
        rigidbody *selected_rigid_body = &obj_per_scene [selected_object];
        if (main_inputs.up_arrow_pressed)   {selected_rigid_body -> mass += adjustment_increment; if (selected_rigid_body -> mass > 0) {selected_rigid_body -> inverse_mass = 1.0f / selected_rigid_body -> mass;} rigidbody_update_inertia_sphere (selected_rigid_body); main_inputs.up_arrow_pressed = false;}
        if (main_inputs.down_arrow_pressed) {selected_rigid_body -> mass -= adjustment_increment; if (selected_rigid_body -> mass < 0.01f) {selected_rigid_body -> mass = 0.01f;} selected_rigid_body -> inverse_mass = 1.0f / selected_rigid_body -> mass; rigidbody_update_inertia_sphere (selected_rigid_body); main_inputs.down_arrow_pressed = false;}
        if (main_inputs.enter_key_pressed)  {main_inputs.object_menu_level = 0; main_inputs.enter_key_pressed = false;}
    } if (main_inputs.object_menu_level == 3) {
        rigidbody *selected_rigid_body = &obj_per_scene [selected_object];
        if (main_inputs.up_arrow_pressed)   {selected_rigid_body -> radius += adjustment_increment; rigidbody_update_inertia_sphere (selected_rigid_body); main_inputs.up_arrow_pressed = false;}
        if (main_inputs.down_arrow_pressed) {selected_rigid_body -> radius -= adjustment_increment; if (selected_rigid_body -> radius < 0.01f) {selected_rigid_body -> radius = 0.01f;} rigidbody_update_inertia_sphere (selected_rigid_body); main_inputs.down_arrow_pressed = false;}
        if (main_inputs.enter_key_pressed)  {main_inputs.object_menu_level = 0; main_inputs.enter_key_pressed = false;}
    } if (main_inputs.object_menu_level == 4) {
        rigidbody *selected_rigid_body = &obj_per_scene [selected_object];
        if (main_inputs.up_arrow_pressed)   {selected_rigid_body -> friction_kinetic += adjustment_increment; selected_rigid_body -> friction_static = selected_rigid_body -> friction_kinetic + 0.1f; main_inputs.up_arrow_pressed = false;}
        if (main_inputs.down_arrow_pressed) {selected_rigid_body -> friction_kinetic -= adjustment_increment; if (selected_rigid_body -> friction_kinetic < 0.0f) {selected_rigid_body -> friction_kinetic = 0.0f;} selected_rigid_body -> friction_static = selected_rigid_body -> friction_kinetic + 0.1f; main_inputs.down_arrow_pressed = false;}
        if (main_inputs.enter_key_pressed)  {main_inputs.object_menu_level = 0; main_inputs.enter_key_pressed = false;}
    } if (main_inputs.object_menu_level == 5) {
        rigidbody *selected_rigid_body = &obj_per_scene [selected_object];
        if ((main_inputs.up_arrow_pressed) || (main_inputs.down_arrow_pressed)) {
            selected_rigid_body -> static_state = !selected_rigid_body -> static_state;
            if (selected_rigid_body -> static_state) {selected_rigid_body -> inverse_mass = 0.0f; selected_rigid_body -> velocity = vector3_zero (); selected_rigid_body -> angular_velocity = vector3_zero ();}
            else {if (selected_rigid_body -> mass > 0) {selected_rigid_body -> inverse_mass = 1.0f / selected_rigid_body -> mass;}}
            main_inputs.up_arrow_pressed = false;
            main_inputs.down_arrow_pressed = false;
        } if (main_inputs.enter_key_pressed) {main_inputs.object_menu_level = 0; main_inputs.enter_key_pressed = false;}
    } //Core Physics Substeps, Dampening on Collisions
    const int physics_sub_steps = 12;
    float sub_step_dt = frame_delta_time / (float)physics_sub_steps;
    float linear_damping_factor = powf (world_drag_coefficient, sub_step_dt);
    float angular_damping_factor = powf (world_drag_coefficient * 0.97f, sub_step_dt);
    //Collision Resolution
    static broadphase_pair *persistent_collision_pairs = NULL;
    static int persistent_pairs_capacity = 0;
    int detected_collision_count = 0;
    int maximum_possible_pairs = (object_count * (object_count - 1)) / 2;
    if (maximum_possible_pairs < 1) maximum_possible_pairs = 1;
    if (maximum_possible_pairs > persistent_pairs_capacity) {
        persistent_pairs_capacity = maximum_possible_pairs + 128;
        persistent_collision_pairs = realloc (persistent_collision_pairs, persistent_pairs_capacity * sizeof (broadphase_pair));
    } if (persistent_collision_pairs) {detected_collision_count = broadphase_generate_pairing (persistent_collision_pairs, persistent_pairs_capacity);}
    //Simulation Substep
    for (int sub_iteration = 0; sub_iteration < physics_sub_steps; sub_iteration++) {
        apply_force_all_joints ();
        for (int object_iterator_index = 0; object_iterator_index < object_count; object_iterator_index++) {
            vector3 constant_gravity_acceleration = {0, world_gravity_y, 0};
            rigidbody *rigid_body = &obj_per_scene [object_iterator_index];
            vector3 up_axis = {0, 1, 0};
            float projection = rigid_body -> radius;
            if (rigid_body -> type == object_cube) {
                vector3 *axes = rigid_body -> cached_axes;
                projection = rigid_body -> half_extensions.x * fabsf (vector3_dot (axes [0], up_axis)) + rigid_body -> half_extensions.y * fabsf (vector3_dot (axes [1], up_axis)) + rigid_body -> half_extensions.z * fabsf (vector3_dot (axes [2], up_axis));
            } if (rigid_body -> position.y <= (projection + 0.01f)) {
                if (rigid_body -> type == object_sphere) {
                    force_applicant_gravity_normal (rigid_body, constant_gravity_acceleration, (vector3) {0.0f, 1.0f, 0.0f});
                    force_applicant_friction_rolling (rigid_body, (vector3) {0.0f, 1.0f, 0.0f}, rigid_body -> friction_static, rigid_body -> friction_kinetic, world_gravity_y);
                } else {
                    vector3 *axes = rigid_body -> cached_axes;
                    vector3 contact_offset = {0,0,0};
                    for (int axis_index = 0; axis_index < 3; axis_index++) {
                        float extent;
                        if (axis_index == 0) {extent = rigid_body -> half_extensions.x;}
                        else if (axis_index == 1) {extent = rigid_body -> half_extensions.y;}
                        else {extent = rigid_body -> half_extensions.z;}
                        vector3 offset = vector3_scaling (axes [axis_index], extent);
                        if (vector3_dot (offset, (vector3){0, -1, 0}) > 0) contact_offset = vector3_addition (contact_offset, offset);
                        else contact_offset = vector3_subtraction (contact_offset, offset);
                    } vector3 lowest_vertex = vector3_addition (rigid_body -> position, contact_offset);
                    vector3 gravity_force = vector3_scaling (constant_gravity_acceleration, rigid_body -> mass);
                    float weight_along_normal = vector3_dot (gravity_force, (vector3){0, 1, 0});
                    if (weight_along_normal < 0) {
                        vector3 normal_force = vector3_scaling ((vector3){0, 1, 0}, -weight_along_normal);
                        rb_apply_forces_localised (rigid_body, normal_force, lowest_vertex);
                        rb_apply_forces_perfect (rigid_body, gravity_force);
                    } vector3 contact_normal = {0, 1, 0};
                    vector3 velocity_at_contact = vector3_addition (rigid_body -> velocity, vector3_cross (rigid_body -> angular_velocity, contact_offset));
                    vector3 tangential_velocity = vector3_subtraction (velocity_at_contact, vector3_scaling (contact_normal, vector3_dot (velocity_at_contact, contact_normal)));
                    if (vector3_length_squared (tangential_velocity) > 0.0001f) {
                        vector3 friction_force = vector3_scaling (vector3_normalisation (tangential_velocity), -rigid_body -> friction_kinetic * fabsf (rigid_body -> mass * world_gravity_y));
                        rb_apply_forces_localised (rigid_body, friction_force, lowest_vertex);
                    }
                }
            } else {rb_apply_forces_perfect (rigid_body, vector3_scaling (constant_gravity_acceleration, rigid_body -> mass));}
        } for (int collision_index = 0; collision_index < detected_collision_count; collision_index++) {
            rigidbody *rigid_body_a = &obj_per_scene [persistent_collision_pairs [collision_index].object_index_a];
            rigidbody *rigid_body_b = &obj_per_scene [persistent_collision_pairs [collision_index].object_index_b];
            collision_data narrowphase_collision;
            bool collided = false;
            if (rigid_body_a -> type == object_sphere && rigid_body_b -> type == object_sphere) collided = collision_dual_sphere (rigid_body_a, rigid_body_b, &narrowphase_collision);
            else if (rigid_body_a -> type == object_sphere && rigid_body_b -> type == object_cube) collided = collision_sphere_cube (rigid_body_a, rigid_body_b, &narrowphase_collision);
            else if (rigid_body_a -> type == object_cube && rigid_body_b -> type == object_sphere) {
                collided = collision_sphere_cube (rigid_body_b, rigid_body_a, &narrowphase_collision);
                narrowphase_collision.normal_vector = vector3_scaling (narrowphase_collision.normal_vector, -1.0f);
                narrowphase_collision.object_a = rigid_body_a; narrowphase_collision.object_b = rigid_body_b;
            } else if (rigid_body_a -> type == object_cube && rigid_body_b -> type == object_cube) collided = collision_dual_cube (rigid_body_a, rigid_body_b, &narrowphase_collision);
            if (collided) collision_resolve (&narrowphase_collision);
        } for (int object_iterator_index = 0; object_iterator_index < object_count; object_iterator_index++) {
            rigidbody *rigid_body = &obj_per_scene [object_iterator_index];
            rb_integrate (rigid_body, sub_step_dt, linear_damping_factor, angular_damping_factor);
            if (!main_inputs.is_debug_mode_active) {boundary_apply_box (rigid_body, (vector3){-250, 0, -250}, (vector3){250, 500, 250});}
            else {boundary_apply_floor (rigid_body, 0.0f);}
        }
    } gtk_widget_queue_draw (GTK_WIDGET (user_data_pointer));
    overlay_update ();
    return TRUE;
}
