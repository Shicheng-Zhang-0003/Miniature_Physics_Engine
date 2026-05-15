#include "../../master_header_2.h"
#include "../../../stage3/master_header_3.h"
#include "../../../stage1/master_header.h"
#include "../../../stage4/master_header_4.h"
#include "../../../stage5/master_header_5.h"
#include <complex.h>
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
float world_surface_friction_static = 0.3f;
float world_surface_friction_kinetic = 0.2f;
gboolean physics_step_increment (gpointer user_data_pointer) {
    float adjustment_increment;
    if (main_inputs.is_debug_mode_active) {adjustment_increment = 0.01f;}
    else {adjustment_increment = 0.2f;}
    static int status_dir_checked = 0;
    if (!status_dir_checked) {mkdir ("status", 0755); status_dir_checked = 1;}
    frame_timer_update (&main_timer);
    float frame_delta_time = main_timer.delta_time;
    // Camera Controls
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
        main_inputs.mouse_delta_x = 0.0f;
        main_inputs.mouse_delta_y = 0.0f;
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
        if (selected_object >= 0) {selector_apply_force_impulse (50.0f);}
        main_inputs.space_key_pressed = false;
    } // Spawn Logic (Shift Press/Hold) - Spheres (Machine Gun)
    static float shift_hold_timer = 0.0f;
    static float shift_spawn_interval_timer = 0.0f;
    static bool shift_previously_held = false;
    if (main_inputs.shift_key_pressed) {
        if (!shift_previously_held) {
            spawner_launch_sphere (spawn_radius, spawn_mass, spawn_speed);
            shift_hold_timer = 0.0f;
            shift_spawn_interval_timer = 0.0f;
        } else {
            shift_hold_timer += frame_delta_time;
            if (shift_hold_timer > 0.3f) {
                shift_spawn_interval_timer += frame_delta_time;
                if (shift_spawn_interval_timer >= 0.02f) { // Machine gun rate: 50/sec
                    spawner_launch_sphere (spawn_radius, spawn_mass, spawn_speed);
                    shift_spawn_interval_timer = 0.0f;
                }
            }
        } shift_previously_held = true;
    } else {
        shift_hold_timer = 0.0f;
        shift_previously_held = false;
    } // Cube Spawn Logic (C Press/Hold) (Machine Gun)
    static float c_hold_timer = 0.0f;
    static float c_spawn_interval_timer = 0.0f;
    static bool c_previously_held = false;
    if (main_inputs.c_key_pressed) {
        if (!c_previously_held) {
            spawner_launch_cube (main_camera_fov.position, (vector3) {spawn_cube_extent, spawn_cube_extent, spawn_cube_extent}, spawn_cube_mass);
            c_hold_timer = 0.0f;
            c_spawn_interval_timer = 0.0f;
        } else {
            c_hold_timer += frame_delta_time;
            if (c_hold_timer > 0.3f) {
                c_spawn_interval_timer += frame_delta_time;
                if (c_spawn_interval_timer >= 0.02f) { // Machine gun rate: 50/sec
                    spawner_launch_cube (main_camera_fov.position, (vector3) {spawn_cube_extent, spawn_cube_extent, spawn_cube_extent}, spawn_cube_mass);
                    c_spawn_interval_timer = 0.0f;
                }
            }
        } c_previously_held = true;
    } else {
        c_hold_timer = 0.0f;
        c_previously_held = false;
    } if (main_inputs.menu_1_pressed) {
        save_scene ("status/scene.dat");
        main_inputs.menu_1_pressed = false;
        main_inputs.is_menu_open = false;
    } if (main_inputs.menu_2_pressed) {
        scene_loading ("status/scene.dat");
        main_inputs.menu_2_pressed = false;
        main_inputs.is_menu_open = false;
    } if (main_inputs.menu_3_pressed) {
        main_inputs.menu_3_pressed = false;
        gtk_main_quit ();
    } // Spawner Menu Logic
    if (main_inputs.spawner_menu_level == 3) {
        if (main_inputs.up_arrow_pressed)   {spawn_mass += adjustment_increment; main_inputs.up_arrow_pressed = false;}
        if (main_inputs.down_arrow_pressed) {spawn_mass -= adjustment_increment; if (spawn_mass < 0.01f) {spawn_mass = 0.01f;} main_inputs.down_arrow_pressed = false;}
        if (main_inputs.enter_key_pressed)  {main_inputs.spawner_menu_level = 0; main_inputs.enter_key_pressed = false;}
    } if (main_inputs.spawner_menu_level == 4) {
        if (main_inputs.up_arrow_pressed)   {spawn_radius += adjustment_increment; main_inputs.up_arrow_pressed = false;}
        if (main_inputs.down_arrow_pressed) {spawn_radius -= adjustment_increment; if (spawn_radius < 0.01f) {spawn_radius = 0.01f;} main_inputs.down_arrow_pressed = false;}
        if (main_inputs.enter_key_pressed)  {main_inputs.spawner_menu_level = 0; main_inputs.enter_key_pressed = false;}
    } // Cube Menu Logic
    if (main_inputs.spawner_menu_level == 6) { // Cube Mass
        if (main_inputs.up_arrow_pressed)   {spawn_cube_mass += adjustment_increment; main_inputs.up_arrow_pressed = false;}
        if (main_inputs.down_arrow_pressed) {spawn_cube_mass -= adjustment_increment; if (spawn_cube_mass < 0.01f) {spawn_cube_mass = 0.01f;} main_inputs.down_arrow_pressed = false;}
        if (main_inputs.enter_key_pressed)  {main_inputs.spawner_menu_level = 0; main_inputs.enter_key_pressed = false;}
    } if (main_inputs.spawner_menu_level == 7) { // Cube Size
        if (main_inputs.up_arrow_pressed)   {spawn_cube_extent += adjustment_increment; main_inputs.up_arrow_pressed = false;}
        if (main_inputs.down_arrow_pressed) {spawn_cube_extent -= adjustment_increment; if (spawn_cube_extent < 0.01f) {spawn_cube_extent = 0.01f;} main_inputs.down_arrow_pressed = false;}
        if (main_inputs.enter_key_pressed)  {main_inputs.spawner_menu_level = 0; main_inputs.enter_key_pressed = false;}
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
    } if (main_inputs.velocity_menu_level == 21) {
        if (main_inputs.up_arrow_pressed)   {world_gravity_y += adjustment_increment; main_inputs.up_arrow_pressed = false;}
        if (main_inputs.down_arrow_pressed) {world_gravity_y -= adjustment_increment; main_inputs.down_arrow_pressed = false;}
        if (main_inputs.enter_key_pressed)  {main_inputs.velocity_menu_level = 0; main_inputs.enter_key_pressed = false;}
    } if (main_inputs.velocity_menu_level == 22) {
        if (main_inputs.up_arrow_pressed)   {world_drag_coefficient += adjustment_increment; if (world_drag_coefficient > 1.0f) {world_drag_coefficient = 1.0f;} main_inputs.up_arrow_pressed = false;}
        if (main_inputs.down_arrow_pressed) {world_drag_coefficient -= adjustment_increment; if (world_drag_coefficient < 0.0f) {world_drag_coefficient = 0.0f;} main_inputs.down_arrow_pressed = false;}
        if (main_inputs.enter_key_pressed)  {main_inputs.velocity_menu_level = 0; main_inputs.enter_key_pressed = false;}
    } if (main_inputs.velocity_menu_level == 23) {
        if (main_inputs.up_arrow_pressed)   {world_surface_friction_kinetic += adjustment_increment; world_surface_friction_static = world_surface_friction_kinetic + 0.1f; main_inputs.up_arrow_pressed = false;}
        if (main_inputs.down_arrow_pressed) {world_surface_friction_kinetic -= adjustment_increment; if (world_surface_friction_kinetic < 0.0f) {world_surface_friction_kinetic = 0.0f;} world_surface_friction_static = world_surface_friction_kinetic + 0.1f; main_inputs.down_arrow_pressed = false;}
        if (main_inputs.enter_key_pressed)  {main_inputs.velocity_menu_level = 0; main_inputs.enter_key_pressed = false;}
    } // Selected Object Menu Logic
    if (selected_object >= 0) {
        if (main_inputs.e_key_pressed) {
            main_inputs.e_key_pressed = false;
            if (main_inputs.object_menu_level > 0) {main_inputs.object_menu_level = 0;}
            else {main_inputs.object_menu_level = 1;}
            main_inputs.is_menu_open = false;
            main_inputs.spawner_menu_level = 0;
            main_inputs.velocity_menu_level = 0;
        } rigidbody *target = &obj_per_scene [selected_object];
        if (main_inputs.object_menu_level == 2) {
            if (main_inputs.up_arrow_pressed)   {
                target -> mass += adjustment_increment;
                target -> inverse_mass = 1.0f / target -> mass;
                if (target -> type == object_sphere) rigidbody_update_inertia_sphere (target);
                else rigidbody_update_inertia_cube (target);
                main_inputs.up_arrow_pressed = false;
            } if (main_inputs.down_arrow_pressed) {
                target -> mass -= adjustment_increment;
                if (target -> mass < 0.01f) {target -> mass = 0.01f;}
                target -> inverse_mass = 1.0f / target -> mass;
                if (target -> type == object_sphere) rigidbody_update_inertia_sphere (target);
                else rigidbody_update_inertia_cube (target);
                main_inputs.down_arrow_pressed = false;
            } if (main_inputs.enter_key_pressed)  {main_inputs.object_menu_level = 0; main_inputs.enter_key_pressed = false;}
        } else if (main_inputs.object_menu_level == 3) {
            if (main_inputs.up_arrow_pressed)   {
                if (target -> type == object_sphere) {
                    target -> radius += adjustment_increment;
                    rigidbody_update_inertia_sphere (target);
                } else {
                    target -> half_extensions = vector3_addition (target -> half_extensions, (vector3){adjustment_increment, adjustment_increment, adjustment_increment});
                    target -> radius = vector3_length (target -> half_extensions);
                    rigidbody_update_inertia_cube (target);
                } main_inputs.up_arrow_pressed = false;
            } if (main_inputs.down_arrow_pressed) {
                if (target -> type == object_sphere) {
                    target -> radius -= adjustment_increment;
                    if (target -> radius < 0.01f) {target -> radius = 0.01f;}
                    rigidbody_update_inertia_sphere (target);
                } else {
                    target -> half_extensions = vector3_subtraction (target -> half_extensions, (vector3){adjustment_increment, adjustment_increment, adjustment_increment});
                    if (target -> half_extensions.x < 0.01f) target -> half_extensions = (vector3){0.01f, 0.01f, 0.01f};
                    target -> radius = vector3_length (target -> half_extensions);
                    rigidbody_update_inertia_cube (target);
                } main_inputs.down_arrow_pressed = false;
            } if (main_inputs.enter_key_pressed)  {main_inputs.object_menu_level = 0; main_inputs.enter_key_pressed = false;}
        } else if (main_inputs.object_menu_level == 4) {
            if (main_inputs.up_arrow_pressed)   {target -> friction_kinetic += adjustment_increment; target -> friction_static = target -> friction_kinetic + 0.1f; main_inputs.up_arrow_pressed = false;}
            if (main_inputs.down_arrow_pressed) {target -> friction_kinetic -= adjustment_increment; if (target -> friction_kinetic < 0.0f) {target -> friction_kinetic = 0.0f;} target -> friction_static = target -> friction_kinetic + 0.1f; main_inputs.down_arrow_pressed = false;}
            if (main_inputs.enter_key_pressed)  {main_inputs.object_menu_level = 0; main_inputs.enter_key_pressed = false;}
        } else if (main_inputs.object_menu_level == 5) {
            if ((main_inputs.up_arrow_pressed) || (main_inputs.down_arrow_pressed)) {
                target -> static_state = !target -> static_state;
                if (target -> static_state) {
                    target -> velocity = vector3_zero ();
                    target -> angular_velocity = vector3_zero ();
                } main_inputs.up_arrow_pressed = false; main_inputs.down_arrow_pressed = false;
            } if (main_inputs.enter_key_pressed)  {main_inputs.object_menu_level = 0; main_inputs.enter_key_pressed = false;}
        }
    } else {main_inputs.object_menu_level = 0;}
    if (main_inputs.left_mouse_button_clicked) {
        if (selected_object >= 0) {
            for (int object_shift_index = selected_object; object_shift_index < (object_count - 1); object_shift_index++) {obj_per_scene [object_shift_index] = obj_per_scene [object_shift_index + 1];}
            object_count -= 1;
            clear_selection ();
        } main_inputs.left_mouse_button_clicked = false;
    } //Core Physics Substeps
    const int physics_sub_steps = 12;
    float sub_step_dt = frame_delta_time / (float)physics_sub_steps;
    // Static buffers for broadphase to avoid reallocation lag
    static broadphase_pair *persistent_collision_pairs = NULL;
    static int persistent_pairs_capacity = 0;
    for (int sub_iteration = 0; sub_iteration < physics_sub_steps; sub_iteration++) {
        //Force Accumulation
        for (int object_iterator_index = 0; object_iterator_index < object_count; object_iterator_index++) {
            vector3 constant_gravity_acceleration = {0, world_gravity_y, 0};
            rigidbody *rb = &obj_per_scene [object_iterator_index];
            vector3 up_axis = {0, 1, 0};
            float projection = rb -> radius;
            if (rb -> type == object_cube) {
                vector3 axes [3];
                get_obb_axes (rb, axes);
                projection = rb -> half_extensions.x * fabsf (vector3_dot (axes [0], up_axis)) +
                             rb -> half_extensions.y * fabsf (vector3_dot (axes [1], up_axis)) +
                             rb -> half_extensions.z * fabsf (vector3_dot (axes [2], up_axis));
            } if (rb -> position.y <= (projection + 0.01f)) {
                if (rb -> type == object_sphere) {
                    force_applicant_gravity_normal (rb, constant_gravity_acceleration, (vector3) {0.0f, 1.0f, 0.0f});
                    force_applicant_friction_rolling (rb, (vector3) {0.0f, 1.0f, 0.0f}, rb -> friction_static, rb -> friction_kinetic, world_gravity_y);
                } else {
                    vector3 axes [3];
                    get_obb_axes (rb, axes);
                    vector3 contact_offset = {0,0,0};
                    for (int i = 0; i < 3; i++) {
                        float extent = (i == 0) ? rb -> half_extensions.x : (i == 1 ? rb -> half_extensions.y : rb -> half_extensions.z);
                        vector3 offset = vector3_scaling (axes [i], extent);
                        if (vector3_dot (offset, (vector3){0, -1, 0}) > 0) contact_offset = vector3_addition (contact_offset, offset);
                        else contact_offset = vector3_subtraction (contact_offset, offset);
                    } vector3 lowest_vertex = vector3_addition (rb -> position, contact_offset);
                    vector3 gravity_force = vector3_scaling (constant_gravity_acceleration, rb -> mass);
                    float weight_along_normal = vector3_dot (gravity_force, (vector3){0, 1, 0});
                    if (weight_along_normal < 0) {
                        vector3 normal_force = vector3_scaling ((vector3){0, 1, 0}, -weight_along_normal);
                        rb_apply_forces_localised (rb, normal_force, lowest_vertex);
                        rb_apply_forces_perfect (rb, gravity_force);
                    } // Simple friction
                    vector3 contact_normal = {0, 1, 0};
                    vector3 velocity_at_contact = vector3_addition (rb -> velocity, vector3_cross (rb -> angular_velocity, contact_offset));
                    vector3 tangential_velocity = vector3_subtraction (velocity_at_contact, vector3_scaling (contact_normal, vector3_dot (velocity_at_contact, contact_normal)));
                    if (vector3_length_squared (tangential_velocity) > 0.0001f) {
                        vector3 friction_force = vector3_scaling (vector3_normalisation (tangential_velocity), -rb -> friction_kinetic * fabsf (rb -> mass * world_gravity_y));
                        rb_apply_forces_localised (rb, friction_force, lowest_vertex);
                    }
                }
            } else {rb_apply_forces_perfect (rb, vector3_scaling (constant_gravity_acceleration, rb -> mass));}
        } //Collision Detection & Resolution (Must happen every sub-step for objects to feel each other)
        int maximum_possible_pairs = (object_count * (object_count - 1)) / 2;
        if (maximum_possible_pairs < 1) maximum_possible_pairs = 1;
        if (maximum_possible_pairs > persistent_pairs_capacity) {
            persistent_pairs_capacity = maximum_possible_pairs + 128;
            persistent_collision_pairs = realloc (persistent_collision_pairs, persistent_pairs_capacity * sizeof (broadphase_pair));
        } if (persistent_collision_pairs) {
            int detected_collision_count = broadphase_generate_pairing (persistent_collision_pairs, persistent_pairs_capacity);
            for (int collision_index = 0; collision_index < detected_collision_count; collision_index++) {
                rigidbody *rb_a = &obj_per_scene [persistent_collision_pairs [collision_index].object_index_a];
                rigidbody *rb_b = &obj_per_scene [persistent_collision_pairs [collision_index].object_index_b];
                collision_data narrowphase_collision;
                bool collided = false;
                if (rb_a -> type == object_sphere && rb_b -> type == object_sphere) collided = collision_dual_sphere (rb_a, rb_b, &narrowphase_collision);
                else if (rb_a -> type == object_sphere && rb_b -> type == object_cube) collided = collision_sphere_cube (rb_a, rb_b, &narrowphase_collision);
                else if (rb_a -> type == object_cube && rb_b -> type == object_sphere) {
                    collided = collision_sphere_cube (rb_b, rb_a, &narrowphase_collision);
                    narrowphase_collision.normal_vector = vector3_scaling (narrowphase_collision.normal_vector, -1.0f);
                    narrowphase_collision.object_a = rb_a; narrowphase_collision.object_b = rb_b;
                } else if (rb_a -> type == object_cube && rb_b -> type == object_cube) collided = collision_dual_cube (rb_a, rb_b, &narrowphase_collision);
                if (collided) collision_resolve (&narrowphase_collision);
            }
        } //Integration & Boundaries
        for (int object_iterator_index = 0; object_iterator_index < object_count; object_iterator_index++) {
            rigidbody *rb = &obj_per_scene [object_iterator_index];
            rb_integrate (rb, sub_step_dt, world_drag_coefficient);
            if (!main_inputs.is_debug_mode_active) {boundary_apply_box (rb, (vector3){-250, 0, -250}, (vector3){250, 500, 250});}
            else {boundary_apply_floor (rb, 0.0f);}
        }
    } //Physics Substep Block
    gtk_widget_queue_draw (GTK_WIDGET (user_data_pointer));
    overlay_update ();
    return TRUE;
}
