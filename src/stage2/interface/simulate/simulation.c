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
    } //Spawn Logic (Shift Press/Hold)
    static float shift_hold_timer = 0.0f;
    static bool shift_previously_held = false;
    if (main_inputs.shift_key_pressed) {
        if (! shift_previously_held) {
            // Initial simple spawn on press
            spawner_launch_sphere (spawn_radius, spawn_mass, spawn_speed);
            shift_hold_timer = 0.0f;
        } else {
            // Key is being held
            shift_hold_timer += frame_delta_time;
            if (shift_hold_timer > 1.0f) {
                //Continuous spawning
                spawner_launch_sphere (spawn_radius, spawn_mass, spawn_speed);
            }
        } shift_previously_held = true;
    } else {
        shift_hold_timer = 0.0f;
        shift_previously_held = false;
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
    } // User Mechanics Menu Logic (Renamed from Velocity Menu)
    if (main_inputs.velocity_menu_level == 3) {
        if (main_inputs.up_arrow_pressed)   {spawn_speed += adjustment_increment; main_inputs.up_arrow_pressed = false;}
        if (main_inputs.down_arrow_pressed) {spawn_speed -= adjustment_increment; if (spawn_speed < 0.0f) {spawn_speed = 0.0f;} main_inputs.down_arrow_pressed = false;}
        if (main_inputs.enter_key_pressed)  {main_inputs.velocity_menu_level = 0; main_inputs.enter_key_pressed = false;}
    } if (main_inputs.velocity_menu_level == 4) {
        if (main_inputs.up_arrow_pressed)   {friction_kinetic += adjustment_increment; friction_static = friction_kinetic + 0.1f; main_inputs.up_arrow_pressed = false;}
        if (main_inputs.down_arrow_pressed) {friction_kinetic -= adjustment_increment; if (friction_kinetic < 0.0f) {friction_kinetic = 0.0f;} friction_static = friction_kinetic + 0.1f; main_inputs.down_arrow_pressed = false;}
        if (main_inputs.enter_key_pressed)  {main_inputs.velocity_menu_level = 0; main_inputs.enter_key_pressed = false;}
    } if (main_inputs.velocity_menu_level == 11) {
        if (main_inputs.up_arrow_pressed)   {main_camera_fov.movement_speed += adjustment_increment; main_inputs.up_arrow_pressed = false;}
        if (main_inputs.down_arrow_pressed) {main_camera_fov.movement_speed -= adjustment_increment; if (main_camera_fov.movement_speed < 0.01f) {main_camera_fov.movement_speed = 0.01f;} main_inputs.down_arrow_pressed = false;}
        if (main_inputs.enter_key_pressed)  {main_inputs.velocity_menu_level = 0; main_inputs.enter_key_pressed = false;}
    } if (main_inputs.velocity_menu_level == 21) { // World Gravity
        if (main_inputs.up_arrow_pressed)   {world_gravity_y += adjustment_increment; main_inputs.up_arrow_pressed = false;}
        if (main_inputs.down_arrow_pressed) {world_gravity_y -= adjustment_increment; main_inputs.down_arrow_pressed = false;}
        if (main_inputs.enter_key_pressed)  {main_inputs.velocity_menu_level = 0; main_inputs.enter_key_pressed = false;}
    } if (main_inputs.velocity_menu_level == 22) { // World Air Resistance (Drag)
        if (main_inputs.up_arrow_pressed)   {world_drag_coefficient += adjustment_increment; if (world_drag_coefficient > 1.0f) {world_drag_coefficient = 1.0f;} main_inputs.up_arrow_pressed = false;}
        if (main_inputs.down_arrow_pressed) {world_drag_coefficient -= adjustment_increment; if (world_drag_coefficient < 0.0f) {world_drag_coefficient = 0.0f;} main_inputs.down_arrow_pressed = false;}
        if (main_inputs.enter_key_pressed)  {main_inputs.velocity_menu_level = 0; main_inputs.enter_key_pressed = false;}
    } if (main_inputs.velocity_menu_level == 23) { // World Surface Friction (Floor)
        if (main_inputs.up_arrow_pressed)   {world_surface_friction_kinetic += adjustment_increment; world_surface_friction_static = world_surface_friction_kinetic + 0.1f; main_inputs.up_arrow_pressed = false;}
        if (main_inputs.down_arrow_pressed) {world_surface_friction_kinetic -= adjustment_increment; if (world_surface_friction_kinetic < 0.0f) {world_surface_friction_kinetic = 0.0f;} world_surface_friction_static = world_surface_friction_kinetic + 0.1f; main_inputs.down_arrow_pressed = false;}
        if (main_inputs.enter_key_pressed)  {main_inputs.velocity_menu_level = 0; main_inputs.enter_key_pressed = false;}
    } // Selected Object Menu Logic
    if (selected_object >= 0) {
        if (main_inputs.e_key_pressed) {
            main_inputs.e_key_pressed = false;
            if (main_inputs.object_menu_level > 0) {main_inputs.object_menu_level = 0;}
            else {main_inputs.object_menu_level = 1;}
            // Close other menus
            main_inputs.is_menu_open = false;
            main_inputs.spawner_menu_level = 0;
            main_inputs.velocity_menu_level = 0;
        } rigidbody *target = &obj_per_scene [selected_object];
        if (main_inputs.object_menu_level == 2) { // Mass
            if (main_inputs.up_arrow_pressed)   {target -> mass += adjustment_increment; target -> inverse_mass = 1.0f / target -> mass; rigidbody_update_inertia_sphere (target); main_inputs.up_arrow_pressed = false;}
            if (main_inputs.down_arrow_pressed) {target -> mass -= adjustment_increment; if (target -> mass < 0.01f) {target -> mass = 0.01f;} target -> inverse_mass = 1.0f / target -> mass; rigidbody_update_inertia_sphere (target); main_inputs.down_arrow_pressed = false;}
            if (main_inputs.enter_key_pressed)  {main_inputs.object_menu_level = 0; main_inputs.enter_key_pressed = false;}
        } else if (main_inputs.object_menu_level == 3) { // Radius
            if (main_inputs.up_arrow_pressed)   {target -> radius += adjustment_increment; rigidbody_update_inertia_sphere (target); main_inputs.up_arrow_pressed = false;}
            if (main_inputs.down_arrow_pressed) {target -> radius -= adjustment_increment; if (target -> radius < 0.01f) {target -> radius = 0.01f;} rigidbody_update_inertia_sphere (target); main_inputs.down_arrow_pressed = false;}
            if (main_inputs.enter_key_pressed)  {main_inputs.object_menu_level = 0; main_inputs.enter_key_pressed = false;}
        } else if (main_inputs.object_menu_level == 4) { // Friction
            if (main_inputs.up_arrow_pressed)   {target -> friction_kinetic += adjustment_increment; target -> friction_static = target -> friction_kinetic + 0.1f; main_inputs.up_arrow_pressed = false;}
            if (main_inputs.down_arrow_pressed) {target -> friction_kinetic -= adjustment_increment; if (target -> friction_kinetic < 0.0f) {target -> friction_kinetic = 0.0f;} target -> friction_static = target -> friction_kinetic + 0.1f; main_inputs.down_arrow_pressed = false;}
            if (main_inputs.enter_key_pressed)  {main_inputs.object_menu_level = 0; main_inputs.enter_key_pressed = false;}
        } else if (main_inputs.object_menu_level == 5) { // Immovable Toggle
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
            // Shift array (fill gap)
            for (int object_shift_index = selected_object; object_shift_index < (object_count - 1); object_shift_index++) {obj_per_scene [object_shift_index] = obj_per_scene [object_shift_index + 1];}
            object_count -= 1;
            clear_selection ();
        } main_inputs.left_mouse_button_clicked = false;
    } for (int object_iterator_index = 0; object_iterator_index < object_count; object_iterator_index++) {
        vector3 constant_gravity_acceleration = {0, world_gravity_y, 0};
        //Normal Force (Floor check y=0)
        if (obj_per_scene [object_iterator_index].position.y <= (obj_per_scene [object_iterator_index].radius + 0.01f)) {
            force_applicant_gravity_normal (&obj_per_scene [object_iterator_index], constant_gravity_acceleration, (vector3) {0.0f, 1.0f, 0.0f});
            //Apply Floor Friction (Induce Rotation)
            force_applicant_friction_rolling (&obj_per_scene [object_iterator_index], (vector3) {0.0f, 1.0f, 0.0f}, obj_per_scene [object_iterator_index].friction_static, obj_per_scene [object_iterator_index].friction_kinetic, world_gravity_y);
        } else {
            // Gravity (Freefall)
            rb_apply_forces_perfect (&obj_per_scene [object_iterator_index], vector3_scaling (constant_gravity_acceleration, obj_per_scene [object_iterator_index].mass));
        }
    } //Resolve Collisions
    int maximum_possible_pairs = (object_count * (object_count - 1)) / 2;
    if (maximum_possible_pairs < 1) {maximum_possible_pairs = 1;}
    broadphase_pair *broadphase_collision_pairs = malloc (maximum_possible_pairs * sizeof (broadphase_pair));
    if (broadphase_collision_pairs) {
        int detected_collision_count = broadphase_generate_pairing (broadphase_collision_pairs, maximum_possible_pairs);
        for (int collision_index = 0; collision_index < detected_collision_count; collision_index++) {
            collision_data narrowphase_collision;
            if (collision_dual_sphere (&obj_per_scene [broadphase_collision_pairs [collision_index].object_index_a], &obj_per_scene [broadphase_collision_pairs [collision_index].object_index_b], &narrowphase_collision)) {collision_resolve (&narrowphase_collision);}
        } free (broadphase_collision_pairs);
    } //Integrate and Redraw
    for (int object_iterator_index = 0; object_iterator_index < object_count; object_iterator_index++) {rb_integrate (&obj_per_scene [object_iterator_index], frame_delta_time, world_drag_coefficient);}
    if (!main_inputs.is_debug_mode_active) {
        vector3 boundary_minimum_bounds = {-250.0f, 0.0f, -250.0f};
        vector3 boundary_maximum_bounds = {250.0f, 500.0f, 250.0f};
        for (int object_iterator_index = 0; object_iterator_index < object_count; object_iterator_index++) {boundary_apply_box (&obj_per_scene [object_iterator_index], boundary_minimum_bounds, boundary_maximum_bounds);}
    } else {
        for (int object_iterator_index = 0; object_iterator_index < object_count; object_iterator_index++) {boundary_apply_floor (&obj_per_scene [object_iterator_index], 0.0f);}
    } gtk_widget_queue_draw (GTK_WIDGET (user_data_pointer));
    overlay_update ();
    return TRUE;
}
