#include "../mpe_engine.h"
#include "editor.h"

static void editor_reacquire_mouse (GtkWidget *parent_window) {
    if (!parent_window) {return;}
    mouse_lock_reacquire (gtk_widget_get_toplevel (GTK_WIDGET (parent_window)));
}

static rigidbody *editor_selected_object_or_null (void) {
    /* A3_PATCH_25_DIALOG_SAFETY */
    if ((selected_object < 0) || (selected_object >= object_count)) {return NULL;}
    return &obj_per_scene [selected_object];
}

void editor_update_menus (GtkWidget *parent_window) {
    /* A3_PATCH_24_MENU_STATE_MACHINE */
    if ((selected_object < 0) || (selected_object >= object_count)) {
        if (main_inputs.object_menu_level > 0) {main_inputs.object_menu_level = 0;}
    }

    if ((main_inputs.marked_joint_object_index < 0) || (main_inputs.marked_joint_object_index >= object_count)) {
        main_inputs.marked_joint_object_index = -1;
    }

    // Spawner Menu Logic
    if (main_inputs.spawner_menu_level == 3) {
        spawn_mass = open_numerical_input_dialog (parent_window, "Sphere Mass (kg)", spawn_mass);
        editor_reacquire_mouse (parent_window);
        if (spawn_mass < 0.01f) {spawn_mass = 0.01f;}
        main_inputs.spawner_menu_level = 0;
    } else if (main_inputs.spawner_menu_level == 4) {
        spawn_radius = open_numerical_input_dialog (parent_window, "Sphere Radius (m)", spawn_radius);
        editor_reacquire_mouse (parent_window);
        if (spawn_radius < 0.01f) {spawn_radius = 0.01f;}
        main_inputs.spawner_menu_level = 0;
    } else if (main_inputs.spawner_menu_level == 6) {
        spawn_cube_mass = open_numerical_input_dialog (parent_window, "Cube Mass (kg)", spawn_cube_mass);
        editor_reacquire_mouse (parent_window);
        if (spawn_cube_mass < 0.01f) {spawn_cube_mass = 0.01f;}
        main_inputs.spawner_menu_level = 0;
    } else if (main_inputs.spawner_menu_level == 7) {
        spawn_cube_extent = open_numerical_input_dialog (parent_window, "Cube Size (m)", spawn_cube_extent);
        editor_reacquire_mouse (parent_window);
        if (spawn_cube_extent < 0.01f) {spawn_cube_extent = 0.01f;}
        main_inputs.spawner_menu_level = 0;
    }

    if (main_inputs.spawner_menu_level == 8) {
        if ((main_inputs.up_arrow_pressed) || (main_inputs.down_arrow_pressed)) {
            if (main_inputs.current_spawn_type == 0) {main_inputs.current_spawn_type = 1;}
            else {main_inputs.current_spawn_type = 0;}
            main_inputs.up_arrow_pressed = false;
            main_inputs.down_arrow_pressed = false;
        }

        if (main_inputs.enter_key_pressed) {
            main_inputs.spawner_menu_level = 0;
            main_inputs.enter_key_pressed = false;
        }
    }

    // User Mechanics Menu Logic
    if (main_inputs.velocity_menu_level == 3) {
        spawn_speed = open_numerical_input_dialog (parent_window, "Spawn Speed (m/s)", spawn_speed);
        editor_reacquire_mouse (parent_window);
        if (spawn_speed < 0.0f) {spawn_speed = 0.0f;}
        main_inputs.velocity_menu_level = 0;
    } else if (main_inputs.velocity_menu_level == 4) {
        world_surface_friction_kinetic = open_numerical_input_dialog (parent_window, "Spawn Friction (Kinetic)", world_surface_friction_kinetic);
        editor_reacquire_mouse (parent_window);
        if (world_surface_friction_kinetic < 0.0f) {world_surface_friction_kinetic = 0.0f;}
        world_surface_friction_static = world_surface_friction_kinetic + 0.1f;
        main_inputs.velocity_menu_level = 0;
    } else if (main_inputs.velocity_menu_level == 11) {
        main_camera_fov.movement_speed = open_numerical_input_dialog (parent_window, "Camera Speed", main_camera_fov.movement_speed);
        editor_reacquire_mouse (parent_window);
        if (main_camera_fov.movement_speed < 0.01f) {main_camera_fov.movement_speed = 0.01f;}
        main_inputs.velocity_menu_level = 0;
    } else if (main_inputs.velocity_menu_level == 12) {
        jump_height = open_numerical_input_dialog (parent_window, "Jump Height (m)", jump_height);
        editor_reacquire_mouse (parent_window);
        if (jump_height < 0.1f) {jump_height = 0.1f;}
        main_inputs.velocity_menu_level = 0;
    } else if (main_inputs.velocity_menu_level == 21) {
        world_gravity_y = open_numerical_input_dialog (parent_window, "World Gravity (m/s^2)", world_gravity_y);
        editor_reacquire_mouse (parent_window);
        main_inputs.velocity_menu_level = 0;
    } else if (main_inputs.velocity_menu_level == 22) {
        world_drag_coefficient = open_numerical_input_dialog (parent_window, "World Drag Coefficient", world_drag_coefficient);
        editor_reacquire_mouse (parent_window);
        if (world_drag_coefficient > 1.0f) {world_drag_coefficient = 1.0f;}
        if (world_drag_coefficient < 0.1f) {world_drag_coefficient = 0.1f;}
        main_inputs.velocity_menu_level = 0;
    } else if (main_inputs.velocity_menu_level == 23) {
        world_surface_friction_kinetic = open_numerical_input_dialog (parent_window, "World Surface Friction", world_surface_friction_kinetic);
        editor_reacquire_mouse (parent_window);
        if (world_surface_friction_kinetic < 0.0f) {world_surface_friction_kinetic = 0.0f;}
        world_surface_friction_static = world_surface_friction_kinetic + 0.1f;
        main_inputs.velocity_menu_level = 0;
    }

    // Selected Object Menu Logic
    if (main_inputs.object_menu_level == 2) {
        rigidbody *selected_rigid_body = editor_selected_object_or_null ();
        if (!selected_rigid_body) {main_inputs.object_menu_level = 0; return;}
        selected_rigid_body -> mass = open_numerical_input_dialog (parent_window, "Selected Object Mass", selected_rigid_body -> mass);
        editor_reacquire_mouse (parent_window);
        if (selected_rigid_body -> mass < 0.01f) {selected_rigid_body -> mass = 0.01f;}
        selected_rigid_body -> inverse_mass = 1.0f / selected_rigid_body -> mass;
        if (selected_rigid_body -> type == object_sphere) {rigidbody_update_inertia_sphere (selected_rigid_body);}
        else {rigidbody_update_inertia_cube (selected_rigid_body);}
        main_inputs.object_menu_level = 0;
    } else if (main_inputs.object_menu_level == 3) {
        rigidbody *selected_rigid_body = editor_selected_object_or_null ();
        if (!selected_rigid_body) {main_inputs.object_menu_level = 0; return;}
        if (selected_rigid_body -> type == object_sphere) {
            selected_rigid_body -> radius = open_numerical_input_dialog (parent_window, "Selected Object Radius", selected_rigid_body -> radius);
            if (selected_rigid_body -> radius < 0.01f) {selected_rigid_body -> radius = 0.01f;}
            editor_reacquire_mouse (parent_window);
            rigidbody_update_inertia_sphere (selected_rigid_body);
        }
        main_inputs.object_menu_level = 0;
    } else if (main_inputs.object_menu_level == 4) {
        rigidbody *selected_rigid_body = editor_selected_object_or_null ();
        if (!selected_rigid_body) {main_inputs.object_menu_level = 0; return;}
        selected_rigid_body -> friction_kinetic = open_numerical_input_dialog (parent_window, "Selected Object Friction", selected_rigid_body -> friction_kinetic);
        editor_reacquire_mouse (parent_window);
        if (selected_rigid_body -> friction_kinetic < 0.0f) {selected_rigid_body -> friction_kinetic = 0.0f;}
        selected_rigid_body -> friction_static = selected_rigid_body -> friction_kinetic + 0.1f;
        main_inputs.object_menu_level = 0;
    }

    if (main_inputs.object_menu_level == 5) {
        rigidbody *selected_rigid_body = editor_selected_object_or_null ();
        if (!selected_rigid_body) {main_inputs.object_menu_level = 0; return;}
        if ((main_inputs.up_arrow_pressed) || (main_inputs.down_arrow_pressed)) {
            selected_rigid_body -> static_state = !selected_rigid_body -> static_state;
            if (selected_rigid_body -> static_state) {
                selected_rigid_body -> inverse_mass = 0.0f;
                selected_rigid_body -> velocity = vector3_zero ();
                selected_rigid_body -> angular_velocity = vector3_zero ();
            } else {
                if (selected_rigid_body -> mass > 0) {selected_rigid_body -> inverse_mass = 1.0f / selected_rigid_body -> mass;}
                rigidbody_wake (selected_rigid_body);
            }
            main_inputs.up_arrow_pressed = false;
            main_inputs.down_arrow_pressed = false;
        }

        if (main_inputs.enter_key_pressed) {
            main_inputs.object_menu_level = 0;
            main_inputs.enter_key_pressed = false;
        }
    } else if (main_inputs.object_menu_level == 6) {
        main_inputs.marked_joint_object_index = selected_object;
        main_inputs.object_menu_level = 0;
    } else if (main_inputs.object_menu_level == 7) {
        if (main_inputs.marked_joint_object_index != -1 && main_inputs.marked_joint_object_index < object_count && main_inputs.marked_joint_object_index != selected_object) {
            rigidbody *rb_a = &obj_per_scene [main_inputs.marked_joint_object_index];
            rigidbody *rb_b = &obj_per_scene [selected_object];
            float dist = vector3_length (vector3_subtraction (rb_b -> position, rb_a -> position));
            add_joint (main_inputs.marked_joint_object_index, selected_object, dist, 100.0f, 2.0f);
        }
        main_inputs.marked_joint_object_index = -1;
        main_inputs.object_menu_level = 0;
    } else if (main_inputs.object_menu_level >= 81 && main_inputs.object_menu_level <= 88) {
        rigidbody *selected_rigid_body = editor_selected_object_or_null ();
        if (!selected_rigid_body) {main_inputs.object_menu_level = 0; return;}
        if (main_inputs.object_menu_level == 81) { selected_rigid_body -> colour = (vector3){1.0f, 0.0f, 0.0f}; }
        else if (main_inputs.object_menu_level == 82) { selected_rigid_body -> colour = (vector3){0.0f, 1.0f, 0.0f}; }
        else if (main_inputs.object_menu_level == 83) { selected_rigid_body -> colour = (vector3){0.0f, 0.0f, 1.0f}; }
        else if (main_inputs.object_menu_level == 84) { selected_rigid_body -> colour = (vector3){1.0f, 0.4f, 0.2f}; }
        else if (main_inputs.object_menu_level == 85) { selected_rigid_body -> colour = (vector3){0.0f, 1.0f, 1.0f}; }
        else if (main_inputs.object_menu_level == 86) { selected_rigid_body -> colour = (vector3){1.0f, 0.0f, 1.0f}; }
        else if (main_inputs.object_menu_level == 87) { selected_rigid_body -> colour = (vector3){1.0f, 1.0f, 0.0f}; }
        else if (main_inputs.object_menu_level == 88) { selected_rigid_body -> colour = (vector3){1.0f, 1.0f, 1.0f}; }
        main_inputs.object_menu_level = 0;
    }
}
