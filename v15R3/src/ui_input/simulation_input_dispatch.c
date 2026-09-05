/* MFS_INCREMENT_SPLIT_3: Input dispatch extracted from simulation.c.
* Owns: mouse/keyboard bindings, keyboard-only actions, test keys, spawn gun.
*/
#include "../mpe_engine.h"
#include "../core/validation_report.h"    /* MFS_INCREMENT_SPLIT: validation_report_print */
#include "../core/long_run_validation.h"
#include "../robotics/gui_robot_registry.h"
void simulation_input_dispatch(GtkWidget *parent_window) {
    /* MFS_157_GAMEPAD_POLL: drain gamepad events once per frame */
    gamepad_poll(gamepad_get_primary());
    /* Mouse, Escape, E, F key bindings */
    if (main_inputs.escape_key_pressed) {
        if (main_inputs.is_mouse_locked) {
            mouse_lock_disable(parent_window);
            main_inputs.is_mouse_locked = false;
        }
        main_inputs.escape_key_pressed = false;
    }
    if (main_inputs.right_mouse_button_clicked) {
        selector_ray_tracing();
        main_inputs.right_mouse_button_clicked = false;
    }
    if (main_inputs.middle_mouse_button_clicked) {
        if (selected_object >= 0) {
            scene_remove_object_by_index(selected_object);
        }
        main_inputs.middle_mouse_button_clicked = false;
    }
    if (main_inputs.e_key_pressed) {
        if (selected_object >= 0) {
            if (main_inputs.object_menu_level > 0) {
                main_inputs.object_menu_level = 0;
            } else {
                main_inputs.object_menu_level = 1;
            }
        }
            config_menu_close();
    }
    if (main_inputs.f_key_pressed) {
        if (selected_object >= 0) {
            selector_apply_force_impulse(250.0f);
        }
        main_inputs.f_key_pressed = false;
    }

    /* Keyboard-only actions */
    if (main_inputs.r_key_pressed) {
        if (main_inputs.is_debug_mode_active) {
            selector_ray_tracing();
        }
        main_inputs.r_key_pressed = false;
    }
    if (main_inputs.delete_key_pressed) {
        if ((main_inputs.is_debug_mode_active) && (selected_object >= 0) && (selected_object < object_count)) {
            scene_remove_object_by_index(selected_object);
        }
        main_inputs.delete_key_pressed = false;
    }
    if (main_inputs.m_key_pressed) {
        if ((main_inputs.is_debug_mode_active) && (!main_inputs.is_mouse_locked) && (parent_window)) {
            mouse_lock_enable(parent_window);
            main_inputs.is_mouse_locked = true;
        }
        main_inputs.m_key_pressed = false;
    }
    /* T key is now exclusively for robot forward drive */
    /* Test key bindings (F5-F11) */
    if (main_inputs.stability_test_pressed) {
        scene_spawn_stability_stack();
        main_inputs.stability_test_pressed = false;
    }
    if (main_inputs.sleep_wake_test_pressed) {
        scene_spawn_sleep_wake_test();
        main_inputs.sleep_wake_test_pressed = false;
    }
    if (main_inputs.editor_torture_pressed) {
        scene_editor_torture_test();
        main_inputs.editor_torture_pressed = false;
    }
    if (main_inputs.spawn_stress_pressed) {
        scene_spawn_stress_test();
        main_inputs.spawn_stress_pressed = false;
    }
    if (main_inputs.validation_report_pressed) {
        validation_report_print();
        main_inputs.validation_report_pressed = false;
    }
    if (main_inputs.debug_terminal_pressed) {
        if (main_inputs.is_debug_mode_active) {
            debug_terminal_open(parent_window);
        }
        main_inputs.debug_terminal_pressed = false;
    }
    if (main_inputs.long_run_validation_pressed) {
        scene_spawn_long_run_validation();
        long_run_validation_start(a3_long_run_validation_ticks);
        long_run_validation_restore_config = 1;
        main_inputs.long_run_validation_pressed = false;
    }
    if (main_inputs.config_torture_pressed) {
        mpe_config_save("status/engine.cfg.backup");
        scene_spawn_config_torture_test();
        long_run_validation_start(a3_long_run_validation_ticks);
        long_run_validation_restore_config = 1;
        main_inputs.config_torture_pressed = false;
    }

    /* MFS_159_GAMEPAD_ONLY: F310 gamepad is the sole drive input.
 * Left stick Y = forward/back, Left stick X = strafe,
 * Right stick X = rotate. GVBNCH keyboard drive removed. */
    if (gui_robot_get_count() > 0) {
        float drive_forward = 0.0f, drive_strafe = 0.0f, drive_rotate = 0.0f;
        gamepad_state *mfs_pad = gamepad_get_primary();
        if (gamepad_is_connected(mfs_pad)) {
            drive_forward = gamepad_get_axis(mfs_pad, gamepad_axis_left_y);
            drive_strafe  = -gamepad_get_axis(mfs_pad, gamepad_axis_left_x); /* MFS_160_STRAFE_NEG */
            drive_rotate  = gamepad_get_axis(mfs_pad, gamepad_axis_right_x);
            /* clamp to [-1, 1] */
            if (drive_forward >  1.0f) drive_forward =  1.0f;
            if (drive_forward < -1.0f) drive_forward = -1.0f;
            if (drive_strafe  >  1.0f) drive_strafe  =  1.0f;
            if (drive_strafe  < -1.0f) drive_strafe  = -1.0f;
            if (drive_rotate  >  1.0f) drive_rotate  =  1.0f;
            if (drive_rotate  < -1.0f) drive_rotate  = -1.0f;
        }
        gui_robot_apply_drive(drive_forward, drive_strafe, drive_rotate);
    }

/* Spawn gun (Enter hold) */
    static float enter_hold_timer = 0.0f;
    static float enter_spawn_interval_timer = 0.0f;
    static bool enter_previously_held = false;

    if ((main_inputs.enter_spawn_held) && (!editor_dialog_is_active()) && (!main_inputs.is_menu_open) &&
        (main_inputs.spawner_menu_level == 0) && (main_inputs.velocity_menu_level == 0) &&
        (main_inputs.object_menu_level == 0)) {
        if (!enter_previously_held) {
            if (main_inputs.current_spawn_type == 0) {
                spawner_launch_sphere(g_cfg.spawner.radius, g_cfg.spawner.mass, g_cfg.spawner.speed);
            } else {
                vector3 cube_spawn_position = vector3_addition(
                    main_camera_fov.position,
                    vector3_scaling(main_camera_fov.forward_vector, g_cfg.spawner.cube_extent + 1.0f));
                spawner_launch_cube(cube_spawn_position,
                                    (vector3){g_cfg.spawner.cube_extent, g_cfg.spawner.cube_extent, g_cfg.spawner.cube_extent},
                                    g_cfg.spawner.cube_mass);
            }
            enter_hold_timer = 0.0f;
            enter_spawn_interval_timer = 0.0f;
        } else {
            enter_hold_timer += main_timer.delta_time;
            if (enter_hold_timer > g_cfg.ui.enter_spawn_delay) {
                enter_spawn_interval_timer += main_timer.delta_time;
                if (enter_spawn_interval_timer >= g_cfg.ui.enter_spawn_interval) {
                    if (main_inputs.current_spawn_type == 0) {
                        spawner_launch_sphere(g_cfg.spawner.radius, g_cfg.spawner.mass, g_cfg.spawner.speed);
                    } else {
                        vector3 cube_spawn_position = vector3_addition(
                            main_camera_fov.position,
                            vector3_scaling(main_camera_fov.forward_vector, g_cfg.spawner.cube_extent + 1.0f));
                        spawner_launch_cube(cube_spawn_position,
                                            (vector3){g_cfg.spawner.cube_extent, g_cfg.spawner.cube_extent, g_cfg.spawner.cube_extent},
                                            g_cfg.spawner.cube_mass);
                    }
                    enter_spawn_interval_timer = 0.0f;
                }
            }
        }
        enter_previously_held = true;
    } else {
        enter_hold_timer = 0.0f;
        enter_previously_held = false;
    }
}
