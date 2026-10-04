/* GTK4 port (v15S). The GTK3 body was removed 2026-09-29; git history
 * holds the v15R3 GTK3 engine. */
/* MFS_INCREMENT_SPLIT_3: Input dispatch extracted from simulation.c.
* Owns: mouse/keyboard bindings, keyboard-only actions, test keys, spawn gun.
*/
#include "../mpe_engine.h"
#include "../core/validation_report.h" /* MFS_INCREMENT_SPLIT: validation_report_print */
#include "../core/long_run_validation.h"
void simulation_input_dispatch (GtkWidget *parent_window) {
    /* Mouse, Escape, E, F key bindings */
    if (main_inputs.escape_key_pressed) {
        if (main_inputs.is_mouse_locked) {
            mouse_lock_disable (parent_window);
            main_inputs.is_mouse_locked = false;
        }
        main_inputs.escape_key_pressed = false;
    }
    if (main_inputs.right_mouse_button_clicked) {
        selector_ray_tracing ();
        main_inputs.right_mouse_button_clicked = false;
    }
    if (main_inputs.middle_mouse_button_clicked) {
        if (selected_object >= 0) {
            scene_remove_object_by_index (selected_object);
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
            /* Guarded: with no selection E must not disturb the config
             * menu (previously closed unconditionally). */
            config_menu_close ();
        }
        /* DESPOT-2026-09-29: this latch was never cleared, so one E press
         * flipped object_menu_level on EVERY subsequent tick (the menu
         * strobed at 62.5 Hz indefinitely) until alt-tab cleared it via the
         * focus-out handler. on_key_released clears w/a/s/d/r/i/j/k/l/space/
         * shift/return but never `e`. The sibling F case two lines below
         * already does the right thing. */
        main_inputs.e_key_pressed = false;
    }
    if (main_inputs.f_key_pressed) {
        if (selected_object >= 0) {
            selector_apply_force_impulse (250.0f);
        }
        main_inputs.f_key_pressed = false;
    }
    /* Keyboard-only actions (R select; Delete/M keybinds removed) */
    if (main_inputs.r_key_pressed) {
        if (main_inputs.is_debug_mode_active) {
            selector_ray_tracing ();
        }
        main_inputs.r_key_pressed = false;
    }
    /* Test key bindings (F5-F11) */
    if (main_inputs.stability_test_pressed) {
        scene_spawn_stability_stack ();
        main_inputs.stability_test_pressed = false;
    }
    if (main_inputs.sleep_wake_test_pressed) {
        scene_spawn_sleep_wake_test ();
        main_inputs.sleep_wake_test_pressed = false;
    }
    if (main_inputs.editor_torture_pressed) {
        scene_editor_torture_test ();
        main_inputs.editor_torture_pressed = false;
    }
    if (main_inputs.spawn_stress_pressed) {
        scene_spawn_stress_test ();
        main_inputs.spawn_stress_pressed = false;
    }
    if (main_inputs.validation_report_pressed) {
        validation_report_print ();
        main_inputs.validation_report_pressed = false;
    }
    if (main_inputs.debug_terminal_pressed) {
        if (main_inputs.is_debug_mode_active) {
            debug_terminal_open (parent_window);
        }
        main_inputs.debug_terminal_pressed = false;
    }
    if (main_inputs.long_run_validation_pressed) {
        /* DESPOT-2026-10-04: flags BEFORE start, so the start-time config
         * dump is labelled with the right mode (old order dumped first,
         * labelled second). */
        long_run_validation_restore_config = 0;
        long_run_validation_is_torture = 0;
        scene_spawn_long_run_validation ();
        long_run_validation_start (a3_long_run_validation_ticks);
        /* DESPOT-2026-10-01: F10 never dirties the config (no randomization;
         * floor friction set by the scene is the intended live value), so
         * there is nothing to restore. Old code set restore_config=1, which
         * made a plain F10 attempt a backup load and print a bogus
         * "torture values remain live" warning when no backup existed. */
        long_run_validation_restore_config = 0;
        long_run_validation_is_torture = 0;
        main_inputs.long_run_validation_pressed = false;
    }
    if (main_inputs.config_torture_pressed) {
        /* DESPOT-2026-10-01: two stuck-in-torture bugs closed here. (1) A
         * second F11 while a run is active re-saved the ALREADY-TORTURED
         * config over the clean backup, so the restore reloaded torture and
         * the engine never came back. Refuse re-entry loudly instead.
         * (2) The save return was ignored: a failed backup plus a failed
         * restore left torture live with no honest fallback (and the user
         * deleting engine.cfg rightly suspected it — the backup is a
         * separate file, but a missing backup hit the same path). */
        if (long_run_validation_active) {
            printf ("[A3] Config torture already running; ignoring re-press (backup kept clean)\n");
            main_inputs.config_torture_pressed = false;
        } else {
            /* DESPOT-2026-10-04: snapshot BEFORE randomizing (memory cannot
             * go missing like the CWD-relative backup file can), flags
             * BEFORE start (so the start dump is labelled torture), backup
             * failure to stderr+event log (a headless operator must see it).
             * The 2026-10-04 engine.cfg incident was a mid-torture exit with
             * none of these holding. */
            long_run_validation_snapshot_clean ();
            if (!mpe_config_save ("status/engine.cfg.backup")) {
                fprintf (stderr, "[A3] WARNING: config backup failed; F11 will fall back to "
                                 "memory snapshot, else compiled defaults on restore\n");
                event_log_push (2, "F11 backup FAILED; memory snapshot armed instead");
            }
            long_run_validation_restore_config = 1;
            long_run_validation_is_torture = 1;
            scene_spawn_config_torture_test ();
            long_run_validation_start (a3_long_run_validation_ticks);
            main_inputs.config_torture_pressed = false;
        }
    }
    /* Spawn gun (Enter hold) */
    static float enter_hold_timer = 0.0f;
    static float enter_spawn_interval_timer = 0.0f;
    static bool enter_previously_held = false;
    if ((main_inputs.enter_spawn_held) && (!editor_dialog_is_active ()) && (!main_inputs.is_menu_open) &&
        (main_inputs.spawner_menu_level == 0) && (main_inputs.velocity_menu_level == 0) &&
        (main_inputs.object_menu_level == 0)) {
        if (!enter_previously_held) {
            if (main_inputs.current_spawn_type == 0) {
                spawner_launch_sphere (g_cfg.spawner.radius, g_cfg.spawner.mass, g_cfg.spawner.speed);
            } else if (main_inputs.current_spawn_type == 1) {
                vector3 cube_spawn_position =
                    vector3_addition (main_camera_fov.position, vector3_scaling (main_camera_fov.forward_vector,
                                                                                 g_cfg.spawner.cube_extent + 1.0f));
                spawner_launch_cube (
                    cube_spawn_position,
                    (vector3){g_cfg.spawner.cube_extent, g_cfg.spawner.cube_extent, g_cfg.spawner.cube_extent},
                    g_cfg.spawner.cube_mass);
            } else {
                spawner_launch_cylinder (g_cfg.spawner.cyl_radius, g_cfg.spawner.cyl_half_length,
                                         g_cfg.spawner.cyl_mass, g_cfg.spawner.speed);
            }
            enter_hold_timer = 0.0f;
            enter_spawn_interval_timer = 0.0f;
        } else {
            enter_hold_timer += main_timer.delta_time;
            if (enter_hold_timer > g_cfg.ui.enter_spawn_delay) {
                enter_spawn_interval_timer += main_timer.delta_time;
                if (enter_spawn_interval_timer >= g_cfg.ui.enter_spawn_interval) {
                    if (main_inputs.current_spawn_type == 0) {
                        spawner_launch_sphere (g_cfg.spawner.radius, g_cfg.spawner.mass, g_cfg.spawner.speed);
                    } else if (main_inputs.current_spawn_type == 1) {
                        vector3 cube_spawn_position = vector3_addition (
                            main_camera_fov.position,
                            vector3_scaling (main_camera_fov.forward_vector, g_cfg.spawner.cube_extent + 1.0f));
                        spawner_launch_cube (
                            cube_spawn_position,
                            (vector3){g_cfg.spawner.cube_extent, g_cfg.spawner.cube_extent, g_cfg.spawner.cube_extent},
                            g_cfg.spawner.cube_mass);
                    } else {
                        spawner_launch_cylinder (g_cfg.spawner.cyl_radius, g_cfg.spawner.cyl_half_length,
                                                 g_cfg.spawner.cyl_mass, g_cfg.spawner.speed);
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
