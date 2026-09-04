#include "mpe_engine.h"
#include "robotics/gui_robot_registry.h" /* MFS_GUI_BRIDGE */
#include "core/validation_report.h"
#include "physics/depenetration.h"
#include "core/simulation_camera.h"
#include "core/simulation_physics_loop.h"
#include "core/long_run_validation.h"

#include "ui_input/simulation_dispatch.h"
#include <gtk/gtk.h>
#include <stdbool.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <unistd.h> /* MPE_TASK_39 access() */
//World Status right now
frame_timer main_timer;
rigidbody *obj_per_scene = NULL;
int object_count = 0;
int object_capacity = 0;
//World Physics Globals

int debug_last_object_count = 0;
int debug_last_broadphase_pair_count = 0;
int debug_last_manifold_count = 0;
float debug_last_frame_time = 0.0f;
/* MPE_TASK_12_SLEEPING_COUNT_GLOBAL_BEGIN */
int debug_last_sleeping_object_count = 0;
/* MPE_TASK_12_SLEEPING_COUNT_GLOBAL_END */
/* MPE_TASK_09_MANIFOLD_OVERFLOW_COUNTER_BEGIN */
int debug_last_manifold_overflow_count = 0;
/* MPE_TASK_09_MANIFOLD_OVERFLOW_COUNTER_END */




/* MPE_TASK_20A_DEPENETRATION_HELPERS_BEGIN */
/* MPE_TASK_20A_DEPENETRATION_HELPERS_END */
gboolean physics_step_increment(gpointer user_data_pointer) {
    GtkWidget *parent_window = NULL;
    if (user_data_pointer) {
        parent_window = gtk_widget_get_toplevel(GTK_WIDGET(user_data_pointer));
    }

    /* Guard checks */
    if (editor_dialog_is_active()) { return TRUE; }
    if (physics_halt_tick_update()) {
        gtk_widget_queue_draw(GTK_WIDGET(user_data_pointer));
        overlay_update();
        return TRUE;
    }

    /* Mode watch */
    static bool a3_previous_debug_mode_state = false;
    static bool a3_debug_mode_watch_ready = false;
    if (!a3_debug_mode_watch_ready) {
        a3_debug_mode_watch_ready = true;
        a3_previous_debug_mode_state = main_inputs.is_debug_mode_active;
    } else if (main_inputs.is_debug_mode_active != a3_previous_debug_mode_state) {
        a3_previous_debug_mode_state = main_inputs.is_debug_mode_active;
        debug_terminal_sync_mode();
    }

    /* Change rate adjustment */
    if (main_inputs.is_debug_mode_active) {
        if (main_inputs.left_arrow_pressed) { g_cfg.ui.change_rate_debug -= 0.01f; main_inputs.left_arrow_pressed = false; }
        if (main_inputs.right_arrow_pressed) { g_cfg.ui.change_rate_debug += 0.01f; main_inputs.right_arrow_pressed = false; }
    } else {
        if (main_inputs.left_arrow_pressed) { g_cfg.ui.change_rate_game -= 0.2f; main_inputs.left_arrow_pressed = false; }
        if (main_inputs.right_arrow_pressed) { g_cfg.ui.change_rate_game += 0.2f; main_inputs.right_arrow_pressed = false; }
    }

    /* Status dir + frame timer */
    static int status_dir_checked = 0;
    if (!status_dir_checked) { mkdir("status", 0755); status_dir_checked = 1; }
    frame_timer_update(&main_timer);
    float frame_delta_time = main_timer.delta_time;
    debug_last_frame_time = frame_delta_time;

    /* Camera + character */
    simulation_camera_tick(frame_delta_time);

    /* Input dispatch (mouse/keyboard bindings, menus, spawn) */
    /* NOTE: input dispatch still inline for now — extract in next pass */
    simulation_input_dispatch(parent_window);

    /* Menu handling */
    simulation_menu_dispatch(parent_window);
    editor_update_menus(parent_window);
    config_menu_update(parent_window);

    /* THE PHYSICS LOOP */
    simulation_physics_tick(frame_delta_time);

    /* MFS_GUI_BRIDGE_TICK: Robot physics tick only.
       Drive input is handled in simulation_input_dispatch.c (G/V/B/N/C/H). */
    if (gui_robot_get_count() > 0) {
        gui_robot_tick(frame_delta_time);
    }
    /* MFS_GUI_BRIDGE_TICK_END */

    /* Post-physics bookkeeping */
    gtk_widget_queue_draw(GTK_WIDGET(user_data_pointer));
    int a3_sleeping_object_count = 0;
    for (int sleep_count_index = 0; sleep_count_index < object_count; sleep_count_index++) {
        if (obj_per_scene[sleep_count_index].is_sleeping) { a3_sleeping_object_count++; }
    }
    debug_last_sleeping_object_count = a3_sleeping_object_count;
    debug_last_object_count = object_count;
    long_run_validation_tick_update();
    overlay_update();
    return TRUE;
}
