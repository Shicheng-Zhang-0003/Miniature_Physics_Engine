/* MPE_TASK_35_CONFIG_MENU_IMPL_BEGIN */
/* MPE_TASK_35_FIX4_MENU_EXCLUSIVITY */
#include "../mpe_engine.h"
#include "config_menu.h"
#include <stdio.h>
#include <string.h>

static int config_menu_level = 0;
static int config_menu_active_category = -1;
static int config_menu_selected_param = -1;

bool config_menu_is_open (void) {
    return config_menu_level > 0;
}

void config_menu_level_force_open (void) {
    config_menu_level = 1;
    config_menu_active_category = -1;
    config_menu_selected_param = -1;
}

void config_menu_close (void) {
    config_menu_level = 0;
    config_menu_active_category = -1;
    config_menu_selected_param = -1;
}

void config_menu_key_press (int key_number) {
    if (config_menu_level == 0) {return;}
    /* Key 6 always closes the config menu (toggle), never a category. */
    if (key_number == 6) {config_menu_close (); return;}
    if (config_menu_level == 1) {
        /* Page 1: 1-5 + 7-9 select categories, 0 = more. (6 = close) */
        if (key_number == 1) {config_menu_active_category = cat_world; config_menu_level = 10 + cat_world;}
        else if (key_number == 2) {config_menu_active_category = cat_timestep; config_menu_level = 10 + cat_timestep;}
        else if (key_number == 3) {config_menu_active_category = cat_sleep; config_menu_level = 10 + cat_sleep;}
        else if (key_number == 4) {config_menu_active_category = cat_solver; config_menu_level = 10 + cat_solver;}
        else if (key_number == 5) {config_menu_active_category = cat_depenetration; config_menu_level = 10 + cat_depenetration;}
        else if (key_number == 7) {config_menu_active_category = cat_broadphase; config_menu_level = 10 + cat_broadphase;}
        else if (key_number == 8) {config_menu_active_category = cat_joints; config_menu_level = 10 + cat_joints;}
        else if (key_number == 9) {config_menu_active_category = cat_boundary; config_menu_level = 10 + cat_boundary;}
        else if (key_number == 0) {config_menu_level = 2;}
        config_menu_selected_param = -1;
    } else if (config_menu_level == 2) {
        /* Page 2: 1-5 categories, 7 save, 8 reset, 9 back, 0 close. (6 = close) */
        if (key_number == 1) {config_menu_active_category = cat_spawner; config_menu_level = 10 + cat_spawner;}
        else if (key_number == 2) {config_menu_active_category = cat_body_defaults; config_menu_level = 10 + cat_body_defaults;}
        else if (key_number == 3) {config_menu_active_category = cat_camera; config_menu_level = 10 + cat_camera;}
        else if (key_number == 4) {config_menu_active_category = cat_render; config_menu_level = 10 + cat_render;}
        else if (key_number == 5) {config_menu_active_category = cat_ui; config_menu_level = 10 + cat_ui;}
        else if (key_number == 7) {mpe_config_save ("status/engine.cfg");}
        else if (key_number == 8) {mpe_config_reset_defaults (); contact_cache_clear ();}
        else if (key_number == 9) {config_menu_level = 1;}
        else if (key_number == 0) {config_menu_level = 1;} /* MPE_TASK_35_FIX5 */
        config_menu_selected_param = -1;
    } else if (config_menu_level >= 10) {
        /* Param list: number selects a param to edit, 0 = back. */
        int category = config_menu_level - 10;
        size_t param_count = mpe_config_count_by_category ((param_category) category);
        if ((key_number >= 1) && ((size_t) key_number <= param_count)) {
            config_menu_selected_param = key_number - 1;
        } else if (key_number == 0) {
            config_menu_level = (category <= cat_boundary) ? 1 : 2;
            config_menu_active_category = -1;
            config_menu_selected_param = -1;
        }
    }
}

void config_menu_update (GtkWidget *parent_window) {
    if (config_menu_level < 10) {return;}
    if (config_menu_selected_param < 0) {return;}
    int category = config_menu_level - 10;
    static const mpe_param *category_params [64];
    size_t param_count = mpe_config_get_by_category ((param_category) category, category_params, 64);
    if ((size_t) config_menu_selected_param >= param_count) {
        config_menu_selected_param = -1;
        return;
    }
    const mpe_param *param = category_params [config_menu_selected_param];
    if ((param -> debug_only) && (!main_inputs.is_debug_mode_active)) {
        config_menu_selected_param = -1;
        return;
    }
    float current_value = 0.0f;
    if (param -> type == p_float) {current_value = *(float *) param -> storage;}
    else if (param -> type == p_int) {current_value = (float) (*(int *) param -> storage);}
    else if (param -> type == p_bool) {current_value = (*(bool *) param -> storage) ? 1.0f : 0.0f;}
    float new_value = open_numerical_input_dialog (parent_window, param -> display, current_value);
    if (new_value < (float) param -> min) {new_value = (float) param -> min;}
    if (new_value > (float) param -> max) {new_value = (float) param -> max;}
    if (param -> type == p_float) {*(float *) param -> storage = new_value;}
    else if (param -> type == p_int) {*(int *) param -> storage = (int) new_value;}
    else if (param -> type == p_bool) {*(bool *) param -> storage = (new_value != 0.0f);}
    if ((category == cat_solver) || (category == cat_timestep) || (category == cat_depenetration)) {
        contact_cache_clear ();
    }
    config_menu_selected_param = -1;
}

void config_menu_render (char *buffer, size_t buffer_size) {
    if (config_menu_level == 0) {buffer [0] = '\0'; return;}
    if (config_menu_level == 1) {
        snprintf (buffer, buffer_size,
            "-- Config Menu (Page 1) --\n"
            "1: World (%zu)\n"
            "2: Timestep (%zu)\n"
            "3: Sleep (%zu)\n"
            "4: Solver (%zu)\n"
            "5: Depenetration (%zu)\n"
            "7: Broadphase (%zu)\n"
            "8: Joints (%zu)\n"
            "9: Boundary (%zu)\n"
            "0: More...\n"
            "6: Close",
            mpe_config_count_by_category (cat_world),
            mpe_config_count_by_category (cat_timestep),
            mpe_config_count_by_category (cat_sleep),
            mpe_config_count_by_category (cat_solver),
            mpe_config_count_by_category (cat_depenetration),
            mpe_config_count_by_category (cat_broadphase),
            mpe_config_count_by_category (cat_joints),
            mpe_config_count_by_category (cat_boundary));
    } else if (config_menu_level == 2) {
        snprintf (buffer, buffer_size,
            "-- Config Menu (Page 2) --\n"
            "1: Spawner (%zu)\n"
            "2: Body Defaults (%zu)\n"
            "3: Camera (%zu)\n"
            "4: Render (%zu)\n"
            "5: UI (%zu)\n"
            "7: Save Config\n"
            "8: Reset Defaults\n"
            "9: Back\n"
            "0: Back\n"
            "6: Close",
            mpe_config_count_by_category (cat_spawner),
            mpe_config_count_by_category (cat_body_defaults),
            mpe_config_count_by_category (cat_camera),
            mpe_config_count_by_category (cat_render),
            mpe_config_count_by_category (cat_ui));
    } else if (config_menu_level >= 10) {
        int category = config_menu_level - 10;
        static const mpe_param *category_params [64];
        size_t param_count = mpe_config_get_by_category ((param_category) category, category_params, 64);
        size_t offset = 0;
        offset += snprintf (buffer + offset, buffer_size - offset,
            "-- %s --\n", mpe_config_category_name ((param_category) category));
        for (size_t i = 0; (i < param_count) && (offset < buffer_size - 64); i++) {
            const mpe_param *p = category_params [i];
            float val = 0.0f;
            if (p -> type == p_float) {val = *(float *) p -> storage;}
            else if (p -> type == p_int) {val = (float) (*(int *) p -> storage);}
            else if (p -> type == p_bool) {val = (*(bool *) p -> storage) ? 1.0f : 0.0f;}
            const char *debug_tag = (p -> debug_only) ? " [D]" : "";
            if (p -> type == p_int) {
                offset += snprintf (buffer + offset, buffer_size - offset,
                    "%zu: %s = %d%s\n", i + 1, p -> display, (int) val, debug_tag);
            } else {
                offset += snprintf (buffer + offset, buffer_size - offset,
                    "%zu: %s = %.4f%s\n", i + 1, p -> display, val, debug_tag);
            }
        }
        snprintf (buffer + offset, buffer_size - offset, "0: Back | 6: Close");
    }
}
/* MPE_TASK_35_CONFIG_MENU_IMPL_END */
