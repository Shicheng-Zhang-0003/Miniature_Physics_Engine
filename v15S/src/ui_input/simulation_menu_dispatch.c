/* GTK4-PREP: GTK3 preserved under #else; GTK4 full port follows. */
#ifdef MPE_GTK4
/* MFS_INCREMENT_SPLIT_4: Menu dispatch extracted from simulation.c.
* Owns: scene menu key handling, editor menu update, config menu update.
*/
#include "../mpe_engine.h"

void simulation_menu_dispatch(GtkWidget *parent_window) {
    /* Scene menu: 9 key bindings. All failures are reported via event_log
     * (visible in terminal dmesg) and stdout so save/load never fails
     * silently (QUAL-009). */
    if (main_inputs.menu_1_pressed) {
        if (save_scene("status/scene.dat")) {
            event_log_push(0, "scene saved to status/scene.dat");
        } else {
            event_log_push(2, "scene save FAILED (see stderr SVF*)");
            fprintf(stderr, "[menu] scene save failed\n");
        }
        main_inputs.menu_1_pressed = false;
        main_inputs.is_menu_open = false;
    }
    if (main_inputs.menu_2_pressed) {
        if (scene_loading("status/scene.dat")) {
            event_log_push(0, "scene loaded from status/scene.dat");
        } else {
            event_log_push(2, "scene load FAILED (see stderr LDF*)");
            fprintf(stderr, "[menu] scene load failed\n");
        }
        editor_reset();
        main_inputs.menu_2_pressed = false;
        main_inputs.is_menu_open = false;
    }
    if (main_inputs.menu_3_pressed) {
        scene_clear();
        clear_selection();
        contact_cache_clear(physics_world_get_primary());
        editor_reset();
        main_inputs.menu_3_pressed = false;
        main_inputs.is_menu_open = false;
    }
    if (main_inputs.menu_4_pressed) {
        if (mpe_config_save("status/engine.cfg")) {
            event_log_push(0, "config saved to status/engine.cfg");
        } else {
            event_log_push(2, "config save FAILED");
            fprintf(stderr, "[menu] config save failed\n");
        }
        main_inputs.menu_4_pressed = false;
        main_inputs.is_menu_open = false;
    }
    if (main_inputs.menu_5_pressed) {
        mpe_config_reset_defaults();
        contact_cache_clear(physics_world_get_primary());
        main_inputs.menu_5_pressed = false;
        main_inputs.is_menu_open = false;
    }
    if (main_inputs.menu_6_pressed) {
        main_inputs.menu_6_pressed = false;
        GApplication *app = g_application_get_default();
        if (app) {
            g_application_quit(app);
        }
    }

    editor_update_menus(parent_window);
    config_menu_update(parent_window);
}

#else
/* MFS_INCREMENT_SPLIT_4: Menu dispatch extracted from simulation.c.
* Owns: scene menu key handling, editor menu update, config menu update.
*/
#include "../mpe_engine.h"

void simulation_menu_dispatch(GtkWidget *parent_window) {
    /* Scene menu: 9 key bindings. All failures are reported via event_log
     * (visible in terminal dmesg) and stdout so save/load never fails
     * silently (QUAL-009). */
    if (main_inputs.menu_1_pressed) {
        if (save_scene("status/scene.dat")) {
            event_log_push(0, "scene saved to status/scene.dat");
        } else {
            event_log_push(2, "scene save FAILED (see stderr SVF*)");
            fprintf(stderr, "[menu] scene save failed\n");
        }
        main_inputs.menu_1_pressed = false;
        main_inputs.is_menu_open = false;
    }
    if (main_inputs.menu_2_pressed) {
        if (scene_loading("status/scene.dat")) {
            event_log_push(0, "scene loaded from status/scene.dat");
        } else {
            event_log_push(2, "scene load FAILED (see stderr LDF*)");
            fprintf(stderr, "[menu] scene load failed\n");
        }
        editor_reset();
        main_inputs.menu_2_pressed = false;
        main_inputs.is_menu_open = false;
    }
    if (main_inputs.menu_3_pressed) {
        scene_clear();
        clear_selection();
        contact_cache_clear(physics_world_get_primary());
        editor_reset();
        main_inputs.menu_3_pressed = false;
        main_inputs.is_menu_open = false;
    }
    if (main_inputs.menu_4_pressed) {
        if (mpe_config_save("status/engine.cfg")) {
            event_log_push(0, "config saved to status/engine.cfg");
        } else {
            event_log_push(2, "config save FAILED");
            fprintf(stderr, "[menu] config save failed\n");
        }
        main_inputs.menu_4_pressed = false;
        main_inputs.is_menu_open = false;
    }
    if (main_inputs.menu_5_pressed) {
        mpe_config_reset_defaults();
        contact_cache_clear(physics_world_get_primary());
        main_inputs.menu_5_pressed = false;
        main_inputs.is_menu_open = false;
    }
    if (main_inputs.menu_6_pressed) {
        main_inputs.menu_6_pressed = false;
        gtk_main_quit();
    }

    editor_update_menus(parent_window);
    config_menu_update(parent_window);
}

#endif /* MPE_GTK4 */
