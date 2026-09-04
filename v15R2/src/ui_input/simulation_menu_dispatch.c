/* MFS_INCREMENT_SPLIT_4: Menu dispatch extracted from simulation.c.
* Owns: scene menu key handling, editor menu update, config menu update.
*/
#include "../mpe_engine.h"

void simulation_menu_dispatch(GtkWidget *parent_window) {
    /* Scene menu: 9 key bindings */
    if (main_inputs.menu_1_pressed) {
        save_scene("status/scene.dat");
        main_inputs.menu_1_pressed = false;
        main_inputs.is_menu_open = false;
    }
    if (main_inputs.menu_2_pressed) {
        scene_loading("status/scene.dat");
        editor_reset();
        main_inputs.menu_2_pressed = false;
        main_inputs.is_menu_open = false;
    }
    if (main_inputs.menu_3_pressed) {
        scene_clear();
        clear_selection();
        contact_cache_clear(NULL);
        editor_reset();
        main_inputs.menu_3_pressed = false;
        main_inputs.is_menu_open = false;
    }
    if (main_inputs.menu_4_pressed) {
        mpe_config_save("status/engine.cfg");
        main_inputs.menu_4_pressed = false;
        main_inputs.is_menu_open = false;
    }
    if (main_inputs.menu_5_pressed) {
        mpe_config_reset_defaults();
        contact_cache_clear(NULL);
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
