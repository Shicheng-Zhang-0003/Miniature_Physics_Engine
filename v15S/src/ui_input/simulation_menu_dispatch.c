/* GTK4 port (v15S). The GTK3 body was removed 2026-09-29; git history
 * holds the v15R3 GTK3 engine. */
/* MFS_INCREMENT_SPLIT_4: Menu dispatch extracted from simulation.c.
* Owns: scene menu key handling, editor menu update, config menu update.
*/
#include "../mpe_engine.h"

void simulation_menu_dispatch (GtkWidget *parent_window) {
    /* Scene menu: 9 key bindings. All failures are reported via event_log
     * (visible in terminal dmesg) and stdout so save/load never fails
     * silently (QUAL-009). */
    if (main_inputs.menu_1_pressed) {
        int scene_rc = save_scene ("status/scene.dat");
        if (scene_rc == 1) {
            event_log_push (0, "scene saved to status/scene.dat");
        } else if (scene_rc == 2) {
            /* DESPOT-2026-10-04: return-2 is a durability warning, not full
             * success (bytes durable, dir-sync uncertain). Say so. */
            event_log_push (1, "scene saved but dir-sync uncertain (rc=2)");
            fprintf (stderr, "[menu] scene saved, directory sync uncertain\n");
        } else {
            event_log_push (2, "scene save FAILED (see stderr SVF*)");
            fprintf (stderr, "[menu] scene save failed\n");
        }
        main_inputs.menu_1_pressed = false;
        main_inputs.is_menu_open = false;
    }
    if (main_inputs.menu_2_pressed) {
        if (scene_loading ("status/scene.dat")) {
            event_log_push (0, "scene loaded from status/scene.dat");
        } else {
            event_log_push (2, "scene load FAILED (see stderr LDF*)");
            fprintf (stderr, "[menu] scene load failed\n");
        }
        editor_reset ();
        main_inputs.menu_2_pressed = false;
        main_inputs.is_menu_open = false;
    }
    if (main_inputs.menu_3_pressed) {
        scene_clear ();
        clear_selection ();
        contact_cache_clear (physics_world_get_primary ());
        editor_reset ();
        main_inputs.menu_3_pressed = false;
        main_inputs.is_menu_open = false;
    }
    if (main_inputs.menu_4_pressed) {
        /* DESPOT-2026-10-04: guarded — a mid-torture menu-4 save used to
         * publish torture as boot defaults. Refusal is loud, not silent. */
        if (mpe_config_save_guarded ("status/engine.cfg")) {
            event_log_push (0, "config saved to status/engine.cfg");
        } else {
            event_log_push (2, "config save REFUSED (torture live) or FAILED");
            fprintf (stderr, "[menu] config save refused (torture live) or failed\n");
        }
        main_inputs.menu_4_pressed = false;
        main_inputs.is_menu_open = false;
    }
    if (main_inputs.menu_5_pressed) {
        mpe_config_reset_defaults ();
        contact_cache_clear (physics_world_get_primary ());
        main_inputs.menu_5_pressed = false;
        main_inputs.is_menu_open = false;
    }
    if (main_inputs.menu_6_pressed) {
        main_inputs.menu_6_pressed = false;
        GApplication *app = g_application_get_default ();
        if (app) {
            g_application_quit (app);
        }
    }

    editor_update_menus (parent_window);
    config_menu_update (parent_window);
}
