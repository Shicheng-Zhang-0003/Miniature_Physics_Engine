/* GTK4 port (v15S). The GTK3 body was removed 2026-09-29; git history
 * holds the v15R3 GTK3 engine. */
/* MFS_INCREMENT_SPLIT_4: Menu dispatch extracted from simulation.c.
* Owns: scene menu key handling, editor menu update, config menu update.
*/
#include "../mpe_engine.h"
#include "../core/mpe_loader.h"
#include "../core/physics_world.h"
/* Scene-replacement fleet reset (DESPOT-2026-10-14): scene_clear/load wipe
 * or replace the body pool without telling the ftc-fleet module, whose
 * robot count/balls/bucket/BIOBUZZ state would otherwise go stale ("one
 * robot already" after 9 3; stranger-driving after load). Resolved
 * dynamically — the engine never links FTC (same pattern as term_ftc.c);
 * absent bundle = no fleet = nothing to do. The fleet pre_step self-heal
 * covers any path missed here (e.g. stepping paused). Mirrored in
 * term_fs.c (rm -rf) and term_admin.c (mount). */
static void fleet_clear_on_scene_replace (void) {
    typedef int (*clear_fn) (struct physics_world *);
    clear_fn fn = (clear_fn) mpe_loader_symbol ("mfs-simulator", "ftc_fleet_clear");
    if (!fn) { fn = (clear_fn) mpe_loader_symbol ("ecosystem/mfs/plugins/mpe_ftc.so", "ftc_fleet_clear"); }
    if (fn) { fn (physics_world_get_primary ()); }
}
void simulation_menu_dispatch (GtkWidget *parent_window) {
    /* Scene menu: 9 key bindings. All failures are reported via event_log
     * (visible in terminal dmesg) and stdout so save/load never fails
     * silently (QUAL-009). */
    if (main_inputs.menu_1_pressed) {
        int scene_rc = save_scene ("status/scene.dat");
        if (scene_rc == 1) { event_log_push (0, "scene saved to status/scene.dat"); } else if (scene_rc == 2) {
            /* DESPOT-2026-10-04: return-2 is a durability warning, not full
             * success (bytes durable, dir-sync uncertain). Say so. */
            event_log_push (1, "scene saved but dir-sync uncertain (rc=2)");
            fprintf (stderr, "[menu] scene saved, directory sync uncertain\n");
        } else {
            event_log_push (2, "scene save FAILED (see stderr SVF*)");
            fprintf (stderr, "[menu] scene save failed\n");
        } main_inputs.menu_1_pressed = false;
        main_inputs.is_menu_open = false;
    }
    if (main_inputs.menu_2_pressed) {
        if (scene_loading ("status/scene.dat")) {
            event_log_push (0, "scene loaded from status/scene.dat");
            fleet_clear_on_scene_replace ();
        } else {
            event_log_push (2, "scene load FAILED (see stderr LDF*)");
            fprintf (stderr, "[menu] scene load failed\n");
        } editor_reset ();
        main_inputs.menu_2_pressed = false;
        main_inputs.is_menu_open = false;
    }
    if (main_inputs.menu_3_pressed) {
        scene_clear ();
        clear_selection ();
        contact_cache_clear (physics_world_get_primary ());
        fleet_clear_on_scene_replace ();
        editor_reset ();
        main_inputs.menu_3_pressed = false;
        main_inputs.is_menu_open = false;
    }
    if (main_inputs.menu_4_pressed) {
        /* DESPOT-2026-10-04: guarded — a mid-torture menu-4 save used to
         * publish torture as boot defaults. Refusal is loud, not silent. */
        if (mpe_config_save_guarded ("status/engine.cfg")) { event_log_push (0, "config saved to status/engine.cfg"); } else {
            event_log_push (2, "config save REFUSED (torture live) or FAILED");
            fprintf (stderr, "[menu] config save refused (torture live) or failed\n");
        } main_inputs.menu_4_pressed = false;
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
        if (app) { g_application_quit (app); }
    } editor_update_menus (parent_window);
    config_menu_update (parent_window);
    module_config_menu_update (parent_window);
}
