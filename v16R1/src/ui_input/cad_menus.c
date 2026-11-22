/* MIG WP8: CAD editor model menus. Page-driven numeric menus on 7/8/9:
 *   7 = selection/placement, 8 = add/transform, 9 = group/clipboard.
 * Numeric keys 1-9 drive page actions; CAD verbs are the WP8 model API
 * (cad/cad_editor.h); placement verbs are model-level first, GUI wiring
 * follows in later slices. */
#include "mpe_engine.h"
#include "editor.h" /* CAD_MENU_PAGE_* */
#include "cad_menus.h"
#include "cad_window.h"
#include "../cad/cad_editor.h"
#include <string.h>

/* Page-driver action logic (all verbs model-level; write-on-success). */

static bool cad_menus_placed_at_zero = false;

static cad_design *cad_menus_design (void) {
    return NULL; /* model-first: visibility gated by CAD window later; null-safe now */
}

/* ---- Page 7 (selection / placement) ---- */
static void cad_menus_sel_next (cad_design *d) {
    if (d && d -> count > 0) {
        int cur = d -> selected;
        int nxt = (cur < 0) ? 0 : ((cur + 1) >= d -> count ? 0 : cur + 1);
        cad_select (d, nxt);
    }
}
static void cad_menus_sel_prev (cad_design *d) {
    if (d && d -> count > 0) {
        int cur = d -> selected;
        int prv = (cur < 0) ? (d -> count - 1) : ((cur - 1) < 0 ? (d -> count - 1) : cur - 1);
        cad_select (d, prv);
    }
}
static void cad_menus_select_none (cad_design *d) { if (d) { cad_select (d, -1); } }
static void cad_menus_place_selected_at_zero (cad_design *d) {
    if (!d || d -> count <= 0) { return; }
    if (d -> selected < 0) { d -> selected = 0; }
    if (d -> selected >= d -> count) { d -> selected = d -> count - 1; }
    (void) cad_move (d, d -> selected, (vector3) {0.0f, 0.0f, 0.0f});
}
static void cad_menus_duplicate_at_zero (cad_design *d) {
    if (!d || d -> count <= 0) { return; }
    if (d -> selected < 0) { d -> selected = 0; }
    if (d -> selected >= d -> count) { d -> selected = d -> count - 1; }
    int idx = d -> selected;
    cad_design before = *d;
    if (cad_duplicate (d, idx, (vector3) {0.0f, 0.0f, 0.0f}) == cad_ok) {
        d -> selected = d -> count - 1; /* newest at the tail */
        (void) before;
    }
}
static void cad_menus_delete_selected (cad_design *d) {
    if (!d || d -> count <= 0) { return; }
    if (d -> selected < 0) { return; }
    cad_delete (d, d -> selected);
    d -> selected = (d -> selected < d -> count) ? d -> selected : -1;
}
static void cad_menus_move (cad_design *d, int keyval) {
    if (!d || d -> count <= 0) { return; }
    int axis = (keyval == GDK_KEY_J) ? 1 : ((keyval == GDK_KEY_K) ? 2 : 0);
    float sign = (keyval == GDK_KEY_G || keyval == GDK_KEY_J || keyval == GDK_KEY_L || keyval == GDK_KEY_U) ? -1.0f : 1.0f;
    vector3 dv = {0,0,0};
    if (axis == 0) { dv.x = sign; }
    else if (axis == 1) { dv.y = sign; }
    else { dv.z = sign; }
    (void) cad_move (d, d -> selected < 0 ? 0 : d -> selected, dv);
}

static void cad_menus_page7_key (cad_design *d, int keyval) {
    if (keyval == GDK_KEY_1) { cad_menus_sel_next (d); }
    else if (keyval == GDK_KEY_2) { cad_menus_sel_prev (d); }
    else if (keyval == GDK_KEY_3) { cad_menus_select_none (d); }
    else if (keyval == GDK_KEY_0) { cad_menus_place_selected_at_zero (d); }
    else if ((keyval == GDK_KEY_G) || (keyval == GDK_KEY_J) || (keyval == GDK_KEY_L)) { cad_menus_move (d, keyval); }
    else if (keyval == GDK_KEY_4) { cad_menus_delete_selected (d); }
    else if (keyval == GDK_KEY_5) { cad_menus_duplicate_at_zero (d); }
    else if (keyval == GDK_KEY_7) { cad_menus_set_level (0); }
}

/* ---- Page 8 (add / place) ---- */
static void cad_menus_page8_key (cad_design *d, int keyval) {
    if (!d) { return; }
    if (keyval == GDK_KEY_1) {
        if (!cad_menus_placed_at_zero) { (void) cad_add_sphere (d, 0.5f, (vector3) {0,0,0}); cad_menus_placed_at_zero = true; }
    }
    else if (keyval == GDK_KEY_2) {
        if (!cad_menus_placed_at_zero) { (void) cad_add_cube (d, (vector3) {0.5f,0.5f,0.5f}, (vector3) {0,0,0}); cad_menus_placed_at_zero = true; }
    }
    else if (keyval == GDK_KEY_3) {
        if (!cad_menus_placed_at_zero) { (void) cad_add_cylinder (d, 0.5f, 1.0f, (vector3) {0,0,0}); cad_menus_placed_at_zero = true; }
    }
    else if (keyval == GDK_KEY_4) {
        if (d && d -> count > 0) {
            if (d -> selected < 0) { d -> selected = 0; }
            int plc = d -> selected;
            (void) cad_mirror (d, 0);
            event_log_push (0, "CAD menu: mirror X (plane YZ through origin) on sel %d", plc);
        }
    }
    else if (keyval == GDK_KEY_5) { if (d && d -> count > 0) { (void) cad_align (d, 0, 0); event_log_push (0, "CAD menu: align min X on sel"); } }
    else if (keyval == GDK_KEY_6) { if (d && d -> count > 0) { (void) cad_align (d, 0, 2); event_log_push (0, "CAD menu: align max X on sel"); } }
    else if (keyval == GDK_KEY_7) { if (d && d -> count > 0) { (void) cad_align (d, 0, 1); event_log_push (0, "CAD menu: align center X on sel"); } }
    else if (keyval == GDK_KEY_8) { if (d && d -> count > 0) { (void) cad_distribute (d, 0); event_log_push (0, "CAD menu: distribute X on sel"); } }
    else if (keyval == GDK_KEY_9) { cad_menus_placed_at_zero = false; main_inputs.cad_menu_level = 9; }
    else if (keyval == GDK_KEY_0) { main_inputs.cad_menu_level = 0; }
}

/* ---- Page 9 (group / clipboard) ---- */
static void cad_menus_page9_key (cad_design *d, int keyval) {
    if (!d) { return; }
    if (keyval == GDK_KEY_1) { if (d -> count > 0) { int rc = cad_undo_multi (d); event_log_push (rc == cad_ok ? 0 : 1, "CAD menu: undo (rc=%d)", rc); } }
    else if (keyval == GDK_KEY_2) { if (d && d -> count > 0) { int rc = cad_redo_multi (d); event_log_push (rc == cad_ok ? 0 : 1, "CAD menu: redo (rc=%d)", rc); } }
    else if (keyval == GDK_KEY_3) { if (d && d -> count > 0 && d -> selected >= 0) { int rc = cad_group (d); event_log_push (rc >= 0 ? 0 : 1, "CAD menu: group (rc=%d)", rc); } }
    else if (keyval == GDK_KEY_4) { if (d && d -> count > 0 && d -> selected >= 0) { int rc = cad_ungroup (d); event_log_push (rc == cad_ok ? 0 : 1, "CAD menu: ungroup (rc=%d)", rc); } }
    else if (keyval == GDK_KEY_5) { if (d && d -> count > 0 && d -> selected >= 0) { int g = cad_group_members (d, d -> selected); event_log_push (0, "CAD menu: group members = %d", g); } }
    else if (keyval == GDK_KEY_6) { if (d && d -> count > 0 && d -> selected >= 0) { int rc = cad_copy (d); event_log_push (rc == cad_ok ? 0 : 1, "CAD menu: copy (rc=%d)", rc); } }
    else if (keyval == GDK_KEY_7) { if (d && d -> count > 0) { int rc = cad_paste (d, (vector3) {0,0,0}); event_log_push (rc == cad_ok ? 0 : 1, "CAD menu: paste at origin (rc=%d)", rc); } }
    else if (keyval == GDK_KEY_8) { if (d && d -> count >= 2 && d -> selected >= 0) { int a = d -> selected; int b = (a + 1) >= d -> count ? (a - 1) : (a + 1); float dist = cad_measure_distance (d, a, b); event_log_push (0, "CAD menu: measure distance(%d,%d) = %.3f m", a, b, dist); } }
    else if (keyval == GDK_KEY_9) {
        cad_menus_placed_at_zero = false;
        if (d && d -> count > 0 && d -> selected >= 0) {
            int rc = cad_set_mass (d, d -> selected, mig_mass_mode_density, CAD_DEFAULT_DENSITY);
            event_log_push (rc == cad_ok ? 0 : 1, "CAD menu: set default density mass (rc=%d)", rc);
        }
    }
    else if (keyval == GDK_KEY_0) { main_inputs.cad_menu_level = 0; }
}

void cad_menus_update (void) {
    if (main_inputs.cad_menu_level != CAD_MENU_PAGE_SELECTION &&
        main_inputs.cad_menu_level != CAD_MENU_PAGE_ADD &&
        main_inputs.cad_menu_level != CAD_MENU_PAGE_GROUP_CLIPBOARD) {
        return;
    }
    if (editor_dialog_is_active ()) { return; }
    cad_design *d = cad_menus_design ();
    int parts = (d) ? d -> count : 0;
    int sel = (d) ? d -> selected : -1;
    if (main_inputs.cad_menu_level == CAD_MENU_PAGE_SELECTION) {
        printf ("CAD menu 7 — Selection/placement | parts=%d selected=%d\n  [1] next  [2] prev  [3] clear  [0] place@0  [G/J/L] move X/Y/Z  [4] del  [5] dup@0  [7] close\n", parts, sel);
    } else if (main_inputs.cad_menu_level == CAD_MENU_PAGE_ADD) {
        printf ("CAD menu 8 — Add/place | parts=%d selected=%d\n  [1] sphere  [2] cube  [3] cylinder  [4] mirror X  [5] align minX  [6] align maxX  [7] align centerX  [8] distribute X  [9] clip/props  [0] root\n", parts, sel);
    } else {
        printf ("CAD menu 9 — Group/clipboard | parts=%d selected=%d\n  [1] undo  [2] redo  [3] group  [4] ungroup  [5] members  [6] copy  [7] paste@0  [8] measure  [9] density-mass  [0] root\n", parts, sel);
    }
}

void cad_menus_set_level (int level) {
    if (level != 0 && level != CAD_MENU_PAGE_SELECTION && level != CAD_MENU_PAGE_ADD && level != CAD_MENU_PAGE_GROUP_CLIPBOARD) { return; }
    if (level == 0) { cad_menus_placed_at_zero = false; }
    if (level == CAD_MENU_PAGE_SELECTION || level == CAD_MENU_PAGE_ADD) {
        cad_design *d = cad_menus_design ();
        if (d && d -> selected >= d -> count) { d -> selected = d -> count - 1; }
    }
    main_inputs.cad_menu_level = level;
    cad_menus_update ();
}

bool cad_menus_key_press (int keyval) {
    if (editor_dialog_is_active ()) { return false; }
    if (main_inputs.cad_menu_level == CAD_MENU_PAGE_SELECTION) { cad_menus_page7_key (cad_menus_design (), keyval); }
    else if (main_inputs.cad_menu_level == CAD_MENU_PAGE_ADD) { cad_menus_page8_key (cad_menus_design (), keyval); }
    else if (main_inputs.cad_menu_level == CAD_MENU_PAGE_GROUP_CLIPBOARD) { cad_menus_page9_key (cad_menus_design (), keyval); }
    cad_menus_update ();
    return true;
}
