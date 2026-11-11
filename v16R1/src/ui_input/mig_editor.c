/* MIG WP4 4.4: GTK editor panel for MIG object specs. See mig_editor.h.
 *
 * Window recipe cloned from ui_input/resize_panel.c (plain GtkWindow,
 * transient for the main window, CSS provider keyed on the widget name,
 * Esc closes, destroy handler nulls the globals, never modal).
 *
 * The editor edits a copy of a store spec. Save re-parses the edited text
 * through mig_spec_parse_file and stages it back into the spec store
 * (write-on-success: a rejected parse leaves the store unchanged). No
 * editor-only truth exists — the spec text is the single source of truth.
 *
 * Ground-plane translate gizmo: while the editor is open and a part is
 * selected, clicking on the floor (Y = the part's current height) starts
 * an X/Z translate drag; the part follows the ray-plane intersection.
 * Camera motion is suppressed while the gizmo is active (input dispatch
 * consults mig_editor_gizmo_active()). */
#include "mig_editor.h"
#include "../mpe_engine.h"
#include "term_priv.h"
#include "object_selector.h"
#include "mouse_lock.h"
#include <gtk/gtk.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

/* ---- widget handles -------------------------------------------------- */

static GtkWidget *editor_window = NULL;
static GtkWidget *editor_spec_dropdown = NULL;
static GtkWidget *editor_part_dropdown = NULL;
static GtkWidget *editor_save_btn = NULL;
static GtkWidget *editor_add_part_btn = NULL;
static GtkWidget *editor_remove_part_btn = NULL;
static GtkWidget *editor_mass_label = NULL;
static GtkWidget *editor_com_label = NULL;

/* Per-part field widgets, rebuilt when a part is selected. */
static GtkWidget *editor_type_dropdown = NULL;

enum {
    EDIT_FIELD_RADIUS = 0,   /* sphere radius / cylinder radius */
    EDIT_FIELD_HALF_LENGTH,  /* cylinder half length */
    EDIT_FIELD_HX,           /* cube half x */
    EDIT_FIELD_HY,           /* cube half y */
    EDIT_FIELD_HZ,           /* cube half z */
    EDIT_FIELD_PX,           /* position x */
    EDIT_FIELD_PY,           /* position y */
    EDIT_FIELD_PZ,           /* position z */
    EDIT_FIELD_QW,           /* orientation w */
    EDIT_FIELD_QX,           /* orientation x */
    EDIT_FIELD_QY,           /* orientation y */
    EDIT_FIELD_QZ,           /* orientation z */
    EDIT_FIELD_MU_S,         /* material static friction */
    EDIT_FIELD_MU_K,         /* material kinetic friction */
    EDIT_FIELD_REST,         /* material restitution */
    EDIT_FIELD_MASS_DENSITY, /* mass or density value */
    EDIT_FIELD_COLOUR_R,     /* colour r */
    EDIT_FIELD_COLOUR_G,     /* colour g */
    EDIT_FIELD_COLOUR_B,     /* colour b */
    EDIT_FIELD_COUNT
};

static GtkWidget *editor_field_entries [EDIT_FIELD_COUNT] = {NULL};

/* ---- editor state ---------------------------------------------------- */

static char editor_spec_name [MIG_SPEC_NAME_LEN] = {0};
static bool editor_has_spec = false;
static mig_spec editor_spec;          /* working copy of the store spec */
static int editor_selected_part = -1; /* index into editor_spec.parts */
static bool editor_dirty = false;     /* working copy differs from store */
static bool editor_gizmo_active = false;
static int editor_gizmo_part = -1;

/* Forward declarations (called before definition). */
static void editor_update_readouts (void);
static void editor_rebuild_ui (void);
static void editor_rebuild_fields (void);
static bool editor_load_spec_by_store_idx (int store_idx);

/* ---- spec store access ----------------------------------------------- */

/* Refresh the spec dropdown from the store. Keeps the current selection
 * if it still exists, otherwise selects the first spec (or none). */
static void editor_refresh_dropdown (void) {
    GtkStringList *list = gtk_string_list_new (NULL);
    int keep_idx = -1;
    int count = mig_spec_store_count ();
    for (int i = 0; i < count; i++) {
        const char *name = mig_spec_store_name_at (i);
        gtk_string_list_append (list, name);
        if (editor_has_spec && strcmp (name, editor_spec_name) == 0) {
            keep_idx = i;
        }
    }
    gtk_drop_down_set_model (GTK_DROP_DOWN (editor_spec_dropdown), G_LIST_MODEL (list));
    g_object_unref (list);
    if (keep_idx >= 0) {
        gtk_drop_down_set_selected (GTK_DROP_DOWN (editor_spec_dropdown), keep_idx);
    } else if (count > 0) {
        gtk_drop_down_set_selected (GTK_DROP_DOWN (editor_spec_dropdown), 0);
        editor_load_spec_by_store_idx (0);
    } else {
        gtk_drop_down_set_selected (GTK_DROP_DOWN (editor_spec_dropdown), GTK_INVALID_LIST_POSITION);
        editor_has_spec = false;
    }
}

/* Copy the store spec at `store_idx` into the editor working copy. */
static bool editor_load_spec_by_store_idx (int store_idx) {
    if (store_idx < 0) { return false; }
    const mig_spec *src = mig_spec_store_spec_at (store_idx);
    if (!src) { return false; }
    editor_spec = *src;
    snprintf (editor_spec_name, sizeof (editor_spec_name), "%s", src -> name);
    editor_has_spec = true;
    editor_selected_part = (src -> count > 0) ? 0 : -1;
    editor_dirty = false;
    return true;
}

/* Called when the user picks a different spec from the dropdown. */
static void editor_on_spec_changed (GtkDropDown *dropdown, gpointer user_data) {
    (void) user_data;
    guint idx = gtk_drop_down_get_selected (GTK_DROP_DOWN (dropdown));
    if (idx == GTK_INVALID_LIST_POSITION) { editor_has_spec = false; editor_rebuild_ui (); return; }
    /* Map dropdown index to store index (they're in the same order). */
    int seen = 0;
    int store_idx = -1;
    int count = mig_spec_store_count ();
    for (int i = 0; i < count; i++) {
        if (seen == (int) idx) { store_idx = i; break; }
        seen++;
    }
    editor_load_spec_by_store_idx (store_idx);
    editor_rebuild_ui ();
}

/* Called when the user picks a different part from the dropdown. */
static void editor_on_part_changed (GtkDropDown *dropdown, gpointer user_data) {
    (void) user_data;
    if (!editor_has_spec) { return; }
    guint idx = gtk_drop_down_get_selected (GTK_DROP_DOWN (dropdown));
    if (idx == GTK_INVALID_LIST_POSITION) { editor_selected_part = -1; editor_rebuild_ui (); return; }
    editor_selected_part = (int) idx;
    editor_rebuild_fields ();
    editor_update_readouts ();
}

/* ---- per-part field editor ------------------------------------------ */

static const char *field_label (int field) {
    switch (field) {
        case EDIT_FIELD_RADIUS:   return "Radius";
        case EDIT_FIELD_HALF_LENGTH: return "Half length (axle X)";
        case EDIT_FIELD_HX:       return "Size X (half)";
        case EDIT_FIELD_HY:       return "Size Y (half)";
        case EDIT_FIELD_HZ:       return "Size Z (half)";
        case EDIT_FIELD_PX:       return "Pos X";
        case EDIT_FIELD_PY:       return "Pos Y";
        case EDIT_FIELD_PZ:       return "Pos Z";
        case EDIT_FIELD_QW:       return "Orient W";
        case EDIT_FIELD_QX:       return "Orient X";
        case EDIT_FIELD_QY:       return "Orient Y";
        case EDIT_FIELD_QZ:       return "Orient Z";
        case EDIT_FIELD_MU_S:     return "Mu static";
        case EDIT_FIELD_MU_K:     return "Mu kinetic";
        case EDIT_FIELD_REST:     return "Restitution";
        case EDIT_FIELD_MASS_DENSITY: return "Mass / Density";
        case EDIT_FIELD_COLOUR_R: return "Colour R";
        case EDIT_FIELD_COLOUR_G: return "Colour G";
        case EDIT_FIELD_COLOUR_B: return "Colour B";
        default: return "";
    }
}

static double field_value (int field) {
    const mig_spec_part *p = &editor_spec.parts [editor_selected_part];
    switch (field) {
        case EDIT_FIELD_RADIUS:   return p -> radius;
        case EDIT_FIELD_HALF_LENGTH: return p -> cylinder_half_length;
        case EDIT_FIELD_HX:       return p -> half_extensions_x;
        case EDIT_FIELD_HY:       return p -> half_extensions_y;
        case EDIT_FIELD_HZ:       return p -> half_extensions_z;
        case EDIT_FIELD_PX:       return p -> position_x;
        case EDIT_FIELD_PY:       return p -> position_y;
        case EDIT_FIELD_PZ:       return p -> position_z;
        case EDIT_FIELD_QW:       return p -> orientation_w;
        case EDIT_FIELD_QX:       return p -> orientation_x;
        case EDIT_FIELD_QY:       return p -> orientation_y;
        case EDIT_FIELD_QZ:       return p -> orientation_z;
        case EDIT_FIELD_MU_S:     return p -> mu_s;
        case EDIT_FIELD_MU_K:     return p -> mu_k;
        case EDIT_FIELD_REST:     return p -> restitution;
        case EDIT_FIELD_MASS_DENSITY: return p -> mass;
        case EDIT_FIELD_COLOUR_R: return p -> colour_r;
        case EDIT_FIELD_COLOUR_G: return p -> colour_g;
        case EDIT_FIELD_COLOUR_B: return p -> colour_b;
        default: return 0.0;
    }
}

static void field_set_value (int field, double v) {
    mig_spec_part *p = &editor_spec.parts [editor_selected_part];
    switch (field) {
        case EDIT_FIELD_RADIUS:   p -> radius = v; break;
        case EDIT_FIELD_HALF_LENGTH: p -> cylinder_half_length = v; break;
        case EDIT_FIELD_HX:       p -> half_extensions_x = v; break;
        case EDIT_FIELD_HY:       p -> half_extensions_y = v; break;
        case EDIT_FIELD_HZ:       p -> half_extensions_z = v; break;
        case EDIT_FIELD_PX:       p -> position_x = v; break;
        case EDIT_FIELD_PY:       p -> position_y = v; break;
        case EDIT_FIELD_PZ:       p -> position_z = v; break;
        case EDIT_FIELD_QW:       p -> orientation_w = v; break;
        case EDIT_FIELD_QX:       p -> orientation_x = v; break;
        case EDIT_FIELD_QY:       p -> orientation_y = v; break;
        case EDIT_FIELD_QZ:       p -> orientation_z = v; break;
        case EDIT_FIELD_MU_S:     p -> mu_s = v; break;
        case EDIT_FIELD_MU_K:     p -> mu_k = v; break;
        case EDIT_FIELD_REST:     p -> restitution = v; break;
        case EDIT_FIELD_MASS_DENSITY: p -> mass = v; break;
        case EDIT_FIELD_COLOUR_R: p -> colour_r = v; break;
        case EDIT_FIELD_COLOUR_G: p -> colour_g = v; break;
        case EDIT_FIELD_COLOUR_B: p -> colour_b = v; break;
    }
    editor_dirty = true;
    editor_update_readouts ();
}

/* Numeric entry: only digits, sign, dot, exponent. On activate, commit
 * the value to the editor spec. */
static void entry_insert_text_filter (GtkEditable *editable, const gchar *new_text,
                                      gint new_text_length, gint *position, gpointer user_data) {
    (void) position;
    (void) user_data;
    for (int i = 0; i < new_text_length; i++) {
        char c = new_text [i];
        if (!((c >= '0' && c <= '9') || c == '-' || c == '.' || c == 'e' || c == 'E' || c == '+')) {
            g_signal_stop_emission_by_name (editable, "insert-text");
            return;
        }
    }
}

static void entry_on_activate (GtkEntry *entry, gpointer user_data) {
    int field = GPOINTER_TO_INT (user_data);
    const char *text = gtk_editable_get_text (GTK_EDITABLE (entry));
    char *endptr = NULL;
    double v = strtod (text, &endptr);
    if (endptr != text && *endptr == '\0') {
        field_set_value (field, v);
    }
    /* Keep the text in the entry (don't clear) so the user can see it. */
}

static void editor_build_field_row (GtkBox *parent, int field, bool visible) {
    GtkWidget *row = gtk_box_new (GTK_ORIENTATION_HORIZONTAL, 6);
    GtkWidget *cap = gtk_label_new (field_label (field));
    gtk_widget_set_hexpand (cap, FALSE);
    gtk_widget_set_size_request (cap, 110, -1);
    gtk_label_set_xalign (GTK_LABEL (cap), 0.0f);
    GtkWidget *entry = gtk_entry_new ();
    char buf [32];
    snprintf (buf, sizeof (buf), "%.6g", field_value (field));
    gtk_editable_set_text (GTK_EDITABLE (entry), buf);
    gtk_widget_set_hexpand (entry, TRUE);
    gtk_widget_set_size_request (entry, 100, -1);
    gtk_box_append (GTK_BOX (row), cap);
    gtk_box_append (GTK_BOX (row), entry);
    gtk_box_append (parent, row);
    gtk_widget_set_visible (row, visible);
    editor_field_entries [field] = entry;
    g_signal_connect (entry, "insert-text", G_CALLBACK (entry_insert_text_filter), NULL);
    g_signal_connect (entry, "activate", G_CALLBACK (entry_on_activate), GINT_TO_POINTER (field));
    g_object_set_data (G_OBJECT (entry), "edit-field", GINT_TO_POINTER (field));
}

static void editor_rebuild_fields (void) {
    if (!editor_has_spec || editor_selected_part < 0 ||
        editor_selected_part >= editor_spec.count) {
        return;
    }
    const mig_spec_part *p = &editor_spec.parts [editor_selected_part];
    GtkWidget *container = GTK_WIDGET (g_object_get_data (G_OBJECT (editor_window), "field-container"));
    if (!container) { return; }
    gtk_list_box_remove_all (GTK_LIST_BOX (container));

    /* Type dropdown */
    GtkWidget *type_row = gtk_box_new (GTK_ORIENTATION_HORIZONTAL, 6);
    GtkWidget *type_cap = gtk_label_new ("Type");
    gtk_widget_set_hexpand (type_cap, FALSE);
    gtk_widget_set_size_request (type_cap, 110, -1);
    gtk_label_set_xalign (GTK_LABEL (type_cap), 0.0f);
    const char *type_names [] = {"sphere", "cube", "cylinder"};
    GtkStringList *type_list = gtk_string_list_new (NULL);
    for (int i = 0; i < 3; i++) { gtk_string_list_append (type_list, type_names [i]); }
    GtkWidget *type_dd = gtk_drop_down_new (G_LIST_MODEL (type_list), NULL);
    gtk_drop_down_set_selected (GTK_DROP_DOWN (type_dd), p -> type);
    gtk_widget_set_hexpand (type_dd, TRUE);
    gtk_box_append (GTK_BOX (type_row), type_cap);
    gtk_box_append (GTK_BOX (type_row), type_dd);
    gtk_list_box_append (GTK_LIST_BOX (container), type_row);
    editor_type_dropdown = type_dd;
    g_object_unref (type_list);

    /* Dims: per-type subset */
    GtkWidget *dims_row = gtk_box_new (GTK_ORIENTATION_HORIZONTAL, 6);
    GtkWidget *dims_cap = gtk_label_new ("Dims");
    gtk_widget_set_hexpand (dims_cap, FALSE);
    gtk_widget_set_size_request (dims_cap, 110, -1);
    gtk_label_set_xalign (GTK_LABEL (dims_cap), 0.0f);
    gtk_box_append (GTK_BOX (dims_row), dims_cap);

    if (p -> type == 0) { /* sphere: radius */
        editor_build_field_row (GTK_BOX (dims_row), EDIT_FIELD_RADIUS, true);
    } else if (p -> type == 1) { /* cube: hx hy hz */
        editor_build_field_row (GTK_BOX (dims_row), EDIT_FIELD_HX, true);
        editor_build_field_row (GTK_BOX (dims_row), EDIT_FIELD_HY, true);
        editor_build_field_row (GTK_BOX (dims_row), EDIT_FIELD_HZ, true);
    } else { /* cylinder: radius + half length */
        editor_build_field_row (GTK_BOX (dims_row), EDIT_FIELD_RADIUS, true);
        editor_build_field_row (GTK_BOX (dims_row), EDIT_FIELD_HALF_LENGTH, true);
    }
    gtk_list_box_append (GTK_LIST_BOX (container), dims_row);

    /* Pose */
    GtkWidget *pose_row = gtk_box_new (GTK_ORIENTATION_VERTICAL, 4);
    GtkWidget *pose_cap = gtk_label_new ("Pose (assembly frame)");
    gtk_widget_set_hexpand (pose_cap, FALSE);
    gtk_label_set_xalign (GTK_LABEL (pose_cap), 0.0f);
    gtk_box_append (GTK_BOX (pose_row), pose_cap);
    GtkWidget *pos_row = gtk_box_new (GTK_ORIENTATION_HORIZONTAL, 4);
    editor_build_field_row (GTK_BOX (pos_row), EDIT_FIELD_PX, true);
    editor_build_field_row (GTK_BOX (pos_row), EDIT_FIELD_PY, true);
    editor_build_field_row (GTK_BOX (pos_row), EDIT_FIELD_PZ, true);
    gtk_box_append (GTK_BOX (pose_row), pos_row);
    GtkWidget *ori_row = gtk_box_new (GTK_ORIENTATION_HORIZONTAL, 4);
    editor_build_field_row (GTK_BOX (ori_row), EDIT_FIELD_QW, true);
    editor_build_field_row (GTK_BOX (ori_row), EDIT_FIELD_QX, true);
    editor_build_field_row (GTK_BOX (ori_row), EDIT_FIELD_QY, true);
    editor_build_field_row (GTK_BOX (ori_row), EDIT_FIELD_QZ, true);
    gtk_box_append (GTK_BOX (pose_row), ori_row);
    gtk_list_box_append (GTK_LIST_BOX (container), pose_row);

    /* Material */
    GtkWidget *mat_row = gtk_box_new (GTK_ORIENTATION_VERTICAL, 4);
    GtkWidget *mat_cap = gtk_label_new ("Material");
    gtk_widget_set_hexpand (mat_cap, FALSE);
    gtk_label_set_xalign (GTK_LABEL (mat_cap), 0.0f);
    gtk_box_append (GTK_BOX (mat_row), mat_cap);
    GtkWidget *mat_vals = gtk_box_new (GTK_ORIENTATION_HORIZONTAL, 4);
    editor_build_field_row (GTK_BOX (mat_vals), EDIT_FIELD_MU_S, true);
    editor_build_field_row (GTK_BOX (mat_vals), EDIT_FIELD_MU_K, true);
    editor_build_field_row (GTK_BOX (mat_vals), EDIT_FIELD_REST, true);
    gtk_box_append (GTK_BOX (mat_row), mat_vals);
    gtk_list_box_append (GTK_LIST_BOX (container), mat_row);

    /* Mass XOR density */
    GtkWidget *mass_row = gtk_box_new (GTK_ORIENTATION_VERTICAL, 4);
    GtkWidget *mass_cap = gtk_label_new ("Mass source");
    gtk_widget_set_hexpand (mass_cap, FALSE);
    gtk_label_set_xalign (GTK_LABEL (mass_cap), 0.0f);
    gtk_box_append (GTK_BOX (mass_row), mass_cap);
    GtkWidget *mass_vals = gtk_box_new (GTK_ORIENTATION_HORIZONTAL, 4);
    editor_build_field_row (GTK_BOX (mass_vals), EDIT_FIELD_MASS_DENSITY, true);
    gtk_box_append (GTK_BOX (mass_row), mass_vals);
    gtk_list_box_append (GTK_LIST_BOX (container), mass_row);

    /* Colour */
    GtkWidget *col_row = gtk_box_new (GTK_ORIENTATION_VERTICAL, 4);
    GtkWidget *col_cap = gtk_label_new ("Colour");
    gtk_widget_set_hexpand (col_cap, FALSE);
    gtk_label_set_xalign (GTK_LABEL (col_cap), 0.0f);
    gtk_box_append (GTK_BOX (col_row), col_cap);
    GtkWidget *col_vals = gtk_box_new (GTK_ORIENTATION_HORIZONTAL, 4);
    editor_build_field_row (GTK_BOX (col_vals), EDIT_FIELD_COLOUR_R, true);
    editor_build_field_row (GTK_BOX (col_vals), EDIT_FIELD_COLOUR_G, true);
    editor_build_field_row (GTK_BOX (col_vals), EDIT_FIELD_COLOUR_B, true);
    gtk_box_append (GTK_BOX (col_row), col_vals);
    gtk_list_box_append (GTK_LIST_BOX (container), col_row);
}

static void editor_rebuild_ui (void) {
    if (!editor_window) { return; }
    /* Rebuild part dropdown from editor_spec. */
    GtkStringList *part_list = gtk_string_list_new (NULL);
    if (editor_has_spec) {
        static const char *type_names [] = {"sphere", "cube", "cylinder"};
        for (int i = 0; i < editor_spec.count; i++) {
            char label [96];
            snprintf (label, sizeof (label), "Part %d: %s", i,
                      type_names [editor_spec.parts [i].type]);
            gtk_string_list_append (part_list, label);
        }
    }
    gtk_drop_down_set_model (GTK_DROP_DOWN (editor_part_dropdown), G_LIST_MODEL (part_list));
    g_object_unref (part_list);
    if (editor_has_spec && editor_selected_part >= 0 &&
        editor_selected_part < editor_spec.count) {
        gtk_drop_down_set_selected (GTK_DROP_DOWN (editor_part_dropdown), editor_selected_part);
    } else {
        gtk_drop_down_set_selected (GTK_DROP_DOWN (editor_part_dropdown), GTK_INVALID_LIST_POSITION);
    }
    /* Rebuild fields if a part is selected. */
    GtkWidget *container = GTK_WIDGET (g_object_get_data (G_OBJECT (editor_window), "field-container"));
    if (container) { editor_rebuild_fields (); }
    editor_update_readouts ();
    gtk_widget_set_sensitive (editor_save_btn, editor_has_spec && editor_dirty);
    gtk_widget_set_sensitive (editor_remove_part_btn,
                              editor_has_spec && editor_selected_part >= 0 && editor_spec.count > 1);
}

/* ---- readouts (total mass + COM) ------------------------------------ */

static bool editor_compute_com (vector3 *out) {
    if (!editor_has_spec || editor_spec.count <= 0) { return false; }
    mig_part parts [MIG_SPEC_MAX_PARTS];
    for (int i = 0; i < editor_spec.count; i++) {
        const mig_spec_part *p = &editor_spec.parts [i];
        parts [i].type = (mig_part_type) p -> type;
        parts [i].radius = (float) p -> radius;
        parts [i].cylinder_half_length = (float) p -> cylinder_half_length;
        parts [i].half_extensions = (vector3) {
            (float) p -> half_extensions_x, (float) p -> half_extensions_y,
            (float) p -> half_extensions_z
        };
        parts [i].position = (vector3) {
            (float) p -> position_x, (float) p -> position_y, (float) p -> position_z
        };
        parts [i].orientation = (vector4) {
            (float) p -> orientation_w, (float) p -> orientation_x,
            (float) p -> orientation_y, (float) p -> orientation_z
        };
        parts [i].mass_mode = (mig_mass_mode) p -> mass_mode;
        parts [i].mass = (float) p -> mass;
    }
    mig_totals totals;
    if (mig_compose (parts, editor_spec.count, &totals) != mig_compose_ok) {
        return false;
    }
    *out = (vector3) {(float) totals.com.x, (float) totals.com.y, (float) totals.com.z};
    (void) totals;
    return true;
}

static void editor_update_readouts (void) {
    if (!editor_window) { return; }
    vector3 com;
    bool ok = editor_compute_com (&com);
    if (editor_mass_label) {
        char buf [96];
        if (ok) {
            double mass = 0.0;
            for (int i = 0; i < editor_spec.count; i++) {
                double m = editor_spec.parts [i].mass;
                if (editor_spec.parts [i].mass_mode == 1) {
                    const mig_spec_part *p = &editor_spec.parts [i];
                    double vol;
                    if (p -> type == 0) { vol = (4.0 / 3.0) * 3.14159265358979323846 *
                                                p -> radius * p -> radius * p -> radius; }
                    else if (p -> type == 1) { vol = 8.0 * p -> half_extensions_x *
                                                   p -> half_extensions_y * p -> half_extensions_z; }
                    else { vol = 3.14159265358979323846 * p -> radius * p -> radius *
                               (2.0 * p -> cylinder_half_length); }
                    m = p -> mass * vol;
                }
                mass += m;
            }
            snprintf (buf, sizeof (buf), "Total mass: %.4f kg", mass);
        } else {
            snprintf (buf, sizeof (buf), "Total mass: — (invalid spec)");
        }
        gtk_label_set_text (GTK_LABEL (editor_mass_label), buf);
    }
    if (editor_com_label) {
        char buf [96];
        if (ok) {
            snprintf (buf, sizeof (buf), "COM: (%.4f, %.4f, %.4f)", com.x, com.y, com.z);
        } else {
            snprintf (buf, sizeof (buf), "COM: —");
        }
        gtk_label_set_text (GTK_LABEL (editor_com_label), buf);
    }
}

/* ---- save: re-parse and stage --------------------------------------- */

static void editor_save_spec (void) {
    if (!editor_has_spec) { return; }
    char buf [8192];
    size_t n = mig_spec_emit_normalized (&editor_spec, buf, sizeof (buf));
    if (n == 0 || n >= sizeof (buf)) {
        term_err ("mpe: mig editor: failed to emit normalized spec\n");
        return;
    }
    char path [512];
    snprintf (path, sizeof (path), "./.mig_editor_%s.spec", editor_spec_name);
    FILE *f = fopen (path, "w");
    if (!f) {
        term_err ("mpe: mig editor: cannot write temp spec\n");
        return;
    }
    fwrite (buf, 1, n, f);
    fclose (f);
    mig_spec parsed;
    int rc = mig_spec_parse_file (path, 16, &parsed);
    remove (path);
    if (rc != mig_spec_ok) {
        term_err ("mpe: mig editor: edited spec rejected (code ");
        char n2 [16];
        snprintf (n2, sizeof (n2), "%d", rc);
        term_out (n2);
        term_err (") — not staged\n");
        return;
    }
    if (mig_spec_store_add (parsed.name, &parsed) < 0) {
        term_err ("mpe: mig editor: spec store full\n");
        return;
    }
    editor_dirty = false;
    snprintf (editor_spec_name, sizeof (editor_spec_name), "%s", parsed.name);
    editor_refresh_dropdown ();
    term_ok ("mpe: mig editor: spec ");
    term_out (parsed.name);
    term_out (" staged (");
    char n2 [16];
    snprintf (n2, sizeof (n2), "%d", parsed.count);
    term_out (n2);
    term_out (" parts)\n");
    editor_update_readouts ();
}

/* ---- add / remove part ---------------------------------------------- */

static void editor_add_part (void) {
    if (!editor_has_spec || editor_spec.count >= MIG_SPEC_MAX_PARTS) { return; }
    int idx = editor_spec.count;
    mig_spec_part *p = &editor_spec.parts [idx];
    memset (p, 0, sizeof (*p));
    p -> type = 0; /* sphere */
    p -> radius = 0.5;
    p -> orientation_w = 1.0;
    p -> mass_mode = 0; /* explicit */
    p -> mass = 1.0;
    p -> colour_r = 1.0; p -> colour_g = 1.0; p -> colour_b = 1.0;
    editor_spec.count = idx + 1;
    editor_selected_part = idx;
    editor_dirty = true;
    editor_rebuild_ui ();
}

static void editor_remove_part (void) {
    if (!editor_has_spec || editor_selected_part < 0 ||
        editor_selected_part >= editor_spec.count || editor_spec.count <= 1) {
        return;
    }
    for (int i = editor_selected_part; i < editor_spec.count - 1; i++) {
        editor_spec.parts [i] = editor_spec.parts [i + 1];
    }
    editor_spec.count--;
    editor_dirty = true;
    if (editor_selected_part >= editor_spec.count) {
        editor_selected_part = editor_spec.count - 1;
    }
    editor_rebuild_ui ();
}

/* ---- window lifecycle ------------------------------------------------ */

static void editor_on_destroy (GtkWidget *w, gpointer ud) {
    (void) w;
    (void) ud;
    editor_window = NULL;
    editor_spec_dropdown = NULL;
    editor_part_dropdown = NULL;
    editor_save_btn = NULL;
    editor_add_part_btn = NULL;
    editor_remove_part_btn = NULL;
    editor_mass_label = NULL;
    editor_com_label = NULL;
    memset (editor_field_entries, 0, sizeof (editor_field_entries));
    editor_type_dropdown = NULL;
    editor_gizmo_active = false;
    editor_gizmo_part = -1;
}

static gboolean editor_on_close_request (GtkWidget *w, gpointer ud) {
    (void) w;
    (void) ud;
    return FALSE;
}

static gboolean editor_on_key (GtkEventControllerKey *ctrl, guint keyval, guint keycode,
                               GdkModifierType state, gpointer ud) {
    (void) ctrl;
    (void) keycode;
    (void) state;
    (void) ud;
    if (keyval == GDK_KEY_Escape) {
        if (editor_window) { gtk_window_destroy (GTK_WINDOW (editor_window)); }
        return TRUE;
    }
    return FALSE;
}

void mig_editor_toggle (void) {
    if (editor_window) {
        gtk_window_destroy (GTK_WINDOW (editor_window));
        return;
    }
    editor_window = gtk_window_new ();
    gtk_widget_set_name (editor_window, "mpe-mig-editor");
    gtk_window_set_default_size (GTK_WINDOW (editor_window), 520, 640);
    gtk_window_set_title (GTK_WINDOW (editor_window), "MIG Editor");
    g_signal_connect (editor_window, "destroy", G_CALLBACK (editor_on_destroy), NULL);
    g_signal_connect (editor_window, "close-request", G_CALLBACK (editor_on_close_request), NULL);
    GtkEventController *keys = gtk_event_controller_key_new ();
    gtk_event_controller_set_propagation_phase (keys, GTK_PHASE_CAPTURE);
    gtk_widget_add_controller (editor_window, keys);
    g_signal_connect (keys, "key-pressed", G_CALLBACK (editor_on_key), NULL);

    static bool css_installed = false;
    if (!css_installed) {
        GtkCssProvider *css = gtk_css_provider_new ();
        gtk_css_provider_load_from_string (css,
            "#mpe-mig-editor { background: rgba(18,18,24,0.94); }\n"
            "#mpe-mig-editor label { color: #e8e8f0; font-size: 12px; }\n"
            "#mpe-mig-editor row-title { font-weight: bold; }\n");
        gtk_style_context_add_provider_for_display (
            gdk_display_get_default (), GTK_STYLE_PROVIDER (css),
            GTK_STYLE_PROVIDER_PRIORITY_APPLICATION);
        g_object_unref (css);
        css_installed = true;
    }

    GtkWidget *box = gtk_box_new (GTK_ORIENTATION_VERTICAL, 8);
    gtk_widget_set_margin_top (box, 10);
    gtk_widget_set_margin_bottom (box, 10);
    gtk_widget_set_margin_start (box, 10);
    gtk_widget_set_margin_end (box, 10);

    GtkWidget *title = gtk_label_new ("MIG Editor");
    gtk_label_set_xalign (GTK_LABEL (title), 0.0f);
    gtk_widget_add_css_class (title, "row-title");
    gtk_box_append (GTK_BOX (box), title);

    /* Spec selector + save */
    GtkWidget *spec_bar = gtk_box_new (GTK_ORIENTATION_HORIZONTAL, 6);
    GtkWidget *spec_cap = gtk_label_new ("Spec");
    gtk_widget_set_hexpand (spec_cap, FALSE);
    gtk_widget_set_size_request (spec_cap, 60, -1);
    gtk_label_set_xalign (GTK_LABEL (spec_cap), 0.0f);
    editor_spec_dropdown = gtk_drop_down_new (NULL, NULL);
    gtk_widget_set_hexpand (editor_spec_dropdown, TRUE);
    g_signal_connect (editor_spec_dropdown, "notify::selected", G_CALLBACK (editor_on_spec_changed), NULL);
    gtk_box_append (GTK_BOX (spec_bar), spec_cap);
    gtk_box_append (GTK_BOX (spec_bar), editor_spec_dropdown);
    editor_save_btn = gtk_button_new_with_label ("Save (stage)");
    gtk_widget_add_css_class (editor_save_btn, "suggested-action");
    g_signal_connect (editor_save_btn, "clicked", G_CALLBACK (editor_save_spec), NULL);
    gtk_box_append (GTK_BOX (spec_bar), editor_save_btn);
    gtk_box_append (GTK_BOX (box), spec_bar);

    /* Part selector */
    GtkWidget *part_bar = gtk_box_new (GTK_ORIENTATION_HORIZONTAL, 6);
    GtkWidget *part_cap = gtk_label_new ("Part");
    gtk_widget_set_hexpand (part_cap, FALSE);
    gtk_widget_set_size_request (part_cap, 60, -1);
    gtk_label_set_xalign (GTK_LABEL (part_cap), 0.0f);
    editor_part_dropdown = gtk_drop_down_new (NULL, NULL);
    gtk_widget_set_hexpand (editor_part_dropdown, TRUE);
    g_signal_connect (editor_part_dropdown, "notify::selected", G_CALLBACK (editor_on_part_changed), NULL);
    gtk_box_append (GTK_BOX (part_bar), part_cap);
    gtk_box_append (GTK_BOX (part_bar), editor_part_dropdown);
    editor_add_part_btn = gtk_button_new_with_label ("Add part");
    g_signal_connect (editor_add_part_btn, "clicked", G_CALLBACK (editor_add_part), NULL);
    editor_remove_part_btn = gtk_button_new_with_label ("Remove part");
    g_signal_connect (editor_remove_part_btn, "clicked", G_CALLBACK (editor_remove_part), NULL);
    gtk_box_append (GTK_BOX (part_bar), editor_add_part_btn);
    gtk_box_append (GTK_BOX (part_bar), editor_remove_part_btn);
    gtk_box_append (GTK_BOX (box), part_bar);

    /* Per-part fields (scrollable) */
    GtkWidget *field_cap = gtk_label_new ("Part fields");
    gtk_widget_set_hexpand (field_cap, FALSE);
    gtk_label_set_xalign (GTK_LABEL (field_cap), 0.0f);
    gtk_box_append (GTK_BOX (box), field_cap);

    GtkWidget *field_scroll = gtk_scrolled_window_new ();
    gtk_widget_set_vexpand (field_scroll, TRUE);
    gtk_widget_set_hexpand (field_scroll, TRUE);
    GtkWidget *field_container = gtk_list_box_new ();
    gtk_list_box_set_selection_mode (GTK_LIST_BOX (field_container), GTK_SELECTION_NONE);
    gtk_window_set_child (GTK_WINDOW (field_scroll), field_container);
    g_object_set_data (G_OBJECT (editor_window), "field-container", field_container);
    gtk_box_append (GTK_BOX (box), field_scroll);

    /* Readouts */
    editor_mass_label = gtk_label_new ("Total mass: —");
    gtk_label_set_xalign (GTK_LABEL (editor_mass_label), 0.0f);
    gtk_box_append (GTK_BOX (box), editor_mass_label);
    editor_com_label = gtk_label_new ("COM: —");
    gtk_label_set_xalign (GTK_LABEL (editor_com_label), 0.0f);
    gtk_box_append (GTK_BOX (box), editor_com_label);

    gtk_window_set_child (GTK_WINDOW (editor_window), box);

    editor_refresh_dropdown ();
    if (editor_has_spec) { editor_rebuild_ui (); }

    gtk_window_present (GTK_WINDOW (editor_window));
}

void mig_editor_sync (void) {
    if (!editor_window) { return; }
}

bool mig_editor_is_active (void) {
    return editor_window != NULL;
}

bool mig_editor_com_position (vector3 *out) {
    if (!editor_has_spec) { return false; }
    return editor_compute_com (out);
}

bool mig_editor_gizmo_active (void) {
    return editor_gizmo_active;
}
