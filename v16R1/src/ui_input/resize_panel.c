/* SANDBOX S4a: the resize panel. See resize_panel.h.
 * Window recipe cloned from ui_input/debug_terminal.c (plain GtkWindow,
 * transient, destroy+close-request handlers, Esc closes, one CSS
 * provider keyed on the widget name). Widget rows are built once and
 * shown/hidden per selected body type — no rebuild churn. */
#include "resize_panel.h"
#include "../mpe_engine.h"
#include "object_selector.h"
#include "mouse_lock.h"
#include <stdio.h>
#include <string.h>

#define RESIZE_SLIDER_MIN 0.01f
#define RESIZE_SLIDER_MAX 20.0f

static GtkWidget *panel_window = NULL;
static GtkWidget *panel_title_label = NULL;
/* Rows: 0 = sphere radius, 1 = cylinder radius, 2 = cylinder half length,
 * 3..5 = cube half extents X/Y/Z. Row widget = box [label | slider | value]. */
static GtkWidget *panel_rows [6] = {NULL, NULL, NULL, NULL, NULL, NULL};
static GtkWidget *panel_sliders [6] = {NULL, NULL, NULL, NULL, NULL, NULL};
static GtkWidget *panel_values [6] = {NULL, NULL, NULL, NULL, NULL, NULL};
static uint32_t panel_synced_id = 0;
static bool panel_syncing = false; /* programmatic slider sets must not re-enter */
static bool panel_css_installed = false;

static rigidbody *panel_selected_body (int *index_out) {
    if (selected_object < 0) { return NULL; }
    physics_world *w = physics_world_get_primary ();
    if ((!w) || (selected_object >= w -> body_count)) { return NULL; }
    if (index_out) { *index_out = selected_object; }
    return &w -> bodies [selected_object];
} static void panel_row_show (int row, const char *label, float value, bool visible) {
    gtk_widget_set_visible (panel_rows [row], visible);
    if (!visible) { return; }
    GtkLabel *l = GTK_LABEL (g_object_get_data (G_OBJECT (panel_rows [row]), "caption"));
    if (l) { gtk_label_set_text (l, label); }
    panel_syncing = true;
    gtk_range_set_value (GTK_RANGE (panel_sliders [row]), value);
    panel_syncing = false;
    char buf [32];
    snprintf (buf, sizeof (buf), "%.2f m", value);
    gtk_label_set_text (GTK_LABEL (panel_values [row]), buf);
} static void panel_apply_size (int row, double new_value) {
    rigidbody *rb = panel_selected_body (NULL);
    if ((!rb) || panel_syncing) { return; }
    float v = (float) new_value;
    if (!isfinite (v) || v < 0.01f) { v = 0.01f; }
    if (v > 100.0f) { v = 100.0f; }
    if (rb -> type == object_sphere && row == 0) {
        rb -> radius = v;
        rigidbody_update_inertia_sphere (rb);
    } else if (rb -> type == object_cylinder && row == 1) {
        rb -> radius = v;
        rigidbody_update_inertia_cylinder (rb);
    } else if (rb -> type == object_cylinder && row == 2) {
        rb -> cylinder_half_length = v;
        rigidbody_update_inertia_cylinder (rb);
    } else if (rb -> type == object_cube && row >= 3) {
        if (row == 3) { rb -> half_extensions.x = v; }
        else if (row == 4) { rb -> half_extensions.y = v; }
        else { rb -> half_extensions.z = v; }
        /* The covering bound feeds the broadphase (the spawn convention:
         * radius = |half_extensions|); a stale bound = missed pairs. */
        rb -> radius = vector3_length (rb -> half_extensions);
        rigidbody_update_inertia_cube (rb);
    } else {
        return; /* type changed under the panel; sync will rebuild rows */
    }
    /* The editor radius-edit tail (MPE_TASK_06 + 19): geometry changed,
     * so the contact cache is stale and the body must re-enter the solver. */
    contact_cache_clear (physics_world_get_primary ());
    rigidbody_wake (rb);
    char buf [32];
    snprintf (buf, sizeof (buf), "%.2f m", v);
    gtk_label_set_text (GTK_LABEL (panel_values [row]), buf);
} static void panel_slider_changed (GtkRange *range, gpointer user_data) {
    (void) range;
    panel_apply_size ((int) (intptr_t) user_data, gtk_range_get_value (GTK_RANGE (panel_sliders [(int) (intptr_t) user_data])));
} static void panel_refresh (void) {
    int idx = -1;
    rigidbody *rb = panel_selected_body (&idx);
    if (!rb) {
        gtk_label_set_text (GTK_LABEL (panel_title_label), "Resize — no selection (right-click a body)");
        for (int r = 0; r < 6; r++) { gtk_widget_set_visible (panel_rows [r], FALSE); }
        panel_synced_id = selected_object_id;
        return;
    }
    const char *tn = rb -> type == object_sphere ? "Sphere" : rb -> type == object_cylinder ? "Cylinder" : "Cube";
    char buf [96];
    snprintf (buf, sizeof (buf), "Resize — %s [%d] (id %u)", tn, idx, (unsigned) rb -> object_id);
    gtk_label_set_text (GTK_LABEL (panel_title_label), buf);
    /* Rows: 0 sphere radius; 1 cyl radius; 2 cyl half length; 3-5 cube XYZ. */
    panel_row_show (0, "Radius", rb -> radius, rb -> type == object_sphere);
    panel_row_show (1, "Radius", rb -> radius, rb -> type == object_cylinder);
    panel_row_show (2, "Half length (axle X)", rb -> cylinder_half_length, rb -> type == object_cylinder);
    panel_row_show (3, "Size X (half)", rb -> half_extensions.x, rb -> type == object_cube);
    panel_row_show (4, "Size Y (half)", rb -> half_extensions.y, rb -> type == object_cube);
    panel_row_show (5, "Size Z (half)", rb -> half_extensions.z, rb -> type == object_cube);
    panel_synced_id = selected_object_id;
} static gboolean panel_on_destroy (GtkWidget *w, gpointer ud) {
    (void) w;
    (void) ud;
    panel_window = NULL;
    panel_title_label = NULL;
    for (int r = 0; r < 6; r++) { panel_rows [r] = NULL; panel_sliders [r] = NULL; panel_values [r] = NULL; }
    panel_synced_id = 0;
    return FALSE; /* proceed with destroy */
} static gboolean panel_on_close_request (GtkWidget *w, gpointer ud) {
    (void) w;
    (void) ud;
    /* destroy handler NULLs the globals; keep default handling. */
    return FALSE;
} static gboolean panel_on_key (GtkEventControllerKey *ctrl, guint keyval, guint keycode,
                               GdkModifierType state, gpointer ud) {
    (void) ctrl;
    (void) keycode;
    (void) state;
    (void) ud;
    if (keyval == GDK_KEY_Escape) {
        if (panel_window) { gtk_window_destroy (GTK_WINDOW (panel_window)); }
        return TRUE;
    }
    return FALSE;
} static GtkWidget *panel_build_row (GtkBox *parent, const char *caption, int row_index) {
    GtkWidget *row = gtk_box_new (GTK_ORIENTATION_HORIZONTAL, 6);
    GtkWidget *cap = gtk_label_new (caption);
    gtk_widget_set_hexpand (cap, FALSE);
    gtk_widget_set_size_request (cap, 150, -1);
    gtk_label_set_xalign (GTK_LABEL (cap), 0.0f);
    GtkWidget *slider = gtk_scale_new_with_range (GTK_ORIENTATION_HORIZONTAL,
                                                  RESIZE_SLIDER_MIN, RESIZE_SLIDER_MAX, 0.01);
    gtk_widget_set_hexpand (slider, TRUE);
    gtk_widget_set_size_request (slider, 180, -1);
    GtkWidget *val = gtk_label_new ("—");
    gtk_widget_set_hexpand (val, FALSE);
    gtk_widget_set_size_request (val, 72, -1);
    gtk_label_set_xalign (GTK_LABEL (val), 1.0f);
    gtk_box_append (GTK_BOX (row), cap);
    gtk_box_append (GTK_BOX (row), slider);
    gtk_box_append (GTK_BOX (row), val);
    gtk_box_append (parent, row);
    g_object_set_data (G_OBJECT (row), "caption", cap);
    panel_rows [row_index] = row;
    panel_sliders [row_index] = slider;
    panel_values [row_index] = val;
    g_signal_connect (slider, "value-changed", G_CALLBACK (panel_slider_changed),
                      (gpointer) (intptr_t) row_index);
    return row;
} void resize_panel_toggle (GtkWidget *parent_window) {
    if (panel_window) {
        gtk_window_destroy (GTK_WINDOW (panel_window));
        return;
    }
    panel_window = gtk_window_new ();
    gtk_widget_set_name (panel_window, "mpe-resize-panel");
    gtk_window_set_default_size (GTK_WINDOW (panel_window), 460, 80);
    gtk_window_set_title (GTK_WINDOW (panel_window), "Resize");
    if (parent_window) {
        GtkRoot *toplevel = gtk_widget_get_root (GTK_WIDGET (parent_window));
        if (toplevel && GTK_IS_WINDOW (toplevel)) {
            gtk_window_set_transient_for (GTK_WINDOW (panel_window), GTK_WINDOW (toplevel));
        }
    }
    g_signal_connect (panel_window, "destroy", G_CALLBACK (panel_on_destroy), NULL);
    g_signal_connect (panel_window, "close-request", G_CALLBACK (panel_on_close_request), NULL);
    GtkEventController *keys = gtk_event_controller_key_new ();
    gtk_event_controller_set_propagation_phase (keys, GTK_PHASE_CAPTURE);
    gtk_widget_add_controller (panel_window, keys);
    g_signal_connect (keys, "key-pressed", G_CALLBACK (panel_on_key), NULL);
    if (!panel_css_installed) {
        GtkCssProvider *css = gtk_css_provider_new ();
        gtk_css_provider_load_from_string (css,
            "#mpe-resize-panel { background: rgba(18,18,24,0.94); }\n"
            "#mpe-resize-panel label { color: #e8e8f0; font-size: 12px; }\n"
            "#mpe-resize-panel row-title { font-weight: bold; }\n");
        gtk_style_context_add_provider_for_display (
            gdk_display_get_default (), GTK_STYLE_PROVIDER (css),
            GTK_STYLE_PROVIDER_PRIORITY_APPLICATION);
        g_object_unref (css);
        panel_css_installed = true;
    }
    GtkWidget *box = gtk_box_new (GTK_ORIENTATION_VERTICAL, 8);
    gtk_widget_set_margin_top (box, 10);
    gtk_widget_set_margin_bottom (box, 10);
    gtk_widget_set_margin_start (box, 10);
    gtk_widget_set_margin_end (box, 10);
    panel_title_label = gtk_label_new ("Resize");
    gtk_label_set_xalign (GTK_LABEL (panel_title_label), 0.0f);
    gtk_widget_add_css_class (panel_title_label, "row-title");
    gtk_box_append (GTK_BOX (box), panel_title_label);
    panel_build_row (GTK_BOX (box), "Radius", 0);
    panel_build_row (GTK_BOX (box), "Radius", 1);
    panel_build_row (GTK_BOX (box), "Half length (axle X)", 2);
    panel_build_row (GTK_BOX (box), "Size X (half)", 3);
    panel_build_row (GTK_BOX (box), "Size Y (half)", 4);
    panel_build_row (GTK_BOX (box), "Size Z (half)", 5);
    gtk_window_set_child (GTK_WINDOW (panel_window), box);
    panel_synced_id = 0; /* force a full refresh */
    panel_refresh ();
    /* The sliders need the pointer: same unlock flow as the terminal. */
    if (main_inputs.is_mouse_locked && parent_window) {
        GtkRoot *toplevel = gtk_widget_get_root (GTK_WIDGET (parent_window));
        mouse_lock_disable (GTK_WIDGET (toplevel) ? GTK_WIDGET (toplevel) : parent_window);
        main_inputs.is_mouse_locked = false;
    }
    gtk_window_present (GTK_WINDOW (panel_window));
} void resize_panel_sync (void) {
    if (!panel_window) { return; }
    if (panel_synced_id == selected_object_id) { return; }
    panel_refresh ();
} bool resize_panel_is_open (void) {
    return panel_window != NULL;
}
