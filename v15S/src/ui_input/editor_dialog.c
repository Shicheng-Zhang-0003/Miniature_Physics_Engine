/* GTK4-PREP: GTK3 preserved under #else; GTK4 implementation follows. */
#include "../mpe_engine.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <gdk/gdkkeysyms.h>
static bool editor_dialog_active = false;
static void on_entry_insert_text (GtkEditable *editable, const gchar * new_text, gint new_text_length, gint * position,
                                  gpointer user_data) {
    (void) position;
    (void) user_data;
    for (int i = 0; i < new_text_length; i++) {
        char c = new_text [i];
        if (!((c >= '0' && c <= '9') || (c == '-') || (c == '.'))) {
            g_signal_stop_emission_by_name (editable, "insert-text");
            return;
        }
    }
}
typedef struct {
    GMainLoop *loop;
    GtkWidget *entry;
    GtkWidget *dialog;
    float current_value;
    float result_value;
    gboolean confirmed;
} EditorDialogState;
static void editor_dialog_quit_with_result (EditorDialogState *state, float value, gboolean confirmed) {
    state -> result_value = value;
    state -> confirmed = confirmed;
    if (state -> loop && g_main_loop_is_running (state -> loop)) {
        g_main_loop_quit (state -> loop);
    }
}
static void editor_dialog_on_ok (GtkButton *button, gpointer user_data) {
    (void) button;
    EditorDialogState *state = (EditorDialogState *) user_data;
    const char *text = gtk_editable_get_text (GTK_EDITABLE (state -> entry));
    char *endptr = NULL;
    float parsed = strtof (text, &endptr);
    if ((endptr != text) && (*endptr == '\0')) {
        editor_dialog_quit_with_result (state, parsed, TRUE);
    } else {
        editor_dialog_quit_with_result (state, state -> current_value, TRUE);
    }
}
static void editor_dialog_on_cancel (GtkButton *button, gpointer user_data) {
    (void) button;
    EditorDialogState *state = (EditorDialogState *) user_data;
    editor_dialog_quit_with_result (state, state -> current_value, FALSE);
}
static gboolean editor_dialog_on_close_request (GtkWindow *window, gpointer user_data) {
    (void) window;
    EditorDialogState *state = (EditorDialogState *) user_data;
    editor_dialog_quit_with_result (state, state -> current_value, FALSE);
    return TRUE;
}
static void editor_dialog_on_entry_activate (GtkEntry *entry, gpointer user_data) {
    (void) entry;
    editor_dialog_on_ok (NULL, user_data);
}
static gboolean editor_dialog_on_key_pressed (GtkEventControllerKey *ctrl, guint keyval, guint keycode,
                                              GdkModifierType state_flags, gpointer user_data) {
    (void) ctrl;
    (void) keycode;
    (void) state_flags;
    EditorDialogState *state = (EditorDialogState *) user_data;
    if (keyval == GDK_KEY_Escape) {
        editor_dialog_on_cancel (NULL, state);
        return TRUE;
    }
    return FALSE;
}
float open_numerical_input_dialog (GtkWidget *parent, const char *title, float current_value) {
    main_inputs.suppress_mouse_delta = true;
    editor_dialog_active = true;
    EditorDialogState state = {0};
    state.current_value = current_value;
    state.result_value = current_value;
    state.confirmed = FALSE;
    state.loop = g_main_loop_new (NULL, FALSE);
    GtkWindow *transient_parent = NULL;
    if ((parent) && (GTK_IS_WIDGET (parent))) {
        GtkRoot *root = gtk_widget_get_root (GTK_WIDGET (parent));
        if ((root) && (GTK_IS_WINDOW (root))) {
            transient_parent = GTK_WINDOW (root);
        } else if (GTK_IS_WINDOW (parent)) {
            transient_parent = GTK_WINDOW (parent);
        }
    }
    GtkWidget *dialog = gtk_window_new ();
    state.dialog = dialog;
    state.entry = NULL;
    gtk_window_set_title (GTK_WINDOW (dialog), title ? title : "Input");
    gtk_window_set_default_size (GTK_WINDOW (dialog), 300, 150);
    gtk_window_set_modal (GTK_WINDOW (dialog), TRUE);
    gtk_window_set_resizable (GTK_WINDOW (dialog), FALSE);
    if (transient_parent) {
        gtk_window_set_transient_for (GTK_WINDOW (dialog), transient_parent);
    }
    g_signal_connect (dialog, "close-request", G_CALLBACK (editor_dialog_on_close_request), &state);
    {
        GtkEventController *key_ctrl = gtk_event_controller_key_new ();
        g_signal_connect (key_ctrl, "key-pressed", G_CALLBACK (editor_dialog_on_key_pressed), &state);
        gtk_widget_add_controller (dialog, key_ctrl);
    }
    GtkWidget *box = gtk_box_new (GTK_ORIENTATION_VERTICAL, 10);
    gtk_widget_set_margin_top (box, 15);
    gtk_widget_set_margin_bottom (box, 15);
    gtk_widget_set_margin_start (box, 15);
    gtk_widget_set_margin_end (box, 15);
    gtk_window_set_child (GTK_WINDOW (dialog), box);
    char label_text [256];
    snprintf (label_text, sizeof (label_text), "Current value: %.4f\nEnter new value:", current_value);
    GtkWidget *label = gtk_label_new (label_text);
    gtk_widget_set_halign (label, GTK_ALIGN_START);
    gtk_label_set_wrap (GTK_LABEL (label), TRUE);
    gtk_box_append (GTK_BOX (box), label);
    GtkWidget *entry = gtk_entry_new ();
    state.entry = entry;
    char current_value_str [64];
    snprintf (current_value_str, sizeof (current_value_str), "%.4f", current_value);
    gtk_editable_set_text (GTK_EDITABLE (entry), current_value_str);
    gtk_widget_set_hexpand (entry, TRUE);
    gtk_box_append (GTK_BOX (box), entry);
    g_signal_connect (entry, "insert-text", G_CALLBACK (on_entry_insert_text), NULL);
    g_signal_connect (entry, "activate", G_CALLBACK (editor_dialog_on_entry_activate), &state);
    GtkWidget *button_box = gtk_box_new (GTK_ORIENTATION_HORIZONTAL, 10);
    gtk_widget_set_halign (button_box, GTK_ALIGN_END);
    gtk_widget_set_hexpand (button_box, TRUE);
    gtk_box_append (GTK_BOX (box), button_box);
    GtkWidget *cancel_btn = gtk_button_new_with_label ("Cancel");
    GtkWidget *ok_btn = gtk_button_new_with_label ("OK");
    gtk_widget_add_css_class (ok_btn, "suggested-action");
    gtk_box_append (GTK_BOX (button_box), cancel_btn);
    gtk_box_append (GTK_BOX (button_box), ok_btn);
    g_signal_connect (cancel_btn, "clicked", G_CALLBACK (editor_dialog_on_cancel), &state);
    g_signal_connect (ok_btn, "clicked", G_CALLBACK (editor_dialog_on_ok), &state);
    gtk_window_set_default_widget (GTK_WINDOW (dialog), ok_btn);
    gtk_widget_grab_focus (entry);
    gtk_editable_select_region (GTK_EDITABLE (entry), 0, -1);
    gtk_window_present (GTK_WINDOW (dialog));
    g_main_loop_run (state.loop);
    g_main_loop_unref (state.loop);
    state.loop = NULL;
    if (GTK_IS_WINDOW (dialog)) {
        gtk_window_destroy (GTK_WINDOW (dialog));
    }
    while (g_main_context_pending (NULL)) {
        g_main_context_iteration (NULL, FALSE);
    }
    main_inputs.suppress_mouse_delta = false;
    editor_dialog_active = false;
    main_inputs.enter_spawn_held = false;
    return state.result_value;
}
void editor_reset (void) {
    clear_selection ();
    main_inputs.is_menu_open = false;
    main_inputs.spawner_menu_level = 0;
    main_inputs.velocity_menu_level = 0;
    main_inputs.object_menu_level = 0;
    main_inputs.marked_joint_object_index = -1;
    main_inputs.menu_1_pressed = false;
    main_inputs.menu_2_pressed = false;
    main_inputs.menu_3_pressed = false;
    main_inputs.menu_4_pressed = false;
    main_inputs.menu_5_pressed = false;
    main_inputs.menu_6_pressed = false;
    main_inputs.up_arrow_pressed = false;
    main_inputs.down_arrow_pressed = false;
    main_inputs.enter_key_pressed = false;
    main_inputs.e_key_pressed = false;
    /* Transient motion + pointer state: a reset must not leave held WASD,
     * queued clicks, or stale deltas driving the next context. Persistent
     * modes (is_mouse_locked, is_debug_mode_active, suppress_mouse_delta)
     * are deliberately kept: they describe the session, not the edit. */
    main_inputs.w_key_pressed = false;
    main_inputs.a_key_pressed = false;
    main_inputs.s_key_pressed = false;
    main_inputs.d_key_pressed = false;
    main_inputs.space_key_pressed = false;
    main_inputs.shift_key_pressed = false;
    main_inputs.i_key_pressed = false;
    main_inputs.j_key_pressed = false;
    main_inputs.k_key_pressed = false;
    main_inputs.l_key_pressed = false;
    main_inputs.r_key_pressed = false;
    main_inputs.f_key_pressed = false;
    main_inputs.right_mouse_button_clicked = false;
    main_inputs.middle_mouse_button_clicked = false;
    main_inputs.mouse_delta_x = 0.0f;
    main_inputs.mouse_delta_y = 0.0f;
    main_inputs.enter_spawn_held = false;
    /* One-shot test/debug requests sampled from the old context. */
    main_inputs.stability_test_pressed = false;
    main_inputs.sleep_wake_test_pressed = false;
    main_inputs.editor_torture_pressed = false;
    main_inputs.spawn_stress_pressed = false;
    main_inputs.validation_report_pressed = false;
    main_inputs.debug_terminal_pressed = false;
    main_inputs.long_run_validation_pressed = false;
    main_inputs.config_torture_pressed = false;
    config_menu_close ();
}
bool editor_dialog_is_active (void) {
    return editor_dialog_active;
}
