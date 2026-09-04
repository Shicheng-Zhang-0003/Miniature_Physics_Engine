/* MFS_PHASE_A: editor dialog helpers extracted from simulation.c.
 * Owns editor_dialog_active, the numerical-input dialog, and editor_reset.
 */
#include "../mpe_engine.h"
#include <stdlib.h>

static bool editor_dialog_active = false;

static void on_entry_insert_text(GtkEditable *editable, const gchar *new_text, gint new_text_length, gint *position,
                                 gpointer user_data) {
    (void) position;
    (void) user_data;
    for (int current_buffer = 0; current_buffer < new_text_length; current_buffer++) {
        char current_header_input = new_text[current_buffer];
        if (!((current_header_input >= '0' && current_header_input <= '9') || (current_header_input == '-') ||
              (current_header_input == '.'))) {
            g_signal_stop_emission_by_name(editable, "insert-text");
            return;
        }
    }
}
float open_numerical_input_dialog(GtkWidget *parent, const char *title, float current_value) {
    main_inputs.suppress_mouse_delta = true;
    editor_dialog_active = true;
    GtkWidget *dialog_parent_widget = NULL; /* A3_PATCH_25_DIALOG_SAFETY */
    if ((parent) && (GTK_IS_WIDGET(parent))) {
        dialog_parent_widget = gtk_widget_get_toplevel(GTK_WIDGET(parent));
    }
    GtkWidget *dialog = gtk_dialog_new_with_buttons(title, GTK_WINDOW(dialog_parent_widget),
                                                    GTK_DIALOG_MODAL | GTK_DIALOG_DESTROY_WITH_PARENT, "_Cancel",
                                                    GTK_RESPONSE_CANCEL, "_OK", GTK_RESPONSE_OK, NULL);
    gtk_window_set_default_size(GTK_WINDOW(dialog), 300, 150);
    GtkWidget *content_area = gtk_dialog_get_content_area(GTK_DIALOG(dialog));
    GtkWidget *box = gtk_box_new(GTK_ORIENTATION_VERTICAL, 10);
    gtk_container_set_border_width(GTK_CONTAINER(box), 15);
    gtk_container_add(GTK_CONTAINER(content_area), box);
    char label_text[256];
    snprintf(label_text, sizeof(label_text), "Current value: %.4f\nEnter new value:", current_value);
    GtkWidget *label = gtk_label_new(label_text);
    gtk_box_pack_start(GTK_BOX(box), label, FALSE, FALSE, 0);
    GtkWidget *entry = gtk_entry_new();
    char current_value_str[64];
    snprintf(current_value_str, sizeof(current_value_str), "%.4f", current_value);
    gtk_entry_set_text(GTK_ENTRY(entry), current_value_str);
    gtk_entry_set_activates_default(GTK_ENTRY(entry), TRUE);
    gtk_box_pack_start(GTK_BOX(box), entry, FALSE, FALSE, 0);
    g_signal_connect(entry, "insert-text", G_CALLBACK(on_entry_insert_text), NULL);
    gtk_dialog_set_default_response(GTK_DIALOG(dialog), GTK_RESPONSE_OK);
    gtk_widget_show_all(dialog);
    float result_value = current_value;
    gint response = gtk_dialog_run(GTK_DIALOG(dialog));
    if (response == GTK_RESPONSE_OK) {
        const gchar *text = gtk_entry_get_text(GTK_ENTRY(entry));
        char *endptr;
        float parsed = strtof(text, &endptr);
        if ((endptr != text) && (*endptr == '\0')) {
            result_value = parsed;
        }
    }
    gtk_widget_destroy(dialog);
    main_inputs.suppress_mouse_delta = false; /* A3_PATCH_25_DIALOG_SAFETY */
    editor_dialog_active = false;
    /* MPE_TASK_22_ENTER_SPAWN_DIALOG_CLEAR_BEGIN */
    main_inputs.enter_spawn_held = false;
    /* MPE_TASK_22_ENTER_SPAWN_DIALOG_CLEAR_END */
    return result_value;
}
void editor_reset(void) {
    clear_selection();

    main_inputs.is_menu_open = false;
    main_inputs.spawner_menu_level = 0;
    main_inputs.velocity_menu_level = 0;
    main_inputs.object_menu_level = 0;
    main_inputs.marked_joint_object_index = -1;

    main_inputs.menu_1_pressed = false;
    main_inputs.menu_2_pressed = false;
    main_inputs.menu_3_pressed = false;

    main_inputs.up_arrow_pressed = false;
    main_inputs.down_arrow_pressed = false;
    main_inputs.left_arrow_pressed = false;
    main_inputs.right_arrow_pressed = false;
    main_inputs.enter_key_pressed = false;
    main_inputs.e_key_pressed = false;
    config_menu_close(); /* MPE_TASK_35 */
}

/* MFS_PHASE_A: accessor so simulation.c can check dialog state without
 * touching the now module-private editor_dialog_active flag. */
bool editor_dialog_is_active(void) {
    return editor_dialog_active;
}
