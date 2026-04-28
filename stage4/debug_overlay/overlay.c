#include "overlay.h"
#include "../../stage1/master_header.h"
#include <stdio.h>
static GtkWidget *debug_overlay_information_label = NULL;
void overlay_init (GtkWidget *main_window_container, GtkWidget *gl_drawing_area_widget) {
    //Gtk overlay allows to stack widgets incrementally over top of each other
    GtkWidget *ui_overlay_layout_container = gtk_overlay_new ();
    //Move gl_area value into a overlay container
    g_object_ref (gl_drawing_area_widget);
    gtk_container_remove (GTK_CONTAINER (main_window_container), gl_drawing_area_widget);
    gtk_container_add (GTK_CONTAINER (ui_overlay_layout_container), gl_drawing_area_widget);
    g_object_unref (gl_drawing_area_widget);
    gtk_container_add (GTK_CONTAINER (main_window_container), ui_overlay_layout_container);
    //Create labels and anchor to the top left
    debug_overlay_information_label = gtk_label_new ("NORMAL");
    gtk_widget_set_halign (debug_overlay_information_label, GTK_ALIGN_START);
    gtk_widget_set_valign (debug_overlay_information_label, GTK_ALIGN_START);
    gtk_widget_set_margin_start (debug_overlay_information_label, 10);
    gtk_widget_set_margin_top (debug_overlay_information_label, 10);
    //White Text --> shows over dark background of gtk widgets
    GtkCssProvider *debug_label_style_provider = gtk_css_provider_new ();
    gtk_css_provider_load_from_data (debug_label_style_provider, "label {color: white; font-size: 14px;}", -1, NULL);
    gtk_style_context_add_provider (gtk_widget_get_style_context (debug_overlay_information_label), GTK_STYLE_PROVIDER (debug_label_style_provider), GTK_STYLE_PROVIDER_PRIORITY_APPLICATION);
    gtk_overlay_add_overlay (GTK_OVERLAY (ui_overlay_layout_container), debug_overlay_information_label);
} void overlay_update (void) {
    if (!debug_overlay_information_label) {return;}
    char debug_text_formatting_buffer [256];
    if ((selected_object < 0) || (selected_object >= object_count)) {
        gtk_label_set_text (GTK_LABEL (debug_overlay_information_label), "No object selected | E: select object  F: impulse application  Space: spawn object");
        return;
    } rigidbody *selected_rigidbody_object_pointer = &obj_per_scene [selected_object];
    vector3 constant_gravitational_acceleration_vector = {0.0, -9.81, 0.0};
    state_energy calculated_system_energy_state = force_to_system_energy_amount (selected_rigidbody_object_pointer, constant_gravitational_acceleration_vector);
    float current_object_speed_magnitude = vector3_length (selected_rigidbody_object_pointer -> velocity);
    snprintf (debug_text_formatting_buffer, sizeof (debug_text_formatting_buffer), "Object %d | Position = (%.1f, %.1f, %.1f) | Speed = %.2f | Ek = %.2f | Epg = %.2f | Em = %.2f", selected_object,
        selected_rigidbody_object_pointer -> position.x, selected_rigidbody_object_pointer -> position.y, selected_rigidbody_object_pointer -> position.z,
        current_object_speed_magnitude,
        calculated_system_energy_state.ek, calculated_system_energy_state.epg, calculated_system_energy_state.em
    ); gtk_label_set_text (GTK_LABEL (debug_overlay_information_label), debug_text_formatting_buffer);
}
