#include "overlay.h"
#include "../../stage1/master_header.h"
#include <stdint.h>
#include <stdio.h>
static GtkWidget *debug_information_label = NULL;
GtkWidget *overlay_init (GtkWidget *gl_drawing_area_widget) {
    //Gtk overlay allows to stack widgets incrementally over top of each other
    GtkWidget *ui_overlay_container = gtk_overlay_new ();
    //Add a fresh overlay container to the new window
    gtk_container_add (GTK_CONTAINER (ui_overlay_container), gl_drawing_area_widget);
    //Create labels and anchor to the top left
    debug_information_label = gtk_label_new ("NORMAL");
    gtk_widget_set_halign (debug_information_label, GTK_ALIGN_START);
    gtk_widget_set_valign (debug_information_label, GTK_ALIGN_START);
    gtk_widget_set_margin_start (debug_information_label, 10);
    gtk_widget_set_margin_top (debug_information_label, 10);
    //White Text --> shows over dark background of gtk widgets
    GtkCssProvider *overlay_style_provider = gtk_css_provider_new ();
    //gtk_css_provider_load_from_data (overlay_style_provider, "label {color: white; font-size: 14px;}", -1, NULL);
    gtk_overlay_add_overlay (GTK_OVERLAY (ui_overlay_container), debug_information_label);
    gtk_style_context_add_provider (gtk_widget_get_style_context (debug_information_label), GTK_STYLE_PROVIDER (overlay_style_provider), GTK_STYLE_PROVIDER_PRIORITY_APPLICATION);

    // Center Crosshair
    GtkWidget *center_crosshair_label = gtk_label_new ("+");
    gtk_widget_set_halign (center_crosshair_label, GTK_ALIGN_CENTER);
    gtk_widget_set_valign (center_crosshair_label, GTK_ALIGN_CENTER);
    // Style it to be subtle
    gtk_style_context_add_provider (gtk_widget_get_style_context (center_crosshair_label), GTK_STYLE_PROVIDER (overlay_style_provider), GTK_STYLE_PROVIDER_PRIORITY_APPLICATION);
    gtk_overlay_add_overlay (GTK_OVERLAY (ui_overlay_container), center_crosshair_label);

    return ui_overlay_container;
} void overlay_update (void) {
    if (!debug_information_label) {return;}
    char information_text_buffer [1024];
    char game_mode_text [32];
    if (main_inputs.is_debug_mode_active) {snprintf (game_mode_text, sizeof (game_mode_text), "DEBUG MODE");}
    else {snprintf (game_mode_text, sizeof (game_mode_text), "GAME MODE");}
    if ((selected_object < 0) || (selected_object >= object_count)) {
        snprintf (information_text_buffer, sizeof (information_text_buffer), "[%s] | No object selected | Shift: spawn | R-Click: select | 0: Toggle Mode", game_mode_text);
        gtk_label_set_text (GTK_LABEL (debug_information_label), information_text_buffer);
        return;
    } rigidbody *selected_rigid_body = &obj_per_scene [selected_object];
    vector3 global_gravity_acceleration = {0.0, -9.81, 0.0};
    //state_energy calculated_energy = force_to_system_energy_amount (selected_rigid_body, global_gravity_acceleration);
    float selected_object_speed = vector3_length (selected_rigid_body -> velocity);
    snprintf (information_text_buffer, sizeof (information_text_buffer), "[%s] | Obj %d | Pos(%.1f, %.1f, %.1f) | Spd %.2f | Space: impulse | L-Click: remove", game_mode_text, selected_object,
        selected_rigid_body -> position.x, selected_rigid_body -> position.y, selected_rigid_body -> position.z,
        selected_object_speed
    ); gtk_label_set_text (GTK_LABEL (debug_information_label), information_text_buffer);
}
