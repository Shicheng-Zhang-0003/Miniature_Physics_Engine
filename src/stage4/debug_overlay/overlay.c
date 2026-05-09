#include "overlay.h"
#include "../../stage1/master_header.h"
#include "../interaction/spawner/object_spawner.h"
#include <stdint.h>
#include <stdio.h>
static GtkWidget *debug_information_label = NULL;
static GtkWidget *menu_label = NULL;
static GtkWidget *spawner_menu_label = NULL;
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
    menu_label = gtk_label_new ("");
    gtk_widget_set_halign (menu_label, GTK_ALIGN_CENTER);
    gtk_widget_set_valign (menu_label, GTK_ALIGN_CENTER);
    gtk_overlay_add_overlay (GTK_OVERLAY (ui_overlay_container), menu_label);
    gtk_widget_hide (menu_label);
    spawner_menu_label = gtk_label_new ("");
    gtk_widget_set_halign (spawner_menu_label, GTK_ALIGN_CENTER);
    gtk_widget_set_valign (spawner_menu_label, GTK_ALIGN_CENTER);
    gtk_overlay_add_overlay (GTK_OVERLAY (ui_overlay_container), spawner_menu_label);
    gtk_widget_hide (spawner_menu_label);
    return ui_overlay_container;
} void overlay_update (void) {
    if (menu_label) {
        if (main_inputs.is_menu_open) {
            gtk_label_set_text (GTK_LABEL (menu_label), "1: Save current state\n2: Load previous state\n3: Exit");
            gtk_widget_show (menu_label);
        } else {
            gtk_label_set_text (GTK_LABEL (menu_label), "");
            gtk_widget_hide (menu_label);
        }
    } if (spawner_menu_label) {
        if (main_inputs.spawner_menu_level == 0) {
            gtk_widget_hide (spawner_menu_label);
        } else {
            char spawner_text [256];
            if (main_inputs.spawner_menu_level == 1) {
                snprintf (spawner_text, sizeof (spawner_text), "-- Spawner Menu --\n1: Sphere\n2: Cube (coming soon)\n3: Rectangular Prism (coming soon)");
            } else if (main_inputs.spawner_menu_level == 2) {
                snprintf (spawner_text, sizeof (spawner_text), "-- Sphere Settings --\n1: Mass\n2: Radius");
            } else if (main_inputs.spawner_menu_level == 3) {
                snprintf (spawner_text, sizeof (spawner_text), "-- Mass --\nCurrent: %.2f kg\nUp Arrow: +0.5 | Down Arrow: -0.5", spawn_mass);
            } else if (main_inputs.spawner_menu_level == 4) {
                snprintf (spawner_text, sizeof (spawner_text), "-- Radius --\nCurrent: %.2f m\nUp Arrow: +0.1 | Down Arrow: -0.1", spawn_radius);
            }
            gtk_label_set_text (GTK_LABEL (spawner_menu_label), spawner_text);
            gtk_widget_show (spawner_menu_label);
        }
    } if (!debug_information_label) {return;}
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
    state_energy calculated_energy = force_to_system_energy_amount (selected_rigid_body, global_gravity_acceleration);
    float selected_object_speed = vector3_length (selected_rigid_body -> velocity);
    if (main_inputs.is_debug_mode_active) {
        snprintf (information_text_buffer, sizeof (information_text_buffer),
            "[%s] | Object %d | Position (%.1f,%.1f,%.1f) | Velocity AVG %.2f | EkL: %.2fJ | EkR: %.2fJ | EpG: %.2fJ | EpS: %.2fJ | Em: %.2fJ",
            game_mode_text, selected_object,
            selected_rigid_body -> position.x, selected_rigid_body -> position.y, selected_rigid_body -> position.z,
            selected_object_speed,
            calculated_energy.linear_kinetic_energy,
            calculated_energy.rotational_kinetic_energy,
            calculated_energy.gravitational_potential_energy,
            calculated_energy.spring_potential_energy,
            calculated_energy.mechanical_energy
        );
    } else {
        snprintf (information_text_buffer, sizeof (information_text_buffer), "[%s] | Object %d | Position (%.1f, %.1f, %.1f) | Velocity AVG %.2f | Space: impulse | L-Click: remove", game_mode_text, selected_object,
            selected_rigid_body -> position.x, selected_rigid_body -> position.y, selected_rigid_body -> position.z,
            selected_object_speed
        );
    } gtk_label_set_text (GTK_LABEL (debug_information_label), information_text_buffer);
}
