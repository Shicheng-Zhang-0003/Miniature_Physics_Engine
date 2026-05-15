#include "overlay.h"
#include "../../stage1/master_header.h"
#include "../interaction/spawner/object_spawner.h"
#include <stdint.h>
#include <stdio.h>
static GtkWidget *debug_information_label = NULL;
static GtkWidget *menu_label = NULL;
static GtkWidget *spawner_menu_label = NULL;
static GtkWidget *velocity_menu_label = NULL;
static GtkWidget *object_menu_label = NULL;
extern float world_gravity_y;
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
    //Spawn Menu Labelling
    spawner_menu_label = gtk_label_new ("");
    gtk_widget_set_halign (spawner_menu_label, GTK_ALIGN_CENTER);
    gtk_widget_set_valign (spawner_menu_label, GTK_ALIGN_CENTER);
    gtk_overlay_add_overlay (GTK_OVERLAY (ui_overlay_container), spawner_menu_label);
    gtk_widget_hide (spawner_menu_label);
    //Velocity Related Menu Labelling
    velocity_menu_label = gtk_label_new ("");
    gtk_widget_set_halign (velocity_menu_label, GTK_ALIGN_CENTER);
    gtk_widget_set_valign (velocity_menu_label, GTK_ALIGN_CENTER);
    gtk_overlay_add_overlay (GTK_OVERLAY (ui_overlay_container), velocity_menu_label);
    gtk_widget_hide (velocity_menu_label);
    //Object Selection Menu Labelling
    object_menu_label = gtk_label_new ("");
    gtk_widget_set_halign (object_menu_label, GTK_ALIGN_CENTER);
    gtk_widget_set_valign (object_menu_label, GTK_ALIGN_CENTER);
    gtk_overlay_add_overlay (GTK_OVERLAY (ui_overlay_container), object_menu_label);
    gtk_widget_hide (object_menu_label);
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
        if (main_inputs.spawner_menu_level == 0) {gtk_widget_hide (spawner_menu_label);}
        else {
            char spawner_text [256];
            if (main_inputs.spawner_menu_level == 1) {snprintf (spawner_text, sizeof (spawner_text), "-- Spawner Menu --\n1: Sphere\n2: Cube");}
            else if (main_inputs.spawner_menu_level == 2) {snprintf (spawner_text, sizeof (spawner_text), "-- Sphere Settings --\n1: Mass\n2: Radius");}
            else if (main_inputs.spawner_menu_level == 3) {snprintf (spawner_text, sizeof (spawner_text), "-- Mass Settings --\nCurrent Mass: %.2f kg\n\nUp/Down: +/- 0.01f\nEnter: Save and Close", spawn_mass);}
            else if (main_inputs.spawner_menu_level == 4) {snprintf (spawner_text, sizeof (spawner_text), "-- Radius Settings --\nCurrent Radius: %.2f m\n\nUp/Down: +/- 0.01f\nEnter: Save and Close", spawn_radius);}
            else if (main_inputs.spawner_menu_level == 5) {snprintf (spawner_text, sizeof (spawner_text), "-- Cube Settings --\n1: Mass\n2: Size");}
            else if (main_inputs.spawner_menu_level == 6) {snprintf (spawner_text, sizeof (spawner_text), "-- Cube Mass --\nCurrent Mass: %.2f kg\n\nUp/Down: +/- 0.01f\nEnter: Save and Close", spawn_cube_mass);}
            else if (main_inputs.spawner_menu_level == 7) {snprintf (spawner_text, sizeof (spawner_text), "-- Cube Size --\nCurrent Size: %.2f m\n\nUp/Down: +/- 0.01f\nEnter: Save and Close", spawn_cube_extent);}
            gtk_label_set_text (GTK_LABEL (spawner_menu_label), spawner_text);
            gtk_widget_show (spawner_menu_label);
        }
    } if (velocity_menu_label) {
        if (main_inputs.velocity_menu_level == 0) {gtk_widget_hide (velocity_menu_label);}
        else {
            char velocity_text [256];
            if (main_inputs.velocity_menu_level == 1) {snprintf (velocity_text, sizeof (velocity_text), "-- User Mechanics --\n1: Spawning\n2: Viewpoint\n3: World Modification");}
            else if (main_inputs.velocity_menu_level == 2) {snprintf (velocity_text, sizeof (velocity_text), "-- Spawning Mechanics --\n1: Launch Velocity\n2: Object Friction");}
            else if (main_inputs.velocity_menu_level == 3) {snprintf (velocity_text, sizeof (velocity_text), "-- Launch Velocity --\nCurrent: %.2f m/s\n\nUp/Down: +/- 0.01f\nEnter: Save and Close", spawn_speed);}
            else if (main_inputs.velocity_menu_level == 4) {snprintf (velocity_text, sizeof (velocity_text), "-- Object Friction --\nStatic (u_s): %.2f | Kinetic (u_k): %.2f\n\nUp/Down: +/- 0.01f (u_k)\nEnter: Save and Close", friction_static, friction_kinetic);}
            else if (main_inputs.velocity_menu_level == 20) {snprintf (velocity_text, sizeof (velocity_text), "-- World Modification --\n1: Gravity\n2: Air Resistance\n3: Surface Friction");}
            else if (main_inputs.velocity_menu_level == 21) {snprintf (velocity_text, sizeof (velocity_text), "-- World Gravity --\nCurrent: %.2f m/s^2\n\nUp/Down: +/- 0.01f\nEnter: Save and Close", world_gravity_y);}
            else if (main_inputs.velocity_menu_level == 22) {snprintf (velocity_text, sizeof (velocity_text), "-- Air Resistance (Drag) --\nCurrent Coeff: %.2f\n\nUp/Down: +/- 0.01f\nEnter: Save and Close", world_drag_coefficient);}
            else if (main_inputs.velocity_menu_level == 23) {snprintf (velocity_text, sizeof (velocity_text), "-- Surface Friction (Floor) --\nStatic (u_s): %.2f | Kinetic (u_k): %.2f\n\nUp/Down: +/- 0.01f (u_k)\nEnter: Save and Close", world_surface_friction_static, world_surface_friction_kinetic);}
            else if (main_inputs.velocity_menu_level == 10) {snprintf (velocity_text, sizeof (velocity_text), "-- Viewpoint Mechanics --\n1: Movement Speed");}
            else if (main_inputs.velocity_menu_level == 11) {snprintf (velocity_text, sizeof (velocity_text), "-- Camera Speed --\nCurrent: %.2f units/s\n\nUp/Down: +/- 0.01f\nEnter: Save and Close", main_camera_fov.movement_speed);}
            gtk_label_set_text (GTK_LABEL (velocity_menu_label), velocity_text);
            gtk_widget_show (velocity_menu_label);
        }
    } if (object_menu_label) {
        if (main_inputs.object_menu_level == 0) {gtk_widget_hide (object_menu_label);}
        else {
            char object_text [512];
            rigidbody *target = &obj_per_scene [selected_object];
            if (main_inputs.object_menu_level == 1) {snprintf (object_text, sizeof (object_text), "-- Object %d Properties --\n1: Mass\n2: Radius\n3: Friction\n4: Immovable Toggle", selected_object);}
            else if (main_inputs.object_menu_level == 2) {snprintf (object_text, sizeof (object_text), "-- Mass Adjustment --\nCurrent: %.2f kg\n\nUp/Down: +/- 0.01f\nEnter: Save", target -> mass);}
            else if (main_inputs.object_menu_level == 3) {snprintf (object_text, sizeof (object_text), "-- Radius Adjustment --\nCurrent: %.2f m\n\nUp/Down: +/- 0.01f\nEnter: Save", target -> radius);}
            else if (main_inputs.object_menu_level == 4) {snprintf (object_text, sizeof (object_text), "-- Friction Adjustment --\nStatic (u_s): %.2f | Kinetic (u_k): %.2f\n\nUp/Down: +/- 0.01f (u_k)\nEnter: Save", target -> friction_static, target -> friction_kinetic);}
            else if (main_inputs.object_menu_level == 5) {snprintf (object_text, sizeof (object_text), "-- Immovable Status --\nCurrent: %s\n\nUp/Down: Toggle\nEnter: Save", target -> static_state ? "YES" : "NO");}
            gtk_label_set_text (GTK_LABEL (object_menu_label), object_text);
            gtk_widget_show (object_menu_label);
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
    vector3 global_gravity_acceleration = {0.0f, world_gravity_y, 0.0f};
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
