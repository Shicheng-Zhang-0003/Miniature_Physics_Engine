#include "../mpe_engine.h"
#include "overlay.h"
#include "object_spawner.h"
#include <stdint.h>
#include <stdio.h>
static GtkWidget *debug_information_label = NULL;
static GtkWidget *crosshair_label = NULL;
static GtkWidget *menu_label = NULL;
static GtkWidget *spawner_menu_label = NULL;
static GtkWidget *velocity_menu_label = NULL;
static GtkWidget *object_menu_label = NULL;
extern input_status main_inputs;
extern camera main_camera_fov;
extern float world_gravity_y;
extern float world_drag_coefficient;
extern float world_surface_friction_static;
extern float world_surface_friction_kinetic;
extern rigidbody *obj_per_scene;
extern int object_count;
extern int selected_object;
extern float variable_change_rate;
extern float jump_height;
GtkWidget *overlay_initialise (GtkWidget *gl_drawing_area_widget) {
    //Debug Info
    GtkWidget *ui_overlay_container = gtk_overlay_new ();
    gtk_container_add (GTK_CONTAINER (ui_overlay_container), gl_drawing_area_widget);
    debug_information_label = gtk_label_new ("- Miniature Physics Engine v1.4 Alpha 2 -");
    gtk_widget_set_halign (debug_information_label, GTK_ALIGN_START);
    gtk_widget_set_valign (debug_information_label, GTK_ALIGN_START);
    gtk_overlay_add_overlay (GTK_OVERLAY (ui_overlay_container), debug_information_label);
    //Crosshair
    crosshair_label = gtk_label_new ("+");
    gtk_widget_set_halign (crosshair_label, GTK_ALIGN_CENTER);
    gtk_widget_set_valign (crosshair_label, GTK_ALIGN_CENTER);
    gtk_overlay_add_overlay (GTK_OVERLAY (ui_overlay_container), crosshair_label);
    gtk_widget_show (crosshair_label);
    //Combined menu
    menu_label = gtk_label_new ("");
    gtk_widget_set_halign (menu_label, GTK_ALIGN_CENTER);
    gtk_widget_set_valign (menu_label, GTK_ALIGN_CENTER);
    gtk_overlay_add_overlay (GTK_OVERLAY (ui_overlay_container), menu_label);
    gtk_widget_hide (menu_label);
    //Totoal spawner characteristics menu
    spawner_menu_label = gtk_label_new ("");
    gtk_widget_set_halign (spawner_menu_label, GTK_ALIGN_CENTER);
    gtk_widget_set_valign (spawner_menu_label, GTK_ALIGN_CENTER);
    gtk_overlay_add_overlay (GTK_OVERLAY (ui_overlay_container), spawner_menu_label);
    gtk_widget_hide (spawner_menu_label);
    //Spawn Velocity Change
    velocity_menu_label = gtk_label_new ("");
    gtk_widget_set_halign (velocity_menu_label, GTK_ALIGN_CENTER);
    gtk_widget_set_valign (velocity_menu_label, GTK_ALIGN_CENTER);
    gtk_overlay_add_overlay (GTK_OVERLAY (ui_overlay_container), velocity_menu_label);
    gtk_widget_hide (velocity_menu_label);
    //Object Individual Menu
    object_menu_label = gtk_label_new ("");
    gtk_widget_set_halign (object_menu_label, GTK_ALIGN_CENTER);
    gtk_widget_set_valign (object_menu_label, GTK_ALIGN_CENTER);
    gtk_overlay_add_overlay (GTK_OVERLAY (ui_overlay_container), object_menu_label);
    gtk_widget_hide (object_menu_label);
    return ui_overlay_container;
} void overlay_update (void) {
    float adjustment_increment = variable_change_rate;
    if (menu_label) {
        if (main_inputs.is_menu_open) {
            gtk_label_set_text (GTK_LABEL (menu_label), "1: Save current state\n2: Load previous state\n3: Exit");
            gtk_widget_show (menu_label);
        } else {gtk_widget_hide (menu_label);}
    } if (spawner_menu_label) {
        if (main_inputs.spawner_menu_level == 0) {gtk_widget_hide (spawner_menu_label);}
        else {
            char spawner_text [512];
            if (main_inputs.spawner_menu_level == 1) {
                const char *spawn_type_text;
                if (main_inputs.current_spawn_type == 0) {spawn_type_text = "Sphere";}
                else {spawn_type_text = "Cube";}
                snprintf (spawner_text, sizeof (spawner_text), "-- Spawner Menu --\n1: Sphere\n2: Cube\n3: Current Type: %s", spawn_type_text);
            } else if (main_inputs.spawner_menu_level == 2) {snprintf (spawner_text, sizeof (spawner_text), "-- Sphere Settings --\n1: Mass\n2: Radius");}
            else if (main_inputs.spawner_menu_level == 3) {snprintf (spawner_text, sizeof (spawner_text), "-- Mass Settings --\nCurrent Mass: %.2f kg\n\nUp/Down: +/- %.2f\nEnter: Save and Close", spawn_mass, adjustment_increment);}
            else if (main_inputs.spawner_menu_level == 4) {snprintf (spawner_text, sizeof (spawner_text), "-- Radius Settings --\nCurrent Radius: %.2f m\n\nUp/Down: +/- %.2f\nEnter: Save and Close", spawn_radius, adjustment_increment);}
            else if (main_inputs.spawner_menu_level == 5) {snprintf (spawner_text, sizeof (spawner_text), "-- Cube Settings --\n1: Mass\n2: Size");}
            else if (main_inputs.spawner_menu_level == 6) {snprintf (spawner_text, sizeof (spawner_text), "-- Cube Mass --\nCurrent Mass: %.2f kg\n\nUp/Down: +/- %.2f\nEnter: Save and Close", spawn_cube_mass, adjustment_increment);}
            else if (main_inputs.spawner_menu_level == 7) {snprintf (spawner_text, sizeof (spawner_text), "-- Cube Size --\nCurrent Size: %.2f m\n\nUp/Down: +/- %.2f\nEnter: Save and Close", spawn_cube_extent, adjustment_increment);}
            else if (main_inputs.spawner_menu_level == 8) {
                const char *spawn_type_text;
                if (main_inputs.current_spawn_type == 0) {spawn_type_text = "Sphere";}
                else {spawn_type_text = "Cube";}
                snprintf (spawner_text, sizeof (spawner_text), "-- Toggle Spawn Type --\nCurrent: %s\n\nUp/Down: Toggle\nEnter: Save and Close", spawn_type_text);
            } gtk_label_set_text (GTK_LABEL (spawner_menu_label), spawner_text);
            gtk_widget_show (spawner_menu_label);
        }
    } if (velocity_menu_label) {
        if (main_inputs.velocity_menu_level == 0) {gtk_widget_hide (velocity_menu_label);}
        else {
            char velocity_text [512];
            if (main_inputs.velocity_menu_level == 1) {snprintf (velocity_text, sizeof (velocity_text), "-- User Mechanics --\n1: Spawning\n2: Viewpoint\n3: World Modification");}
            else if (main_inputs.velocity_menu_level == 2) {snprintf (velocity_text, sizeof (velocity_text), "-- Spawning Mechanics --\n1: Launch Velocity\n2: Object Friction");}
            else if (main_inputs.velocity_menu_level == 3) {snprintf (velocity_text, sizeof (velocity_text), "-- Launch Velocity --\nCurrent: %.2f m/s\n\nUp/Down: +/- %.2f\nEnter: Save and Close", spawn_speed, adjustment_increment);}
            else if (main_inputs.velocity_menu_level == 4) {snprintf (velocity_text, sizeof (velocity_text), "-- Object Friction --\nStatic (u_s): %.2f | Kinetic (u_k): %.2f\n\nUp/Down: +/- %.2f (u_k)\nEnter: Save and Close", friction_static, friction_kinetic, adjustment_increment);}
            else if (main_inputs.velocity_menu_level == 10) {snprintf (velocity_text, sizeof (velocity_text), "-- Viewpoint Settings --\n1: Movement Speed\n2: Character Jump Height");}
            else if (main_inputs.velocity_menu_level == 11) {snprintf (velocity_text, sizeof (velocity_text), "-- Movement Speed --\nCurrent: %.2f m/s\n\nUp/Down: +/- %.2f\nEnter: Save and Close", main_camera_fov.movement_speed, adjustment_increment);}
            else if (main_inputs.velocity_menu_level == 12) {snprintf (velocity_text, sizeof (velocity_text), "-- Jump Height --\nCurrent: %.2f m\n\nUp/Down: +/- %.2f\nEnter: Save and Close", jump_height, adjustment_increment);}
            else if (main_inputs.velocity_menu_level == 20) {snprintf (velocity_text, sizeof (velocity_text), "-- World Modification --\n1: Gravity\n2: Air Resistance\n3: Surface Friction");}
            else if (main_inputs.velocity_menu_level == 21) {snprintf (velocity_text, sizeof (velocity_text), "-- World Gravity --\nCurrent: %.2f m/s^2\n\nUp/Down: +/- %.2f\nEnter: Save and Close", world_gravity_y, adjustment_increment);}
            else if (main_inputs.velocity_menu_level == 22) {snprintf (velocity_text, sizeof (velocity_text), "-- Air Resistance (Drag) --\nCurrent Coeff: %.2f\n\nUp/Down: +/- %.2f\nEnter: Save and Close", world_drag_coefficient, adjustment_increment * 0.01f);}
            else if (main_inputs.velocity_menu_level == 23) {snprintf (velocity_text, sizeof (velocity_text), "-- Surface Friction (Floor) --\nStatic (u_s): %.2f | Kinetic (u_k): %.2f\n\nUp/Down: +/- %.2f (u_k)\nEnter: Save and Close", world_surface_friction_static, world_surface_friction_kinetic, adjustment_increment);}
            gtk_label_set_text (GTK_LABEL (velocity_menu_label), velocity_text);
            gtk_widget_show (velocity_menu_label);
        }
    } if (object_menu_label) {
        if (main_inputs.object_menu_level == 0) {gtk_widget_hide (object_menu_label);}
        else {
            char object_text [512];
            rigidbody *target = &obj_per_scene [selected_object];
            if (main_inputs.object_menu_level == 1) {
                const char *type_name = (target -> type == object_sphere) ? "Sphere" : "Cube";
                int len = snprintf (object_text, sizeof (object_text),
                    "-- Object %d (%s) --\n1: Mass\n2: %s\n3: Friction\n4: Immovable Toggle\n5: Mark for Joint\n",
                    selected_object, type_name, (target -> type == object_sphere) ? "Radius" : "Radius (N/A)");
                if (main_inputs.marked_joint_object_index != -1 && main_inputs.marked_joint_object_index != selected_object) {
                    snprintf (object_text + len, sizeof (object_text) - len,
                        "6: Link Joint (from Obj %d)\n7: Colour Selection", main_inputs.marked_joint_object_index);
                } else {
                    snprintf (object_text + len, sizeof (object_text) - len,
                        "6: Colour Selection");
                }
            } else if (main_inputs.object_menu_level == 2) {snprintf (object_text, sizeof (object_text), "-- Mass Adjustment --\nCurrent: %.2f kg\n\nUp/Down: +/- %.2f\nEnter: Save", target -> mass, adjustment_increment);}
            else if (main_inputs.object_menu_level == 3) {snprintf (object_text, sizeof (object_text), "-- Radius Adjustment --\nCurrent: %.2f m\n\nUp/Down: +/- %.2f\nEnter: Save", target -> radius, adjustment_increment);}
            else if (main_inputs.object_menu_level == 4) {snprintf (object_text, sizeof (object_text), "-- Friction Adjustment --\nStatic (u_s): %.2f | Kinetic (u_k): %.2f\n\nUp/Down: +/- %.2f (u_k)\nEnter: Save", target -> friction_static, target -> friction_kinetic, adjustment_increment);}
            else if (main_inputs.object_menu_level == 5) {
                const char *static_status_text;
                if (target -> static_state) {static_status_text = "YES";}
                else {static_status_text = "NO";}
                snprintf (object_text, sizeof (object_text), "-- Immovable Status --\nCurrent: %s\n\nUp/Down: Toggle\nEnter: Save", static_status_text);
            } else if (main_inputs.object_menu_level == 8) {
                snprintf (object_text, sizeof (object_text),
                    "-- Preset Colours --\n1: Red\n2: Green\n3: Blue\n4: Orange\n5: Cyan\n6: Magenta\n7: Yellow\n8: White");
            } gtk_label_set_text (GTK_LABEL (object_menu_label), object_text);
            gtk_widget_show (object_menu_label);
        }
    } if (!debug_information_label) {return;}
    char information_text_buffer [1024];
    char game_mode_text [32];
    if (main_inputs.is_debug_mode_active) {snprintf (game_mode_text, sizeof (game_mode_text), "DEBUG MODE");}
    else {snprintf (game_mode_text, sizeof (game_mode_text), "GAME MODE");}
    if ((selected_object < 0) || (selected_object >= object_count)) {
        const char *spawn_type_text;
        if (main_inputs.current_spawn_type == 0) {spawn_type_text = "sphere";}
        else {spawn_type_text = "cube";}
        snprintf (information_text_buffer, sizeof (information_text_buffer),
                 "[%s] | No object selected | Shift: spawn %s | R-Click: select | 0: Toggle Mode | L/R Arr: change rate (%.2f)",
                 game_mode_text, spawn_type_text, variable_change_rate);
        gtk_label_set_text (GTK_LABEL (debug_information_label), information_text_buffer);
        return;
    } rigidbody *selected_rigid_body = &obj_per_scene [selected_object];
    float selected_object_speed = vector3_length (selected_rigid_body -> velocity);
    const char *object_type_text;
    if (selected_rigid_body -> type == object_sphere) {object_type_text = "Sphere";}
    else {object_type_text = "Cube";}
    const char *static_status_text;
    if (selected_rigid_body -> static_state) {static_status_text = "(Static)";}
    else {static_status_text = "(Dynamic)";}
    snprintf (information_text_buffer, sizeof (information_text_buffer),
            "[%s] | %s [%d] %s | Pos: (%.1f, %.1f, %.1f) | Speed: %.2f | E: Menu | F: Impulse | M-Click: remove",
            game_mode_text,
            object_type_text,
            selected_object,
            static_status_text,
            selected_rigid_body -> position.x, selected_rigid_body -> position.y, selected_rigid_body -> position.z,
            selected_object_speed
        );
    gtk_label_set_text (GTK_LABEL (debug_information_label), information_text_buffer);
}

