#include "input_control.h"
#include "../camera/camera.h"
#include "../../stage3/input_extension/mouse_lock.h"
#include <gtk/gtk.h>
#include <gdk/gdkkeysyms.h>
#include <stdbool.h>
extern camera main_camera_fov;
extern input_status main_inputs;
void initialise_input (input_status *input_state) {
    //Keyboard
    input_state -> w_key_pressed = false;
    input_state -> a_key_pressed = false;
    input_state -> s_key_pressed = false;
    input_state -> d_key_pressed = false;
    input_state -> space_key_pressed = false;
    input_state -> shift_key_pressed = false;
    input_state -> escape_key_pressed = false;
    //Menu
    input_state -> is_menu_open = false;
    input_state -> menu_1_pressed = false;
    input_state -> menu_2_pressed = false;
    input_state -> menu_3_pressed = false;
    //Spawn
    input_state -> spawner_menu_level = 0;
    input_state -> velocity_menu_level = 0;
    input_state -> object_menu_level = 0;
    input_state -> up_arrow_pressed = false;
    input_state -> down_arrow_pressed = false;
    input_state -> enter_key_pressed = false;
    input_state -> e_key_pressed = false;
    //Mouse
    input_state -> is_mouse_locked = false;
    input_state -> is_debug_mode_active = false;
    input_state -> left_mouse_button_clicked = false;
    input_state -> right_mouse_button_clicked = false;
    input_state -> mouse_delta_x = 0.0f;
    input_state -> mouse_delta_y = 0.0f;
} gboolean on_keypress (GtkWidget *widget, GdkEventKey *event, gpointer user_data_stored) {
    input_status *input_state = (input_status *) user_data_stored;
    if (event -> keyval == GDK_KEY_w) {input_state -> w_key_pressed = true;}
    if (event -> keyval == GDK_KEY_a) {input_state -> a_key_pressed = true;}
    if (event -> keyval == GDK_KEY_s) {input_state -> s_key_pressed = true;}
    if (event -> keyval == GDK_KEY_d) {input_state -> d_key_pressed = true;}
    if (event -> keyval == GDK_KEY_e) {input_state -> e_key_pressed = true;}
    if (event -> keyval == GDK_KEY_9) {input_state -> spawner_menu_level = 0; input_state -> velocity_menu_level = 0; input_state -> object_menu_level = 0; input_state -> is_menu_open = !(input_state -> is_menu_open);}
    if (event -> keyval == GDK_KEY_8) {input_state -> is_menu_open = false; input_state -> velocity_menu_level = 0; input_state -> object_menu_level = 0; if (input_state -> spawner_menu_level > 0) {input_state -> spawner_menu_level = 0;} else {input_state -> spawner_menu_level = 1;}}
    if (event -> keyval == GDK_KEY_7) {input_state -> is_menu_open = false; input_state -> spawner_menu_level = 0; input_state -> object_menu_level = 0; if (input_state -> velocity_menu_level > 0) {input_state -> velocity_menu_level = 0;} else {input_state -> velocity_menu_level = 1;}}
    if (input_state -> is_menu_open) {
        if (event -> keyval == GDK_KEY_1) {input_state -> menu_1_pressed = true;}
        if (event -> keyval == GDK_KEY_2) {input_state -> menu_2_pressed = true;}
        if (event -> keyval == GDK_KEY_3) {input_state -> menu_3_pressed = true;}
    } // Spawner Menu Navigation
    if (input_state -> spawner_menu_level == 1) {
        if (event -> keyval == GDK_KEY_1) {input_state -> spawner_menu_level = 2;} // Sphere
        if (event -> keyval == GDK_KEY_2) {input_state -> spawner_menu_level = 5;} // Cube  (NEW)
    } else if (input_state -> spawner_menu_level == 2) {  // Sphere submenu
        if (event -> keyval == GDK_KEY_1) {input_state -> spawner_menu_level = 3;} // Mass
        if (event -> keyval == GDK_KEY_2) {input_state -> spawner_menu_level = 4;} // Radius
    } else if (input_state -> spawner_menu_level == 5) {  // Cube submenu (NEW)
        if (event -> keyval == GDK_KEY_1) {input_state -> spawner_menu_level = 6;} // Mass
        if (event -> keyval == GDK_KEY_2) {input_state -> spawner_menu_level = 7;} // Size
    } else if (input_state -> spawner_menu_level == 3 || input_state -> spawner_menu_level == 4) {
        // Sphere settings
        if (event -> keyval == GDK_KEY_Up)   {input_state -> up_arrow_pressed = true;}
        if (event -> keyval == GDK_KEY_Down) {input_state -> down_arrow_pressed = true;}
        if ((event -> keyval == GDK_KEY_Return) || (event -> keyval == GDK_KEY_KP_Enter)) {input_state -> enter_key_pressed = true;}
    } else if (input_state -> spawner_menu_level == 6 || input_state -> spawner_menu_level == 7) {
        // Cube settings (NEW)
        if (event -> keyval == GDK_KEY_Up)   {input_state -> up_arrow_pressed = true;}
        if (event -> keyval == GDK_KEY_Down) {input_state -> down_arrow_pressed = true;}
        if ((event -> keyval == GDK_KEY_Return) || (event -> keyval == GDK_KEY_KP_Enter)) {input_state -> enter_key_pressed = true;}
    } // Velocity Menu Navigation (Renamed to User Mechanics)
    if (input_state -> velocity_menu_level == 1) {
        if (event -> keyval == GDK_KEY_1) {input_state -> velocity_menu_level = 2;} // Spawning Mechanics
        if (event -> keyval == GDK_KEY_2) {input_state -> velocity_menu_level = 10;} // Viewpoint Mechanics
        if (event -> keyval == GDK_KEY_3) {input_state -> velocity_menu_level = 20;} // World Modification
    } else if (input_state -> velocity_menu_level == 2) {
        if (event -> keyval == GDK_KEY_1) {input_state -> velocity_menu_level = 3;} // Launch Velocity
        if (event -> keyval == GDK_KEY_2) {input_state -> velocity_menu_level = 4;} // Object Friction
    } else if (input_state -> velocity_menu_level == 20) {
        if (event -> keyval == GDK_KEY_1) {input_state -> velocity_menu_level = 21;} // Gravity
        if (event -> keyval == GDK_KEY_2) {input_state -> velocity_menu_level = 22;} // Air Resistance
        if (event -> keyval == GDK_KEY_3) {input_state -> velocity_menu_level = 23;} // Surface Friction
    } else if (input_state -> velocity_menu_level == 10) {
        if (event -> keyval == GDK_KEY_1) {input_state -> velocity_menu_level = 11;} // Movement Speed
    } else if ((input_state -> velocity_menu_level == 3) || (input_state -> velocity_menu_level == 4) || input_state -> velocity_menu_level == 11 || input_state -> velocity_menu_level == 21 ||(input_state -> velocity_menu_level == 22) || (input_state -> velocity_menu_level == 23)) {
        if (event -> keyval == GDK_KEY_Up)   {input_state -> up_arrow_pressed = true;}
        if (event -> keyval == GDK_KEY_Down) {input_state -> down_arrow_pressed = true;}
        if ((event -> keyval == GDK_KEY_Return) || (event -> keyval == GDK_KEY_KP_Enter)) {input_state -> enter_key_pressed = true;}
    } // Selected Object Menu Navigation
    if (input_state -> object_menu_level == 1) {
        if (event -> keyval == GDK_KEY_1) {input_state -> object_menu_level = 2;} // Mass
        if (event -> keyval == GDK_KEY_2) {input_state -> object_menu_level = 3;} // Radius
        if (event -> keyval == GDK_KEY_3) {input_state -> object_menu_level = 4;} // Friction
        if (event -> keyval == GDK_KEY_4) {input_state -> object_menu_level = 5;} // Immovable
    } else if (input_state -> object_menu_level >= 2) {
        if (event -> keyval == GDK_KEY_Up)   {input_state -> up_arrow_pressed = true;}
        if (event -> keyval == GDK_KEY_Down) {input_state -> down_arrow_pressed = true;}
        if ((event -> keyval == GDK_KEY_Return) || (event -> keyval == GDK_KEY_KP_Enter)) {input_state -> enter_key_pressed = true;}
    } if (event -> keyval == GDK_KEY_space) {input_state -> space_key_pressed = true;}
    if (event -> keyval == GDK_KEY_Shift_L) {input_state -> shift_key_pressed = true;}
    if (event -> keyval == GDK_KEY_c || event -> keyval == GDK_KEY_C) {input_state -> c_key_pressed = true;}
    if (event -> keyval == GDK_KEY_Escape) {input_state -> escape_key_pressed = true;}
    if (event -> keyval == GDK_KEY_0) {input_state -> is_debug_mode_active = !input_state -> is_debug_mode_active;}
    return FALSE;
} gboolean on_key_released (GtkWidget *widget, GdkEventKey *event, gpointer user_data_stored) {
    input_status *input_state = (input_status *) user_data_stored;
    if (event -> keyval == GDK_KEY_w) {input_state -> w_key_pressed = false;}
    if (event -> keyval == GDK_KEY_a) {input_state -> a_key_pressed = false;}
    if (event -> keyval == GDK_KEY_s) {input_state -> s_key_pressed = false;}
    if (event -> keyval == GDK_KEY_d) {input_state -> d_key_pressed = false;}
    if (event -> keyval == GDK_KEY_e) {input_state -> e_key_pressed = false;}
    if (event -> keyval == GDK_KEY_space) {input_state -> space_key_pressed = false;}
    if (event -> keyval == GDK_KEY_Shift_L) {input_state -> shift_key_pressed = false;}
    if (event -> keyval == GDK_KEY_c || event -> keyval == GDK_KEY_C) {input_state -> c_key_pressed = false;}
    if (event -> keyval == GDK_KEY_Escape) {input_state -> escape_key_pressed = false;}
    return FALSE;
} gboolean on_mouse_movements (GtkWidget *widget, GdkEventMotion *event, gpointer user_data_stored) {
    (void) widget;
    (void) user_data_stored;
    input_status *input_state = &main_inputs;
    if (input_state -> is_mouse_locked) {
        static int last_warp_x = -1;
        static int last_warp_y = -1;
        int current_mouse_x = (int) (event -> x);
        int current_mouse_y = (int) (event -> y);
        // If this event is exactly where we just warped to, ignore it to prevent lag/oscillation
        if (((current_mouse_x == last_warp_x) && (current_mouse_y == last_warp_y))) {return FALSE;}
        int widget_width = gtk_widget_get_allocated_width (widget);
        int widget_height = gtk_widget_get_allocated_height (widget);
        int center_x = (widget_width / 2);
        int center_y = (widget_height / 2);
        int delta_x = (current_mouse_x - center_x);
        int delta_y = (center_y - current_mouse_y);
        if (((delta_x != 0) || (delta_y != 0))) {
            input_state -> mouse_delta_x += (float) (delta_x);
            input_state -> mouse_delta_y += (float) (delta_y);
            last_warp_x = center_x;
            last_warp_y = center_y;
            mouse_lock_reset_centre (widget);
        }
    } return FALSE;
} gboolean on_button_press (GtkWidget *widget, GdkEventButton *event, gpointer user_data_stored) {
    input_status *input_state = (input_status *) user_data_stored;
    if (event -> button == 1) {input_state -> left_mouse_button_clicked = true;}
    if (event -> button == 3) {input_state -> right_mouse_button_clicked = true;}
    if (! (input_state -> is_mouse_locked)) {
        // Reset deltas to prevent sudden jumps on lock
        input_state -> mouse_delta_x = 0.0f;
        input_state -> mouse_delta_y = 0.0f;
        mouse_lock_enable (gtk_widget_get_toplevel (widget));
        input_state -> is_mouse_locked = true;
    } return FALSE;
} gboolean on_button_release (GtkWidget *widget, GdkEventButton *event, gpointer user_data_stored) {
    input_status *input_state = (input_status *) user_data_stored;
    if (event -> button == 1) {input_state -> left_mouse_button_clicked = false;}
    if (event -> button == 3) {input_state -> right_mouse_button_clicked = false;}
    return FALSE;
}

