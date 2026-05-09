#include "input_control.h"
#include "../camera/camera.h"
#include "../../stage3/input_extension/mouse_lock.h"
#include <gtk/gtk.h>
#include <gdk/gdkkeysyms.h>
#include <stdbool.h>
extern camera main_camera_fov;
extern input_status main_inputs;
void initialise_input (input_status *input) {
    //Keyboard
    input -> w_key = false;
    input -> a_key = false;
    input -> s_key = false;
    input -> d_key = false;
    input -> e_key = false;
    input -> f_key = false;
    input -> space_key = false;
    input -> esc_key = false;
    //File
    input -> r_key = false;
    input -> l_key = false;
    //Object Joining
    input -> j_key = false;
    input -> x_key = false;
    //Camera Look
    input -> cam_up = false;
    input -> cam_down = false;
    input -> cam_left = false;
    input -> cam_right = false;
    //Mouse
    input -> mouse_locked = false;
    input -> left_click = false;
    input -> right_click = false;
    input -> mouse_dx = 0.0;
    input -> mouse_dy = 0.0;
} gboolean on_keypress (GtkWidget *widget, GdkEventKey *event, gpointer user_data_stored) {
    input_status *input = (input_status *) user_data_stored;
    if (event -> keyval == GDK_KEY_w) {input -> w_key = true;}
    if (event -> keyval == GDK_KEY_a) {input -> a_key = true;}
    if (event -> keyval == GDK_KEY_s) {input -> s_key = true;}
    if (event -> keyval == GDK_KEY_d) {input -> d_key = true;}
    if (event -> keyval == GDK_KEY_e) {input -> e_key = true;}
    if (event -> keyval == GDK_KEY_f) {input -> f_key = true;}
    if (event -> keyval == GDK_KEY_r) {input -> r_key = true;}
    if (event -> keyval == GDK_KEY_o) {input -> l_key = true;}  // rebound from L
    if (event -> keyval == GDK_KEY_g) {input -> j_key = true;}  // rebound from J
    if (event -> keyval == GDK_KEY_x) {input -> x_key = true;}
    if (event -> keyval == GDK_KEY_space) {input -> space_key = true;}
    if (event -> keyval == GDK_KEY_Escape) {input -> esc_key = true;}
    //Camera Look
    if (event -> keyval == GDK_KEY_i) {input -> cam_up = true;}
    if (event -> keyval == GDK_KEY_k) {input -> cam_down = true;}
    if (event -> keyval == GDK_KEY_j) {input -> cam_left = true;}
    if (event -> keyval == GDK_KEY_l) {input -> cam_right = true;}
    return FALSE;
} gboolean on_key_released (GtkWidget *widget, GdkEventKey *event, gpointer user_data_stored) {
    input_status *input = (input_status *) user_data_stored;
    if (event -> keyval == GDK_KEY_w) {input -> w_key = false;}
    if (event -> keyval == GDK_KEY_a) {input -> a_key = false;}
    if (event -> keyval == GDK_KEY_s) {input -> s_key = false;}
    if (event -> keyval == GDK_KEY_d) {input -> d_key = false;}
    if (event -> keyval == GDK_KEY_e) {input -> e_key = false;}
    if (event -> keyval == GDK_KEY_f) {input -> f_key = false;}
    if (event -> keyval == GDK_KEY_r) {input -> r_key = false;}
    if (event -> keyval == GDK_KEY_o) {input -> l_key = false;}  // rebound from L
    if (event -> keyval == GDK_KEY_g) {input -> j_key = false;}  // rebound from J
    if (event -> keyval == GDK_KEY_x) {input -> x_key = false;}
    if (event -> keyval == GDK_KEY_space) {input -> space_key = false;}
    if (event -> keyval == GDK_KEY_Escape) {input -> esc_key = false;}
    //Camera Look
    if (event -> keyval == GDK_KEY_i) {input -> cam_up = false;}
    if (event -> keyval == GDK_KEY_k) {input -> cam_down = false;}
    if (event -> keyval == GDK_KEY_j) {input -> cam_left = false;}
    if (event -> keyval == GDK_KEY_l) {input -> cam_right = false;}
    return FALSE;
} gboolean on_mouse_movements (GtkWidget *widget, GdkEventMotion *event, gpointer user_data_stored) {
    input_status *input = &main_inputs;
    if (input -> mouse_locked) {
        static int last_warp_x = -1;
        static int last_warp_y = -1;
        int x = (int) (event -> x);
        int y = (int) (event -> y);
        // If this event is exactly where we just warped to, ignore it to prevent lag/oscillation
        if (((x == last_warp_x) && (y == last_warp_y))) {return FALSE;}
        int width = gtk_widget_get_allocated_width (widget);
        int height = gtk_widget_get_allocated_height (widget);
        int centerX = (width / 2);
        int centerY = (height / 2);
        int dx = (x - centerX);
        int dy = (centerY - y);
        if (((dx != 0) || (dy != 0))) {
            input -> mouse_dx += (float) (dx);
            input -> mouse_dy += (float) (dy);
            last_warp_x = centerX;
            last_warp_y = centerY;
            mouse_lock_reset_centre (widget);
        }
    } return FALSE;
} gboolean on_button_press (GtkWidget *widget, GdkEventButton *event, gpointer user_data_stored) {
    input_status *input = (input_status *) user_data_stored;
    if (event -> button == 1) {input -> left_click = true;}
    if (event -> button == 3) {input -> right_click = true;}
    if (! (input -> mouse_locked)) {
        // Reset deltas to prevent sudden jumps on lock
        input -> mouse_dx = 0.0;
        input -> mouse_dy = 0.0;
        mouse_lock_enable (gtk_widget_get_toplevel (widget));
        input -> mouse_locked = true;
    } return FALSE;
} gboolean on_button_release (GtkWidget *widget, GdkEventButton *event, gpointer user_data_stored) {
    input_status *input = (input_status *) user_data_stored;
    if (event -> button == 1) {input -> left_click = false;}
    if (event -> button == 3) {input -> right_click = false;}
    return FALSE;
}
