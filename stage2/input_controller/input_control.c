#include "input_control.h"
#include "../camera/camera.h"
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
    input -> mouse_1 = true;
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
    //Camera Look
    if (event -> keyval == GDK_KEY_i) {input -> cam_up = false;}
    if (event -> keyval == GDK_KEY_k) {input -> cam_down = false;}
    if (event -> keyval == GDK_KEY_j) {input -> cam_left = false;}
    if (event -> keyval == GDK_KEY_l) {input -> cam_right = false;}
    return FALSE;
} gboolean on_mouse_movements (GtkWidget *widget, GdkEventMotion *event, gpointer user_data_stored) {
    // Mouse no longer controls camera view, IJKL
    return FALSE;
}
