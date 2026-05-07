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
    //Mouse
    input -> mouse_1 = true;
    input -> last_x_input = 400.0; //800 x 600 Video Rendering Centre
    input -> last_y_input = 300.0;
} gboolean on_keypress (GtkWidget *widget, GdkEventKey *event, gpointer user_data_stored) {
    input_status *input = (input_status *) user_data_stored;
    //Keystroke Recognition
    if (event -> keyval == GDK_KEY_w) {input -> w_key = true;}
    if (event -> keyval == GDK_KEY_a) {input -> a_key = true;}
    if (event -> keyval == GDK_KEY_s) {input -> s_key = true;}
    if (event -> keyval == GDK_KEY_d) {input -> d_key = true;}
    if (event -> keyval == GDK_KEY_e) {input -> e_key = true;}
    if (event -> keyval == GDK_KEY_f) {input -> f_key = true;}
    if (event -> keyval == GDK_KEY_r) {input -> r_key = true;}
    if (event -> keyval == GDK_KEY_l) {input -> l_key = true;}
    if (event -> keyval == GDK_KEY_j) {input -> j_key = true;}
    if (event -> keyval == GDK_KEY_x) {input -> x_key = true;}
    if (event -> keyval == GDK_KEY_space) {input -> space_key = true;}
    return FALSE;
} gboolean on_key_released (GtkWidget *widget, GdkEventKey *event, gpointer user_data_stored) {
    input_status *input = (input_status *) user_data_stored;
    //Keystroke Recognition
    if (event -> keyval == GDK_KEY_w) {input -> w_key = false;}
    if (event -> keyval == GDK_KEY_a) {input -> a_key = false;}
    if (event -> keyval == GDK_KEY_s) {input -> s_key = false;}
    if (event -> keyval == GDK_KEY_d) {input -> d_key = false;}
    if (event -> keyval == GDK_KEY_e) {input -> e_key = false;}
    if (event -> keyval == GDK_KEY_f) {input -> f_key = false;}
    if (event -> keyval == GDK_KEY_r) {input -> r_key = false;}
    if (event -> keyval == GDK_KEY_l) {input -> l_key = false;}
    if (event -> keyval == GDK_KEY_j) {input -> j_key = false;}
    if (event -> keyval == GDK_KEY_x) {input -> x_key = false;}
    if (event -> keyval == GDK_KEY_space) {input -> space_key = false;}
    return FALSE;
} gboolean on_mouse_movements (GtkWidget *widget, GdkEventMotion *event, gpointer user_data_stored) {
    int window_width = gtk_widget_get_allocated_width (widget);
    int window_height = gtk_widget_get_allocated_height (widget);
    float x_centre = window_width / 2.0;
    float y_centre = window_height / 2.0;
    float x_offsetting = event -> x - x_centre;
    float y_offsetting = y_centre - event -> y; //Y coords top to bot
    // Only update if the mouse has actually moved away from the centre
    // This prevents spinning when the reset_centre call itself triggers a motion event
    if ((fabsf (x_offsetting) > 0.0) || (fabsf (y_offsetting) > 0.0)) {
        x_offsetting *= main_camera_fov.sensitivity_mouse;
        y_offsetting *= main_camera_fov.sensitivity_mouse;
        main_camera_fov.yaw += x_offsetting;
        main_camera_fov.pitch += y_offsetting;
        //Ensure Pitch Stays Relative to current Screen View
        if (main_camera_fov.pitch > 89.0) {main_camera_fov.pitch = 89.0;}
        if (main_camera_fov.pitch < -89.0) {main_camera_fov.pitch = -89.0;}
        //Calculate Front, Right/Side, Up/Vertical Vectors
        camera_update_vector_input (&main_camera_fov);
        // Reset mouse to centre immediately after processing movement to prevent hitting window borders
        mouse_lock_reset_centre (widget);
    } return FALSE;
}
