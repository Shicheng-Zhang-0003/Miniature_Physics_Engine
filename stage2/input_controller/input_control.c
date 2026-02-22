#include "input_control.h"
#include "../camera/camera.h"
#include <gtk/gtk.h>
#include <gdk/gdkkeysyms.h>
void initialise_input (input_status *input) {
    //Keyboard
    input->w_key = false;
    input->a_key = false;
    input->s_key = false;
    input->d_key = false;
    //Mouse
    input->mouse_1 = true;
    input->last_x_input = 400.0; //800 x 600 Video Rendering Centre
    input->last_y_input = 300.0;
} gboolean on_keypress (GtkWidget *widget, GdkEventKey *event, gpointer user_data_stored) {
    input_status *input = (input_status *) user_data_stored;
    //Keystroke Recognition
    if (event->keyval == GDK_KEY_w) {input->w_key = true;} 
    if (event->keyval == GDK_KEY_a) {input->a_key = true;}
    if (event->keyval == GDK_KEY_s) {input->s_key = true;}
    if (event->keyval == GDK_KEY_d) {input->d_key = true;}
    return false;
} gboolean on_key_released (GtkWidget *widget, GdkEvenetKey *event, gpointer user_data_stored) {
    input_status *input = (input_status *) user_data_stored; 
    //Keystroke Recognition
    if (event->keyval == GDK_KEY_w) {input->w_key = false;}
    if (event->keyval == GDK_KEY_a) {input->a_key = false;}
    if (event->keyval == GDK_KEY_s) {input->s_key = false;}
    if (event->keyval == GDK_KEY_d) {input->d_key = false;}
    return false;
} gboolean on_mouse_movements (GtkWidget *widget, GdkEventKey *event, gpointer user_data_stored) {
    //Pass a array containing both camera and the input status
    //user_data --> global camera view
    extern camera_t main_camera_fov;
    extern input_status main_inputs;
    if (main_inputs.mouse_1) {
        main_inputs.last_x_input = event->x;
        main_inputs.last_y_input = event->y;
        main_inputs.mouse_1 = false;
    } float x_offsetting = event->x - main_inputs.last_x_input;
    float y_offsetting = main_inputs.last_y_input - event->y; //Y coords top to bot
    main_inputs.last_x_input = event->x;
    main_inputs.last_y_input = event->y;
    x_offsetting *= main_camera_fov.sensitivity_mouse;
    y_offsetting *= main_camera_fov.sensitivity_mouse;
    main_camera_fov.yaw += x_offsetting;
    main_camera_fov.pitch += y_offsetting;
    //Ensure Pitch Stays Relative to current Screen View
    if (main_camera_fov.pitch > 89.0) {main_camera_fov.pitch = 89.0;}
    if (main_camera_fov.pitch < -89.0) {main_camera_fov.pitch = -89.0;}
    //Calculate Front, Right/Side, Up/Vertical Vectors
    camera_update_vector_input (&main_camera_fov);
    return FALSE;
}
