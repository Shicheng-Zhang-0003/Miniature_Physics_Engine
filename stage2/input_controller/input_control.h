#ifndef input_control_h
#define input_control_h
#include <gtk/gtk.h>
#include <stdbool.h>
typedef struct {
    //Keyboard Movement Inputs
    bool w_key, a_key, s_key, d_key, e_key, f_key, space_key;
    //Load File Inputs
    bool r_key, l_key; //Save, Load
    //Join and Disassociate individual objects
    bool j_key, x_key;
    //IJKL Mouse Substitute
    bool cam_up, cam_down, cam_left, cam_right;
    //Mouse Status Inputs
    bool mouse_1;
} input_status;
//Initialise input state to zeroing
void initialise_input (input_status *input);
gboolean on_keypress (GtkWidget *widget, GdkEventKey *event, gpointer user_data_stored);
gboolean on_key_released (GtkWidget *widget, GdkEventKey *event, gpointer user_data_stored);
gboolean on_mouse_movements (GtkWidget *widget, GdkEventMotion *event, gpointer user_data_stored);
#endif
