#ifndef input_control_h
#define input_control_h
#include <gtk/gtk.h>
#include <stdbool.h>
typedef struct {
    //Keyboard Movement Inputs
    bool w_key, a_key, s_key, d_key, e_key, f_key, space_key;
    //Mouse Status Inputs
    bool mouse_1;
    double last_x_input, last_y_input;
} input_status;
//Initialise input state to zeroing
void initialise_input (input_status *input);
gboolean on_keypress (GtkWidget *widget, GdkEventKey *event, gpointer user_data_stored);
gboolean on_key_released (GtkWidget *widget, GdkEventKey *event, gpointer user_data_stored);
gboolean on_mouse_movements (GtkWidget *widget, GdkEventMotion *event, gpointer user_data_stored);
#endif
