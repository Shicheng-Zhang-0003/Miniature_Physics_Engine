#ifndef input_control_h
#define input_control_h
#include <gtk/gtk.h>
#include <stdbool.h>
typedef struct {
    //Keyboard Movement Inputs
    bool w_key_pressed, a_key_pressed, s_key_pressed, d_key_pressed, space_key_pressed, shift_key_pressed, escape_key_pressed;
    //File Load Inputs
    bool is_menu_open;
    bool menu_1_pressed, menu_2_pressed, menu_3_pressed;
    //Spawn Object Status
    int spawner_menu_level;
    bool up_arrow_pressed, down_arrow_pressed;
    //Mouse Status Inputs
    bool is_mouse_locked, is_debug_mode_active;
    bool left_mouse_button_clicked, right_mouse_button_clicked;
    float mouse_delta_x, mouse_delta_y;
} input_status;
//Initialise input state to zeroing
void initialise_input (input_status *input_state);
gboolean on_keypress (GtkWidget *widget, GdkEventKey *event, gpointer user_data_stored);
gboolean on_key_released (GtkWidget *widget, GdkEventKey *event, gpointer user_data_stored);
gboolean on_mouse_movements (GtkWidget *widget, GdkEventMotion *event, gpointer user_data_stored);
gboolean on_button_press (GtkWidget *widget, GdkEventButton *event, gpointer user_data_stored);
gboolean on_button_release (GtkWidget *widget, GdkEventButton *event, gpointer user_data_stored);
#endif
