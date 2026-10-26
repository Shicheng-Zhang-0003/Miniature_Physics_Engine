#ifndef input_control_h
#define input_control_h
/* GTK4-PREP: clean state lives in input_state.h (no GUI). UI handlers keep gtk. */
#include "input_state.h"
#include <gtk/gtk.h>
#include <stdbool.h>
// Initialise input state to zeroing
void initialize_input (input_status *input_state);
void input_control_attach_controllers (GtkWidget *widget, gpointer user_data);
gboolean on_keypress (GtkEventControllerKey *ctrl, guint keyval, guint keycode, GdkModifierType state,
                      gpointer user_data_stored);
gboolean on_key_released (GtkEventControllerKey *ctrl, guint keyval, guint keycode, GdkModifierType state,
                          gpointer user_data_stored);
gboolean on_mouse_movements (GtkEventControllerMotion *ctrl, double x, double y, gpointer user_data_stored);
/* Per-frame drain of Wayland relative deltas into mouse_delta_x/y. Must be
 * called before simulation_camera_tick(), because a locked pointer produces
 * no GTK motion events. */
void input_control_poll_locked_mouse (void);
gboolean on_button_press (GtkGestureClick *gest, int n_press, double x, double y, gpointer user_data_stored);
gboolean on_button_release (GtkGestureClick *gest, int n_press, double x, double y, gpointer user_data_stored);
gboolean on_focus_out (GtkEventControllerFocus *ctrl, gpointer user_data_stored);
#endif
