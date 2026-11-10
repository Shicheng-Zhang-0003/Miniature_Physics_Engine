#ifndef resize_panel_h
#define resize_panel_h
#include <gtk/gtk.h>
#include <stdbool.h>
/* SANDBOX S4a: a small always-available editor window (the debug-terminal
 * pattern: plain GtkWindow, transient for the main window, never modal)
 * with a slider per size dimension of the SELECTED body. Dragging a
 * slider resizes the body live (inertia + contact cache + wake follow,
 * the exact editor radius-edit tail). Follows the selection: open the
 * panel once, then right-click different bodies to resize each in turn. */
void resize_panel_toggle (GtkWidget *parent_window);
/* Re-sync sliders/title to the current selection (cheap every-frame call;
 * only acts when the panel is open and the selected id changed). */
void resize_panel_sync (void);
bool resize_panel_is_open (void);
#endif /* resize_panel_h */
