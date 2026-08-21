#ifndef debug_terminal_h
#define debug_terminal_h

#include <gtk/gtk.h>
#include <stdbool.h>

void debug_terminal_open (GtkWidget *parent_window);
void debug_terminal_sync_mode (void);
void debug_terminal_focus_entry (void);
bool debug_terminal_is_open (void);

#endif
