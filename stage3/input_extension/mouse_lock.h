#ifndef mouse_lock_h
#define mouse_lock_h
#include <gtk/gtk.h>
#include <gdk/gdk.h>
//Lock Cursor --> Actual Object and POV Movement Inside Engine
void mouse_lock_enable (GtkWidget *window);
//Disable Cursor Lock
void mouse_lock_disable (GtkWidget *window);
//Cursor Back to window center
void mouse_lock_reset_centre (GtkWidget *window);
#endif
