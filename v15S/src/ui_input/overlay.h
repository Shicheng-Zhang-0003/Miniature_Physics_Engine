#ifndef overlay_h
#define overlay_h
#include <gtk/gtk.h>
#include "../core/math3d.h"

GtkWidget *overlay_initialise(GtkWidget *gl_drawing_area_widget);
void overlay_update(void);
#endif
