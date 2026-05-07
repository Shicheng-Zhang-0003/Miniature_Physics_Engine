#ifndef overlay_h
#define overlay_h
#include <gtk/gtk.h>
#include "../../stage1/master_header.h"
#include "../../stage2/master_header_2.h"
#include "../interaction/selector/object_selector.h"
//Call at init to set up widget for the overlay
GtkWidget *overlay_init (GtkWidget *gl_area);
//Call every physics increment to refresh values
void overlay_update (void);
#endif
