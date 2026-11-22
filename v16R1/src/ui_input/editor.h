#ifndef editor_h
#define editor_h
#include <gtk/gtk.h>
#include "../cad/cad_editor.h" /* MIG WP8: cad_design for CAD menus (CAD/MIG) */
/* CAD/MIG menus: page drivers live in cad_menus.{c,h}; level constants here
 * so both editor.c and cad_menus.c agree (engine derived — each number can
 * open a new menu, not engine identical). */
#define CAD_MENU_PAGE_SELECTION 7
#define CAD_MENU_PAGE_ADD       8
#define CAD_MENU_PAGE_GROUP_CLIPBOARD 9
void editor_update_menus (GtkWidget *parent_window);
void cad_menus_update (void);
void cad_menus_set_level (int level);
bool cad_menus_key_press (int keyval);
#endif
