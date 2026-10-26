#ifndef module_config_menu_h
#define module_config_menu_h
#include <gtk/gtk.h>
#include <stdbool.h>
/* Module config menu (5 key): the loaded ecosystem bundle exposes numeric
 * parameters through the module contract mpe_module_config_count/entry/set
 * (see module_config.c on the MFS side); the menu lists and edits them the
 * way the engine config menu (6 key) lists engine parameters.
 * level 0 = closed
 * level 1 = parameter list
 * selected = index pending an edit dialog (-1 none)
 * Symbols resolve per call (no cached dlsym pointers held across ticks) so
 * mod load/unload changes what the menu offers immediately. */
bool module_config_menu_is_open (void);
bool module_config_menu_available (void);
void module_config_menu_open (void);
void module_config_menu_close (void);
void module_config_menu_key_press (int key_number);
void module_config_menu_update (GtkWidget *parent_window);
void module_config_menu_render (char *buffer, size_t buffer_size);
#endif
