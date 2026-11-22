/* MIG WP8: the CAD editor model menus (input_state). The menu drive is
 * model/API-level first; GUI/TUI wiring follows. See editor.h for the
 * page constants (CAD_MENU_PAGE_*). */
#ifndef cad_menus_h
#define cad_menus_h
void cad_menus_update (void);
void cad_menus_set_level (int level);
bool cad_menus_key_press (int keyval); /* true if consumed */
#endif
