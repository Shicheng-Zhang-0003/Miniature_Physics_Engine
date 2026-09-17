/* MPE_TASK_35_CONFIG_MENU_BEGIN */
#ifndef config_menu_h
#define config_menu_h

#include <gtk/gtk.h>
#include <stdbool.h>

/* Config menu state:
 * level 0  = closed
 * level 1  = category page 1 (categories 0-8)
 * level 2  = category page 2 (categories 9-12 + LOCKED + Save + Reset)
 * level 10+= param list for category (level - 10)
 */
void config_menu_key_press(int key_number);
void config_menu_update(GtkWidget *parent_window);
void config_menu_render(char *buffer, size_t buffer_size);
bool config_menu_is_open(void);
void config_menu_close(void);
void config_menu_level_force_open(void);

#endif
/* MPE_TASK_35_CONFIG_MENU_END */
