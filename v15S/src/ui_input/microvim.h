/* MPE_TASK_V15R2_MICROVIM_HEADER_BEGIN */
#ifndef microvim_h
#define microvim_h

#include <gtk/gtk.h>
#include <stdbool.h>
#include "term_posix.h" /* POSIX command-logic helpers (no libglib) */

typedef enum { mv_normal, mv_insert, mv_command, mv_search } mv_mode;

void microvim_open(const char *filename);
void microvim_close(void);
bool microvim_is_active(void);
#ifdef MPE_GTK4
void microvim_handle_key(guint keyval, guint keycode, GdkModifierType state);
#else
void microvim_handle_key(GdkEventKey *event);
#endif
void microvim_render(GtkTextBuffer *buffer);
void microvim_ensure_tags(GtkTextBuffer *buffer);
mv_mode microvim_get_mode(void);

#endif /* microvim_h */
/* MPE_TASK_V15R2_MICROVIM_HEADER_END */
