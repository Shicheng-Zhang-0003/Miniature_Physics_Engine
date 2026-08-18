/* MPE_TASK_V15R2_MICROVIM_HEADER_BEGIN */
#ifndef microvim_h
#define microvim_h

#include <gtk/gtk.h>
#include <stdbool.h>

typedef enum { MV_NORMAL, MV_INSERT, MV_COMMAND, MV_SEARCH } mv_mode;

void microvim_open (const char *filename);
void microvim_close (void);
bool microvim_is_active (void);
void microvim_handle_key (GdkEventKey *event);
void microvim_render (GtkTextBuffer *buffer);
void microvim_ensure_tags (GtkTextBuffer *buffer);
mv_mode microvim_get_mode (void);

#endif /* microvim_h */
/* MPE_TASK_V15R2_MICROVIM_HEADER_END */
