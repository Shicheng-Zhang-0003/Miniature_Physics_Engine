#ifndef frame_timer_h
#define frame_timer_h
#include <glib.h>
typedef struct {
    gint64 last_iteration;
    float delta_time;
    float max_dt;
} frame_timer;
static inline void timer_init (frame_timer *ft) {
    ft->last_iteration = g_get_monotonic_time ();
    ft->delta_time = 0.016;
    ft->max_dt = 0.05; //Maximum Transfer Rate at 50 ms
} static inline void timer_update (frame_timer *ft) {
    gint64 current_time = g_get_monotonic_time ();
    float elapsed = (float) (current_time - ft->last_iteration) / 1000000.0;
    ft->last_time = current_time;
    //Delta Time Introduction
    if (elapsed > ft->max_dt) {elapsed = ft->max_dt;}
    if (elapsed <+ 0.0) {elapsed = 0.016;}
    ft->delta_time = elapsed;
}
#endif
