#ifndef frame_timer_h
#define frame_timer_h
#include <glib.h>
typedef struct {
    gint64 last_iteration_time;
    float delta_time;
    float maximum_delta_time;
} frame_timer;
static inline void frame_timer_init (frame_timer *timer_object) {
    timer_object -> last_iteration_time = g_get_monotonic_time ();
    timer_object -> delta_time = 0.016f;
    timer_object -> maximum_delta_time = 0.05f; //Maximum Transfer Rate at 50 ms
} static inline void frame_timer_update (frame_timer *timer_object) {
    gint64 current_monotonic_time = g_get_monotonic_time ();
    float elapsed_seconds = (float) (current_monotonic_time - timer_object -> last_iteration_time) / 1000000.0f;
    timer_object -> last_iteration_time = current_monotonic_time;
    //Delta Time Introduction
    if (elapsed_seconds > timer_object -> maximum_delta_time) {elapsed_seconds = timer_object -> maximum_delta_time;}
    if (elapsed_seconds <= 0.0f) {elapsed_seconds = 0.016f;}
    timer_object -> delta_time = elapsed_seconds;
}
#endif
