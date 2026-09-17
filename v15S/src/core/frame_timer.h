#ifndef frame_timer_h
#define frame_timer_h
/* GTK4-PREP: zero GUI headers in core. This header used <glib.h> for
 * g_get_monotonic_time/gint64 only; POSIX clock_gettime gives identical
 * microsecond-monotonic semantics with no system GUI dependency. */
#include <stdint.h>
#include <time.h>
typedef struct {
    int64_t last_iteration_time;
    float delta_time;
    float maximum_delta_time;
} frame_timer;
static inline int64_t frame_timer_now_us(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t) ts.tv_sec * 1000000LL + (int64_t) (ts.tv_nsec / 1000);
}
static inline void frame_timer_init(frame_timer *timer_object) {
    timer_object->last_iteration_time = frame_timer_now_us();
    timer_object->delta_time = 0.016f;
    timer_object->maximum_delta_time = 0.05f; //Maximum Transfer Rate at 50 ms
}
static inline void frame_timer_update(frame_timer *timer_object) {
    int64_t current_monotonic_time = frame_timer_now_us();
    float elapsed_seconds = (float) (current_monotonic_time - timer_object->last_iteration_time) / 1000000.0f;
    timer_object->last_iteration_time = current_monotonic_time;
    //Delta Time Introduction
    if (elapsed_seconds > timer_object->maximum_delta_time) {
        elapsed_seconds = timer_object->maximum_delta_time;
    }
    if (elapsed_seconds <= 0.0f) {
        /* FIX-AUDIT: clock repeat/pause must freeze, not inject 16ms motion. */
        elapsed_seconds = 0.0f;
    }
    timer_object->delta_time = elapsed_seconds;
}
#endif
