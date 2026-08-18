/* MPE_TASK_V15R2_EVENT_LOG_IMPL_BEGIN */
#include "event_log.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

typedef struct {
    char message[EVENT_MSG_LENGTH];
    time_t timestamp;
    log_level level;
} engine_event;

static engine_event event_log_ring[EVENT_LOG_CAPACITY];
static int event_log_head = 0;
static int event_log_count = 0;

void event_log_init(void) {
    event_log_head = 0;
    event_log_count = 0;
    memset(event_log_ring, 0, sizeof(event_log_ring));
}

void event_log_push(log_level level, const char *format, ...) {
    va_list args;
    va_start(args, format);
    vsnprintf(event_log_ring[event_log_head].message, EVENT_MSG_LENGTH, format, args);
    va_end(args);
    event_log_ring[event_log_head].timestamp = time(NULL);
    event_log_ring[event_log_head].level = level;
    event_log_head = (event_log_head + 1) % EVENT_LOG_CAPACITY;
    if (event_log_count < EVENT_LOG_CAPACITY) {
        event_log_count++;
    }
}

int event_log_get_count(void) {
    return event_log_count;
}

const char *event_log_get_message(int index, log_level *level, time_t *timestamp) {
    if ((index < 0) || (index >= event_log_count)) { return NULL; }
    int actual_index;
    if (event_log_count < EVENT_LOG_CAPACITY) {
        actual_index = index;
    } else {
        actual_index = (event_log_head + index) % EVENT_LOG_CAPACITY;
    }
    if (level) { *level = event_log_ring[actual_index].level; }
    if (timestamp) { *timestamp = event_log_ring[actual_index].timestamp; }
    return event_log_ring[actual_index].message;
}

void event_log_clear(void) {
    event_log_head = 0;
    event_log_count = 0;
}
/* MPE_TASK_V15R2_EVENT_LOG_IMPL_END */
