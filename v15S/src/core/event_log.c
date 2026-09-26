/* MPE_TASK_V15R2_EVENT_LOG_IMPL_BEGIN */
#include "event_log.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <pthread.h>

typedef struct {
    char message[event_msg_length];
    time_t timestamp;
    log_level level;
} engine_event;

static engine_event event_log_ring[event_log_capacity];
static int event_log_head = 0;
static int event_log_count = 0;
/* FIX-AUDIT-DESPOT: the ring (head/count/messages) was lock-free mutable
 * global state: a physics worker pushing while the terminal/overlay read
 * raced (torn vsnprintf target, head/count skew returning a half-written
 * slot). One process-wide mutex serializes push vs get/clear/init; time()
 * and vsnprintf run under it (log rate is low, never hot-path). */
static pthread_mutex_t event_log_lock = PTHREAD_MUTEX_INITIALIZER;

void event_log_init(void) {
    pthread_mutex_lock(&event_log_lock);
    event_log_head = 0;
    event_log_count = 0;
    memset(event_log_ring, 0, sizeof(event_log_ring));
    pthread_mutex_unlock(&event_log_lock);
}

void event_log_push(log_level level, const char *format, ...) {
    if (!format) {
        return;
    }
    pthread_mutex_lock(&event_log_lock);
    va_list args;
    va_start(args, format);
    vsnprintf(event_log_ring[event_log_head].message, event_msg_length, format, args);
    va_end(args);
    event_log_ring[event_log_head].message[event_msg_length - 1] = '\0';
    event_log_ring[event_log_head].timestamp = time(NULL);
    event_log_ring[event_log_head].level = level;
    event_log_head = (event_log_head + 1) % event_log_capacity;
    if (event_log_count < event_log_capacity) {
        event_log_count++;
    }
    pthread_mutex_unlock(&event_log_lock);
}

int event_log_get_count(void) {
    pthread_mutex_lock(&event_log_lock);
    int n = event_log_count;
    pthread_mutex_unlock(&event_log_lock);
    return n;
}

const char *event_log_get_message(int index, log_level *level, time_t *timestamp) {
    /* NOTE: the returned pointer aliases the ring slot: the caller must
     * copy it before the next push/init/clear (same contract as before;
     * the lock cannot be held across the return). Index bounds are checked
     * under lock so a concurrent push cannot skew the computation. */
    pthread_mutex_lock(&event_log_lock);
    if ((index < 0) || (index >= event_log_count)) {
        pthread_mutex_unlock(&event_log_lock);
        return NULL;
    }
    int actual_index;
    if (event_log_count < event_log_capacity) {
        actual_index = index;
    } else {
        actual_index = (event_log_head + index) % event_log_capacity;
    }
    if (level) {
        *level = event_log_ring[actual_index].level;
    }
    if (timestamp) {
        *timestamp = event_log_ring[actual_index].timestamp;
    }
    const char *msg = event_log_ring[actual_index].message;
    pthread_mutex_unlock(&event_log_lock);
    return msg;
}

void event_log_clear(void) {
    pthread_mutex_lock(&event_log_lock);
    event_log_head = 0;
    event_log_count = 0;
    pthread_mutex_unlock(&event_log_lock);
}
/* MPE_TASK_V15R2_EVENT_LOG_IMPL_END */
