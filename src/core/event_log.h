/* MPE_TASK_V15R2_EVENT_LOG_HEADER_BEGIN */
#ifndef event_log_h
#define event_log_h

#include <time.h>

#define EVENT_LOG_CAPACITY 256
#define EVENT_MSG_LENGTH 256

typedef enum { LOG_INFO, LOG_WARN, LOG_ERROR } log_level;

void event_log_init(void);
void event_log_push(log_level level, const char *format, ...);
int event_log_get_count(void);
const char *event_log_get_message(int index, log_level *level, time_t *timestamp);
void event_log_clear(void);

#endif /* event_log_h */
/* MPE_TASK_V15R2_EVENT_LOG_HEADER_END */
