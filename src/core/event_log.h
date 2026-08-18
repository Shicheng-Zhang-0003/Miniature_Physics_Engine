/* MPE_TASK_V15R2_EVENT_LOG_HEADER_BEGIN */
#ifndef event_log_h
#define event_log_h

#include <time.h>

#define event_log_capacity 256
#define event_msg_length 256

typedef enum { log_info, log_warn, log_error } log_level;

void event_log_init(void);
void event_log_push(log_level level, const char *format, ...);
int event_log_get_count(void);
const char *event_log_get_message(int index, log_level *level, time_t *timestamp);
void event_log_clear(void);

#endif /* event_log_h */
/* MPE_TASK_V15R2_EVENT_LOG_HEADER_END */
