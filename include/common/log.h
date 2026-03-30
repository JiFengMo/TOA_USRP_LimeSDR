#ifndef COMMON_LOG_H
#define COMMON_LOG_H

typedef enum {
  LOG_LEVEL_ERROR = 0,
  LOG_LEVEL_WARN = 1,
  LOG_LEVEL_INFO = 2,
  LOG_LEVEL_DEBUG = 3
} log_level_t;

void log_set_level(log_level_t level);
void log_msg(log_level_t level, const char *tag, const char *fmt, ...);

#endif
