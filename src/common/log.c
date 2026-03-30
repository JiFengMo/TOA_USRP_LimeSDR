#include "common/log.h"

#include <stdarg.h>
#include <stdio.h>

static log_level_t g_log_level = LOG_LEVEL_INFO;

void log_set_level(log_level_t level)
{
  g_log_level = level;
}

void log_msg(log_level_t level, const char *tag, const char *fmt, ...)
{
  va_list ap;

  if (level > g_log_level)
    return;

  fprintf(stderr, "[%s] ", tag ? tag : "LOG");

  va_start(ap, fmt);
  vfprintf(stderr, fmt, ap);
  va_end(ap);

  fputc('\n', stderr);
}
