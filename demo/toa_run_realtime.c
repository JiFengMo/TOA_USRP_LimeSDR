#include "common/log.h"
#include "toa/toa_context.h"

#include <stdio.h>

int main(void)
{
  toa_context_t ctx;
  int ret;

  log_set_level(LOG_LEVEL_INFO);
  ret = toa_context_init(&ctx);
  if (ret != 0) {
    printf("toa_context_init failed, ret=%d\n", ret);
    return -1;
  }

  log_msg(LOG_LEVEL_INFO, "TOA", "realtime scaffold ready: fs=%.2f", ctx.sample_rate_hz);
  return 0;
}
