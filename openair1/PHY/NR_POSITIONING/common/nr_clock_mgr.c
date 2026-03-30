#include "openair1/PHY/NR_POSITIONING/nr_pos_types.h"
#include "radio/COMMON/common_lib.h"

#include <string.h>

int nr_toa_wait_clock_lock(openair0_device_t *dev, nr_clock_status_t *st)
{
  if (!dev || !st) {
    return -1;
  }
  memset(st, 0, sizeof(*st));
  st->locked = 1;
  st->ref_locked = 1;
  return 0;
}

int nr_toa_set_epoch_at_next_pps(openair0_device_t *dev, uint64_t epoch_ns)
{
  (void)dev;
  (void)epoch_ns;
  return 0;
}

int nr_toa_get_device_time(openair0_device_t *dev, openair0_timestamp_t *ts)
{
  if (!dev || !ts) {
    return -1;
  }
  *ts = 0;
  return 0;
}
