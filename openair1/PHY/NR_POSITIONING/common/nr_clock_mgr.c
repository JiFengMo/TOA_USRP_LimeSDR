#include "openair1/PHY/NR_POSITIONING/nr_pos_types.h"
#include "radio/COMMON/common_lib.h"

#include <string.h>

static openair0_timestamp_t g_fake_time;

int nr_toa_wait_clock_lock(openair0_device_t *dev, nr_clock_status_t *st)
{
  if (!dev || !st) {
    return -1;
  }
  memset(st, 0, sizeof(*st));
  if (dev->trx_get_clock_status_func) {
    uint8_t ref_locked = 0U;
    uint8_t pps_locked = 0U;
    uint8_t gps_locked = 0U;
    if (dev->trx_get_clock_status_func(dev, &ref_locked, &pps_locked, &gps_locked) != 0) {
      return -1;
    }
    st->ref_locked = ref_locked;
    st->pps_locked = pps_locked;
    st->gps_locked = gps_locked;
    st->locked = (ref_locked && pps_locked) ? 1U : 0U;
    return 0;
  }

  st->locked = 1;
  st->ref_locked = 1;
  st->pps_locked = 1;
  return 0;
}

int nr_toa_set_epoch_at_next_pps(openair0_device_t *dev, uint64_t epoch_ns)
{
  if (!dev) {
    return -1;
  }
  if (dev->trx_set_time_next_pps_func) {
    if (dev->trx_set_time_next_pps_func(dev, epoch_ns) != 0) {
      return -1;
    }
  }
  g_fake_time = (openair0_timestamp_t)epoch_ns;
  return 0;
}

int nr_toa_get_device_time(openair0_device_t *dev, openair0_timestamp_t *ts)
{
  if (!dev || !ts) {
    return -1;
  }
  if (dev->trx_get_time_func) {
    return dev->trx_get_time_func(dev, ts);
  }
  g_fake_time += 30720; /* default 1 ms at 30.72 Msps */
  *ts = g_fake_time;
  return 0;
}
