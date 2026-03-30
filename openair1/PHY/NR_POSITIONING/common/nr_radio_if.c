#include "openair1/PHY/NR_POSITIONING/nr_pos_types.h"
#include "radio/COMMON/common_lib.h"

#include <stdlib.h>
#include <string.h>

int nr_toa_radio_init(openair0_device_t **dev, openair0_config_t *cfg)
{
  if (!dev || !cfg) {
    return -1;
  }
  *dev = NULL;
  if (cfg->device_type == USRP_B200_DEV) {
    *dev = openair0_device_get_usrp(cfg);
  } else if (cfg->device_type == LMSSDR_DEV) {
    *dev = openair0_device_get_lime(cfg);
  } else {
    return -1;
  }
  return *dev ? 0 : -1;
}

int nr_toa_radio_start(openair0_device_t *dev)
{
  if (!dev || !dev->trx_start_func) {
    return -1;
  }
  return dev->trx_start_func(dev);
}

int nr_toa_radio_read(openair0_device_t *dev, nr_iq_block_t *blk)
{
  if (!dev || !blk || !dev->trx_read_func) {
    return -1;
  }
  void *buf[NR_TOA_MAX_RX_ANT] = {0};
  openair0_timestamp_t ts = 0;
  int ret = dev->trx_read_func(dev, &ts, buf, blk->nsamps, 0);
  blk->ts_first = ts;
  return ret;
}

int nr_toa_radio_write(openair0_device_t *dev, const nr_iq_block_t *blk)
{
  if (!dev || !blk || !dev->trx_write_func) {
    return -1;
  }
  void *buf[NR_TOA_MAX_RX_ANT];
  for (int i = 0; i < NR_TOA_MAX_RX_ANT; i++) {
    buf[i] = blk->rx[i];
  }
  openair0_timestamp_t ts = blk->ts_first;
  return dev->trx_write_func(dev, &ts, buf, blk->nsamps, 0, 0);
}
