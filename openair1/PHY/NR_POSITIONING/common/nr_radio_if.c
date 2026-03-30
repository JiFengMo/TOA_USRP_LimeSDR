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
  if (!*dev) {
    return -1;
  }

  /* Important Phase-0 step: configure RF immediately after device creation. */
  if ((*dev)->trx_config_func) {
    if ((*dev)->trx_config_func(*dev, cfg) != 0) {
      return -1;
    }
  }
  return 0;
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
  uint8_t n_ant = blk->rx_ant ? blk->rx_ant : 1;
  if (n_ant > NR_TOA_MAX_RX_ANT) {
    n_ant = NR_TOA_MAX_RX_ANT;
  }
  for (int i = 0; i < (int)n_ant; i++) {
    buf[i] = blk->rx[i];
  }

  openair0_timestamp_t ts = 0;
  /* Phase-0 skeleton: backends may ignore antenna, but we pass the buffers. */
  int ret = dev->trx_read_func(dev, &ts, buf, blk->nsamps, 0);
  if (ret < 0) {
    return -1;
  }
  blk->ts_first = ts;
  blk->abs_samp0 = (uint64_t)ts;
  return 0;
}

int nr_toa_radio_write(openair0_device_t *dev, const nr_iq_block_t *blk)
{
  if (!dev || !blk || !dev->trx_write_func) {
    return -1;
  }
  void *buf[NR_TOA_MAX_RX_ANT];
  uint8_t n_ant = blk->rx_ant ? blk->rx_ant : 1;
  if (n_ant > NR_TOA_MAX_RX_ANT) {
    n_ant = NR_TOA_MAX_RX_ANT;
  }
  for (int i = 0; i < (int)n_ant; i++) {
    buf[i] = blk->rx[i];
  }
  openair0_timestamp_t ts = blk->ts_first;
  return dev->trx_write_func(dev, &ts, buf, blk->nsamps, 0, 0);
}
