#include "openair1/PHY/NR_POSITIONING/nr_pos_types.h"
#include "radio/COMMON/common_lib.h"

#include <stdio.h>
#include <string.h>

int nr_toa_load_config(const char *path, nr_toa_app_cfg_t *cfg)
{
  if (!path || !cfg) {
    return -1;
  }
  memset(cfg, 0, sizeof(*cfg));
  (void)snprintf(cfg->sdr, sizeof(cfg->sdr), "lime");
  cfg->center_freq_hz = 3.5e9;
  cfg->sample_rate_hz = 30.72e6;
  cfg->mode = NR_TOA_MODE_SSB_TOA;
  cfg->meas_mode = NR_MEAS_MODE_MEAS_ONLY;
  (void)path;
  return 0;
}

int nr_toa_build_rf_cfg(const nr_toa_app_cfg_t *cfg, openair0_config_t *rf_cfg)
{
  if (!cfg || !rf_cfg) {
    return -1;
  }
  memset(rf_cfg, 0, sizeof(*rf_cfg));
  if (strcmp(cfg->sdr, "usrp") == 0) {
    rf_cfg->device_type = USRP_B200_DEV;
  } else {
    rf_cfg->device_type = LMSSDR_DEV;
  }
  rf_cfg->sample_rate = cfg->sample_rate_hz;
  rf_cfg->rx_freq_hz = cfg->center_freq_hz;
  rf_cfg->tx_freq_hz = cfg->center_freq_hz;
  rf_cfg->rx_gain_db = cfg->rx_gain_db;
  rf_cfg->tx_gain_db = cfg->tx_gain_db;
  return 0;
}
