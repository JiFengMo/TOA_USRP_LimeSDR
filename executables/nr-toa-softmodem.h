#pragma once

#include "openair1/PHY/NR_POSITIONING/nr_pos_types.h"
#include "openair1/PHY/NR_POSITIONING/nr_pos_provider_if.h"
#include "radio/COMMON/common_lib.h"

typedef struct PHY_VARS_NR_TOA_UE {
  nr_toa_state_t state;
  nr_sync_state_t sync;
  nr_iq_ring_t iq_ring;
  nr_epoch_mgr_t epoch_mgr;
  openair0_device_t *dev;
  openair0_config_t rf_cfg;
  nr_toa_app_cfg_t app_cfg;
  const nr_pos_provider_if_t *provider;
} PHY_VARS_NR_TOA_UE;
