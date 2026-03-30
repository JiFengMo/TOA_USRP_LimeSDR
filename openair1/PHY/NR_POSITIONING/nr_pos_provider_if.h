#pragma once

#include "openair1/PHY/NR_POSITIONING/nr_pos_types.h"

typedef struct {
  const char *name;

  int (*init)(void *ctx);
  int (*acquire)(const nr_iq_block_t *blk, nr_sync_state_t *sync);
  int (*track)(const nr_iq_block_t *blk, nr_sync_state_t *sync);
  int (*extract_meas)(const nr_iq_block_t *blk,
                      const nr_sync_state_t *sync,
                      nr_toa_meas_t *meas);
  int (*dump_trace)(void *obj);
} nr_pos_provider_if_t;

extern const nr_pos_provider_if_t nr_ssb_provider;
extern const nr_pos_provider_if_t nr_prs_provider;
