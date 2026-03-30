#pragma once

#include "openair1/PHY/NR_POSITIONING/nr_pos_types.h"

typedef struct {
  const char *name;

  int (*init)(void *ctx);
  int (*acquire)(void *ctx, const nr_iq_block_t *blk, nr_sync_state_t *sync);
  int (*track)(void *ctx, const nr_iq_block_t *blk, nr_sync_state_t *sync);
  int (*extract_meas)(void *ctx, const nr_iq_block_t *blk,
                       const nr_sync_state_t *sync,
                       nr_toa_meas_t *meas);
  int (*dump_trace)(void *ctx);
} nr_pos_provider_if_t;

extern const nr_pos_provider_if_t nr_ssb_provider;
extern const nr_pos_provider_if_t nr_prs_provider;
