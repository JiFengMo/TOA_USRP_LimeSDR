#ifndef SSB_SYNC_H
#define SSB_SYNC_H

#include <stdint.h>
#include "common/cf32.h"
#include "nr/nr_types.h"

int nr_ssb_sync_init(nr_ssb_sync_ctx_t *ctx, const nr_ssb_cfg_t *cfg);
void nr_ssb_sync_reset(nr_ssb_sync_ctx_t *ctx);
int nr_ssb_sync_scan_block(nr_ssb_sync_ctx_t *ctx,
                           const cf32_t *samples,
                           uint32_t sample_count,
                           int64_t block_time_ns,
                           nr_ssb_result_t *result);

#endif
