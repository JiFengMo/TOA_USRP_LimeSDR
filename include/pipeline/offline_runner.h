#ifndef PIPELINE_OFFLINE_RUNNER_H
#define PIPELINE_OFFLINE_RUNNER_H

#include <stdint.h>

#include "common/cf32.h"
#include "common/error.h"
#include "io/iq_reader.h"

/*
 * Process each IQ chunk from file. Return non-TOA_OK to stop the loop.
 */
typedef toa_error_t (*offline_iq_block_fn)(void *userdata,
                                           const cf32_t *samples,
                                           uint32_t nsamps,
                                           const iq_block_info_t *blk);

/*
 * Replay a cf32 IQ file in chunks of chunk_samples.
 * time_fs_hz: if > 0, used for synthetic block times; if <= 0, use meta.sample_rate_hz when present.
 * meta_path: explicit JSON; NULL tries <stem>.meta.json.
 * blocklog_path: explicit blocklog CSV; NULL tries <stem>.blocklog.csv (optional);
 *               "" disables blocklog. When a row matches block index + sample_offset, hw_time_ns0 comes from log.
 */
toa_error_t offline_run_cf32_file(const char *iq_path,
                                  const char *meta_path,
                                  const char *blocklog_path,
                                  uint32_t chunk_samples,
                                  double time_fs_hz,
                                  void *userdata,
                                  offline_iq_block_fn cb);

#endif
