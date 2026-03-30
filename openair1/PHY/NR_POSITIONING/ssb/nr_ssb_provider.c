#include "openair1/PHY/NR_POSITIONING/nr_pos_provider_if.h"

static int ssb_init(void *ctx)
{
  (void)ctx;
  return 0;
}

static int ssb_acquire(const nr_iq_block_t *blk, nr_sync_state_t *sync)
{
  if (!blk || !sync) {
    return -1;
  }
  return 0;
}

static int ssb_track(const nr_iq_block_t *blk, nr_sync_state_t *sync)
{
  if (!blk || !sync) {
    return -1;
  }
  return 0;
}

static int ssb_extract_meas(const nr_iq_block_t *blk,
                            const nr_sync_state_t *sync, nr_toa_meas_t *meas)
{
  if (!blk || !sync || !meas) {
    return -1;
  }
  return 0;
}

static int ssb_dump_trace(void *obj)
{
  (void)obj;
  return 0;
}

const nr_pos_provider_if_t nr_ssb_provider = {
    .name = "ssb_provider",
    .init = ssb_init,
    .acquire = ssb_acquire,
    .track = ssb_track,
    .extract_meas = ssb_extract_meas,
    .dump_trace = ssb_dump_trace,
};
