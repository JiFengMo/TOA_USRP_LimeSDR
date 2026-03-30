#include "openair1/PHY/NR_POSITIONING/nr_pos_provider_if.h"

static int prs_init(void *ctx)
{
  (void)ctx;
  return 0;
}

static int prs_acquire(const nr_iq_block_t *blk, nr_sync_state_t *sync)
{
  if (!blk || !sync) {
    return -1;
  }
  return 0;
}

static int prs_track(const nr_iq_block_t *blk, nr_sync_state_t *sync)
{
  if (!blk || !sync) {
    return -1;
  }
  return 0;
}

static int prs_extract_meas(const nr_iq_block_t *blk,
                            const nr_sync_state_t *sync, nr_toa_meas_t *meas)
{
  if (!blk || !sync || !meas) {
    return -1;
  }
  return 0;
}

static int prs_dump_trace(void *obj)
{
  (void)obj;
  return 0;
}

const nr_pos_provider_if_t nr_prs_provider = {
    .name = "prs_provider",
    .init = prs_init,
    .acquire = prs_acquire,
    .track = prs_track,
    .extract_meas = prs_extract_meas,
    .dump_trace = prs_dump_trace,
};
