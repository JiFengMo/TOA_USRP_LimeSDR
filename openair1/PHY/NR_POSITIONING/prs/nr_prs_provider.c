#include "openair1/PHY/NR_POSITIONING/nr_pos_provider_if.h"
#include "openair1/PHY/NR_POSITIONING/nr_pos_api.h"

static int prs_init(void *ctx)
{
  (void)ctx;
  return 0;
}

static int prs_acquire(void *ctx, const nr_iq_block_t *blk, nr_sync_state_t *sync)
{
  (void)ctx;
  if (!blk || !sync) {
    return -1;
  }
  /* Phase-0: skeleton acquire always "locks". Real PRS sync later. */
  sync->locked = 1;
  sync->pci = 0;
  sync->ssb_index = 0;
  sync->coarse_offset_samp = 0;
  sync->frac_offset_samp = 0.0f;
  sync->cfo_hz = 0.0f;
  sync->cum_tracking_shift_samp = 0;
  return 0;
}

static int prs_track(void *ctx, const nr_iq_block_t *blk, nr_sync_state_t *sync)
{
  (void)ctx;
  if (!blk || !sync) {
    return -1;
  }
  sync->locked = 1;
  return 0;
}

static int prs_extract_meas(void *ctx, const nr_iq_block_t *blk,
                            const nr_sync_state_t *sync, nr_toa_meas_t *meas)
{
  (void)ctx;
  if (!blk || !sync || !meas) {
    return -1;
  }
  (void)nr_toa_build_meas(sync, blk, 0, 0.0, meas);
  return 0;
}

static int prs_dump_trace(void *ctx)
{
  (void)ctx;
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
