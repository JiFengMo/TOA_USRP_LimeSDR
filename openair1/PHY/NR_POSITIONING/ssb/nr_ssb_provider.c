#include "openair1/PHY/NR_POSITIONING/nr_pos_provider_if.h"
#include "openair1/PHY/NR_POSITIONING/nr_pos_api.h"

#include <stdint.h>
#include <stdio.h>

static int ssb_init(void *ctx)
{
  (void)ctx;
  return 0;
}

static int ssb_acquire(void *ctx, const nr_iq_block_t *blk, nr_sync_state_t *sync)
{
  (void)ctx;
  static uint32_t dbg_cnt = 0;
  if (!blk || !sync) {
    return -1;
  }

  nr_pss_hit_t hits[4];
  if (nr_ssb_pss_search(blk, hits, 4) != 0) {
    sync->locked = 0;
    if ((dbg_cnt++ % 100U) == 0U) {
      printf("pss_search: not-detected\n");
    }
    return -1;
  }
  if ((dbg_cnt++ % 100U) == 0U) {
    printf("pss_hits: [0]nid2=%u off=%d m=%.3f [1]nid2=%u off=%d m=%.3f [2]nid2=%u off=%d m=%.3f\n",
           (unsigned)hits[0].nid2, hits[0].peak_samp, hits[0].metric,
           (unsigned)hits[1].nid2, hits[1].peak_samp, hits[1].metric,
           (unsigned)hits[2].nid2, hits[2].peak_samp, hits[2].metric);
  }
  if (nr_ssb_refine_sync(blk, &hits[0], sync) != 0) {
    sync->locked = 0;
    return -1;
  }
  /* PBCH decode is optional in V0 sniff mode. */
  (void)nr_ssb_pbch_decode(blk, sync);
  return 0;
}

static int ssb_track(void *ctx, const nr_iq_block_t *blk, nr_sync_state_t *sync)
{
  (void)ctx;
  if (!blk || !sync) {
    return -1;
  }

  if (nr_ssb_track_cfo(blk, sync) != 0) {
    sync->locked = 0;
    return -1;
  }
  int sample_shift = 0;
  if (nr_ssb_track_timing(blk, sync, &sample_shift) != 0) {
    sync->locked = 0;
    return -1;
  }
  if (nr_ssb_check_lost_lock(sync) != 0) {
    sync->locked = 0;
  }
  return 0;
}

static int ssb_extract_meas(void *ctx, const nr_iq_block_t *blk,
                            const nr_sync_state_t *sync, nr_toa_meas_t *meas)
{
  (void)ctx;
  if (!blk || !sync || !meas) {
    return -1;
  }

  nr_ssb_window_t win;
  nr_ssb_grid_t grid;
  nr_chest_t h;
  nr_chest_full_t hf;
  nr_cir_t cir;
  int peak_idx = 0;
  double frac = 0.0;

  if (nr_ssb_extract_window(blk, sync, &win) != 0) {
    return -1;
  }
  if (nr_ssb_demod(&win, &grid) != 0) {
    return -1;
  }
  if (nr_ssb_ls_estimate(&grid, &h) != 0) {
    return -1;
  }
  if (nr_ssb_interp_channel(&h, &hf) != 0) {
    return -1;
  }
  if (nr_ssb_build_cir(&hf, &cir) != 0) {
    return -1;
  }
  if (nr_toa_find_integer_peak(&cir, &peak_idx) != 0) {
    return -1;
  }
  if (nr_toa_refine_fractional(&cir, peak_idx, &frac) != 0) {
    return -1;
  }
  if (nr_toa_build_meas(sync, blk, peak_idx, frac, meas) != 0) {
    return -1;
  }
  return 0;
}

static int ssb_dump_trace(void *ctx)
{
  (void)ctx;
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
