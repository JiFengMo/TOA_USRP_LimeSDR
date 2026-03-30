#include "openair1/PHY/NR_POSITIONING/nr_pos_types.h"

/*
 * Sample-domain arrival (blueprint section 6):
 * rx_arrival_samp = blk->ts_first + sync->coarse_offset_samp
 *                  + sync->cum_tracking_shift_samp + peak_idx + frac;
 * toa_samp = rx_arrival_samp - tx_depart_samp; use Fs for ns/m.
 */

int nr_toa_find_integer_peak(const nr_cir_t *cir, int *peak_idx)
{
  if (!cir || !peak_idx) {
    return -1;
  }
  *peak_idx = 0;
  return 0;
}

int nr_toa_refine_fractional(const nr_cir_t *cir, int peak_idx, double *frac)
{
  if (!cir || !frac) {
    return -1;
  }
  (void)peak_idx;
  *frac = 0.0;
  return 0;
}

int nr_toa_build_meas(const nr_sync_state_t *sync, const nr_iq_block_t *blk,
                      int peak_idx, double frac, nr_toa_meas_t *meas)
{
  if (!sync || !blk || !meas) {
    return -1;
  }
  (void)peak_idx;
  (void)frac;
  meas->rx_ts_int = blk->ts_first;
  meas->rx_ts_frac = 0.0;
  meas->fs_hz = 30.72e6;
  meas->valid = 0;
  (void)sync;
  return 0;
}
