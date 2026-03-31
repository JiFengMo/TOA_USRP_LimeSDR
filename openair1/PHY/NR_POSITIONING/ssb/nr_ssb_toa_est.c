#include "openair1/PHY/NR_POSITIONING/nr_pos_types.h"
#include "openair1/PHY/NR_POSITIONING/nr_pos_api.h"
#include <math.h>
#include <string.h>

/*
 * Sample-domain arrival (blueprint section 6):
 * rx_arrival_samp = blk->ts_first + sync->coarse_offset_samp
 *                  + sync->cum_tracking_shift_samp + peak_idx + frac;
 * toa_samp = rx_arrival_samp - tx_depart_samp; use Fs for ns/m.
 */

int nr_toa_find_integer_peak(const nr_cir_t *cir, int *peak_idx)
{
  float best_mag = -1.0f;
  if (!cir || !peak_idx) {
    return -1;
  }
  if (!cir->cir || cir->cir_len == 0U) {
    *peak_idx = 0;
    return 0;
  }
  *peak_idx = 0;
  for (uint32_t n = 0U; n < cir->cir_len; n++) {
    const float mag = hypotf(cir->cir[n].r, cir->cir[n].i);
    if (mag > best_mag) {
      best_mag = mag;
      *peak_idx = (int)n;
    }
  }
  return 0;
}

int nr_toa_refine_fractional(const nr_cir_t *cir, int peak_idx, double *frac)
{
  float yl = 0.0f;
  float yc = 0.0f;
  float yr = 0.0f;
  float den = 0.0f;
  if (!cir || !frac) {
    return -1;
  }
  if (!cir->cir || cir->cir_len < 3U || peak_idx <= 0 || (uint32_t)(peak_idx + 1) >= cir->cir_len) {
    *frac = 0.0;
    return 0;
  }
  yl = hypotf(cir->cir[peak_idx - 1].r, cir->cir[peak_idx - 1].i);
  yc = hypotf(cir->cir[peak_idx].r, cir->cir[peak_idx].i);
  yr = hypotf(cir->cir[peak_idx + 1].r, cir->cir[peak_idx + 1].i);
  den = yl - 2.0f * yc + yr;
  if (fabsf(den) < 1.0e-6f) {
    *frac = 0.0;
    return 0;
  }
  *frac = 0.5 * (double)(yl - yr) / (double)den;
  if (*frac > 0.5) *frac = 0.5;
  if (*frac < -0.5) *frac = -0.5;
  return 0;
}

int nr_toa_build_meas(const nr_sync_state_t *sync, const nr_iq_block_t *blk,
                      int peak_idx, double frac, nr_toa_meas_t *meas)
{
  if (!sync || !blk || !meas) {
    return -1;
  }
  memset(meas, 0, sizeof(*meas));
  /* Sample-domain arrival (rx) in skeleton form. tx depart is not known yet. */
  {
    int64_t rx_arrival_int = (int64_t)blk->ts_first + sync->coarse_offset_samp +
                              sync->cum_tracking_shift_samp + peak_idx;
    double rx_arrival_frac = frac;

    meas->rx_ts_int = (openair0_timestamp_t)rx_arrival_int;
    meas->rx_ts_frac = rx_arrival_frac;

    openair0_timestamp_t tx_ts = nr_toa_get_last_tx_timestamp();
    meas->tx_ts_int = tx_ts;
    meas->tx_ts_frac = 0.0;
    if (tx_ts > 0 && rx_arrival_int >= (int64_t)tx_ts) {
      int64_t toa_int = rx_arrival_int - (int64_t)tx_ts;
      double toa_frac = rx_arrival_frac;
      meas->toa_samp_int = toa_int;
      meas->toa_samp_frac = toa_frac;
      meas->fs_hz = (blk->fs_hz > 0.0) ? blk->fs_hz : 30.72e6;
      meas->toa_ns = ((double)toa_int + toa_frac) * 1.0e9 / meas->fs_hz;
      meas->range_m = meas->toa_ns * 1.0e-9 * 299792458.0;
      meas->valid = 1;
    } else {
      meas->toa_samp_int = 0;
      meas->toa_samp_frac = 0.0;
      meas->fs_hz = (blk->fs_hz > 0.0) ? blk->fs_hz : 30.72e6;
      meas->toa_ns = 0.0;
      meas->range_m = 0.0;
      meas->valid = 0;
    }
  }
  meas->pci = sync->pci;
  meas->ssb_index = sync->ssb_index;
  meas->snr_db = sync->snr_db;
  meas->peak_metric = 0.0f;
  meas->quality = 0.0f;
  return 0;
}
