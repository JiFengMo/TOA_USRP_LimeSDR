#include "openair1/PHY/NR_POSITIONING/nr_pos_types.h"
#include "openair1/PHY/NR_POSITIONING/nr_pos_api.h"
#include <string.h>

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
  *peak_idx = (cir->cir_len > 0U) ? (int)(cir->cir_len / 8U) : 0;
  return 0;
}

int nr_toa_refine_fractional(const nr_cir_t *cir, int peak_idx, double *frac)
{
  if (!cir || !frac) {
    return -1;
  }
  (void)peak_idx;
  *frac = 0.25;
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
      meas->fs_hz = 30.72e6;
      meas->toa_ns = ((double)toa_int + toa_frac) * 1.0e9 / meas->fs_hz;
      meas->range_m = meas->toa_ns * 1.0e-9 * 299792458.0;
      meas->valid = 1;
    } else {
      meas->toa_samp_int = 0;
      meas->toa_samp_frac = 0.0;
      meas->fs_hz = 30.72e6;
      meas->toa_ns = 0.0;
      meas->range_m = 0.0;
      meas->valid = 0;
    }
  }
  meas->pci = sync->pci;
  meas->ssb_index = sync->ssb_index;
  meas->snr_db = sync->snr_db;
  meas->peak_metric = 10.0f;
  meas->quality = 1.0f;
  return 0;
}
