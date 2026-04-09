#include "openair1/PHY/NR_POSITIONING/nr_pos_types.h"
#include "openair1/PHY/NR_POSITIONING/nr_pos_api.h"

#include <math.h>
#include <string.h>

int nr_ssb_extract_window(const nr_iq_block_t *blk, const nr_sync_state_t *sync,
                          nr_ssb_window_t *win)
{
  if (!blk || !sync || !win) {
    return -1;
  }
  memset(win, 0, sizeof(*win));
  int64_t start = (int64_t)sync->coarse_offset_samp + sync->cum_tracking_shift_samp;
  if (start < 0) {
    start = 0;
  }
  if ((uint64_t)start >= blk->nsamps) {
    start = 0;
  }
  win->start_samp = (uint32_t)start;
  const uint32_t want = nr_v0_ssb_burst_len_fs(blk->fs_hz);
  const uint32_t rem = blk->nsamps - win->start_samp;
  win->len_samp = (rem > want) ? want : rem;
  return 0;
}

int nr_ssb_demod(const nr_iq_block_t *blk, const nr_ssb_window_t *win,
                 float cfo_hz, nr_ssb_grid_t *grid)
{
  if (!blk || !blk->rx[0] || !win || !grid) {
    return -1;
  }
  memset(grid, 0, sizeof(*grid));
  const uint32_t nfft = nr_v0_ssb_nfft(blk->fs_hz);
  const double fs_hz = (blk->fs_hz > 0.0) ? blk->fs_hz : 30720000.0;
  const double w = -2.0 * M_PI * (double)cfo_hz / fs_hz;
  if (win->len_samp < nr_v0_ssb_burst_len_fs(blk->fs_hz)) {
    return -1;
  }

  uint32_t sym_off = 0U;
  for (int sym = 0; sym < NR_SSB_RE_ROWS; sym++) {
    const uint32_t cp = nr_v0_ssb_symbol_cp_len_fs(blk->fs_hz, (uint32_t)sym);
    const uint32_t sym_len = nfft + cp;
    const uint32_t sym_start = win->start_samp + sym_off;
    const uint32_t fft_start = sym_start + cp;
    if ((uint64_t)fft_start + nfft > blk->nsamps) {
      return -1;
    }
    for (int rel = 0; rel < NR_SSB_RE_COLS; rel++) {
      const int k = rel - 120;
      double sr = 0.0;
      double si = 0.0;
      for (uint32_t n = 0; n < nfft; n++) {
        const c16_t s = blk->rx[0][fft_start + n];
        double xr = (double)s.r;
        double xq = (double)s.i;
        if (cfo_hz != 0.0f) {
          const double ph_cfo = w * (double)(sym_off + cp + n);
          const double cc = cos(ph_cfo);
          const double ss = sin(ph_cfo);
          const double tr = xr * cc - xq * ss;
          const double tq = xr * ss + xq * cc;
          xr = tr;
          xq = tq;
        }
        const double ph = -2.0 * M_PI * (double)k * (double)n / (double)nfft;
        const double c = cos(ph);
        const double sgn = sin(ph);
        sr += xr * c - xq * sgn;
        si += xr * sgn + xq * c;
      }
      grid->re[sym][rel].r = (float)(sr / (double)nfft);
      grid->re[sym][rel].i = (float)(si / (double)nfft);
    }
    sym_off += sym_len;
  }
  grid->valid = 1;
  return 0;
}
