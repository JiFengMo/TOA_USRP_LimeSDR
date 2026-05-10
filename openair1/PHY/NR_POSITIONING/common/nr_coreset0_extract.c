#include "openair1/PHY/NR_POSITIONING/nr_pos_api.h"

#include <math.h>
#include <string.h>

int nr_type0_coreset0_extract_grid(const nr_cell_ctx_t *cell,
                                   const nr_iq_block_t *blk,
                                   nr_coreset0_grid_t *grid)
{
  nr_coreset0_monitor_window_t win;
  const uint32_t scs_khz =
      (cell && cell->type0_css.valid && cell->type0_css.scs_pdcch_khz > 0U)
          ? cell->type0_css.scs_pdcch_khz
          : nr_scs_khz_from_mib(cell ? &cell->mib : NULL);
  const uint32_t nfft = nr_v0_ssb_nfft(blk ? blk->fs_hz : 0.0);
  const int32_t num_subcarriers =
      cell && cell->type0_css.valid ? (cell->type0_css.num_rbs * 12) : 0;

  if (!grid) {
    return -1;
  }
  memset(grid, 0, sizeof(*grid));

  if (!cell || !blk || !blk->rx[0] || !cell->valid || !cell->mib.valid || !cell->type0_css.valid) {
    return -1;
  }

  if (num_subcarriers <= 0 || num_subcarriers > NR_CORESET0_MAX_SC) {
    return -1;
  }

  if (nr_type0_coreset0_monitor_window(cell, blk, &win) != 0 || !win.valid) {
    return -1;
  }

  memset(grid, 0, sizeof(*grid));
  grid->valid = 1U;
  grid->frame = win.frame;
  grid->slot = win.slot;
  grid->first_symbol_index = win.first_symbol_index;
  grid->num_symbols = win.num_symbols;
  grid->num_rbs = (uint32_t)win.num_rbs;
  grid->num_subcarriers = (uint32_t)num_subcarriers;
  grid->start_sc_rel_dc = win.start_sc_rel_dc;
  grid->end_sc_rel_dc = win.end_sc_rel_dc;

  for (uint32_t sym = 0U; sym < win.num_symbols && sym < NR_CORESET0_MAX_SYMBOLS; sym++) {
    uint64_t sym_start = win.start_samp;

    for (uint32_t prev = 0U; prev < sym; prev++) {
      sym_start += nr_ofdm_symbol_len_fs(blk->fs_hz,
                                         scs_khz,
                                         win.slot * 14U + win.first_symbol_index + prev);
    }

    {
      const uint32_t abs_sym = win.slot * 14U + win.first_symbol_index + sym;
      const uint32_t cp = nr_ofdm_symbol_cp_len_fs(blk->fs_hz, scs_khz, abs_sym);
      const uint64_t fft_start = sym_start + cp;

      if (fft_start + nfft > blk->nsamps) {
        return -1;
      }

      for (int32_t sc = 0; sc < num_subcarriers; sc++) {
        const int32_t k_rel = win.start_sc_rel_dc + sc;
        double sr = 0.0;
        double si = 0.0;
        const c16_t *x = blk->rx[0];

        for (uint32_t n = 0U; n < nfft; n++) {
          const double xr = (double)x[fft_start + n].r;
          const double xq = (double)x[fft_start + n].i;
          const double ph = -2.0 * M_PI * (double)k_rel * (double)n / (double)nfft;
          const double c = cos(ph);
          const double s = sin(ph);
          sr += xr * c - xq * s;
          si += xr * s + xq * c;
        }

        grid->re[sym][sc].r = (float)(sr / (double)nfft);
        grid->re[sym][sc].i = (float)(si / (double)nfft);
      }
    }
  }

  {
    double sum_pow = 0.0;
    double peak_pow = 0.0;
    uint32_t count = 0U;

    for (uint32_t sym = 0U; sym < grid->num_symbols && sym < NR_CORESET0_MAX_SYMBOLS; sym++) {
      for (uint32_t sc = 0U; sc < grid->num_subcarriers; sc++) {
        const double r = grid->re[sym][sc].r;
        const double i = grid->re[sym][sc].i;
        const double p = r * r + i * i;
        sum_pow += p;
        if (p > peak_pow) {
          peak_pow = p;
        }
        count++;
      }
    }

    if (count > 0U) {
      grid->avg_re_power = (float)(sum_pow / (double)count);
      grid->peak_re_power = (float)peak_pow;
    }
  }

  return 0;
}
