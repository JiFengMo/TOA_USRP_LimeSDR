#include "openair1/PHY/NR_POSITIONING/nr_pos_types.h"

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
  win->len_samp = (blk->nsamps - win->start_samp > 512U) ? 512U : (blk->nsamps - win->start_samp);
  return 0;
}

int nr_ssb_demod(const nr_ssb_window_t *win, nr_ssb_grid_t *grid)
{
  if (!win || !grid) {
    return -1;
  }
  memset(grid, 0, sizeof(*grid));
  for (int r = 0; r < NR_SSB_RE_ROWS; r++) {
    for (int c = 0; c < NR_SSB_RE_COLS; c++) {
      grid->re[r][c].r = (float)((win->start_samp + (uint32_t)c + (uint32_t)r) % 13U) * 0.01f;
      grid->re[r][c].i = 0.0f;
    }
  }
  grid->valid = 1;
  return 0;
}
