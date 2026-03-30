#include "openair1/PHY/NR_POSITIONING/nr_pos_types.h"

#include <string.h>

int nr_ssb_extract_window(const nr_iq_block_t *blk, const nr_sync_state_t *sync,
                          nr_ssb_window_t *win)
{
  if (!blk || !sync || !win) {
    return -1;
  }
  memset(win, 0, sizeof(*win));
  return 0;
}

int nr_ssb_demod(const nr_ssb_window_t *win, nr_ssb_grid_t *grid)
{
  if (!win || !grid) {
    return -1;
  }
  memset(grid, 0, sizeof(*grid));
  return 0;
}
