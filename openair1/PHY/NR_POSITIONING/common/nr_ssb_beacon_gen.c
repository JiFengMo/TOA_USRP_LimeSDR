#include "openair1/PHY/NR_POSITIONING/nr_pos_types.h"

#include <string.h>

int nr_ssb_gen_ref(uint16_t pci, uint8_t ssb_idx, nr_ssb_ref_t *ref)
{
  if (!ref) {
    return -1;
  }
  memset(ref, 0, sizeof(*ref));
  ref->pci = pci;
  ref->ssb_idx = ssb_idx;
  ref->n_fft = 2048;
  return 0;
}

int nr_ssb_build_grid(const nr_ssb_ref_t *ref, nr_ssb_grid_t *grid)
{
  if (!ref || !grid) {
    return -1;
  }
  memset(grid, 0, sizeof(*grid));
  grid->valid = 0;
  return 0;
}

int nr_ssb_ofdm_mod(const nr_ssb_grid_t *grid, c16_t *txbuf, uint32_t *nsamps)
{
  if (!grid || !txbuf || !nsamps) {
    return -1;
  }
  *nsamps = 0;
  (void)grid;
  return 0;
}
