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

int nr_ssb_ofdm_mod(const nr_ssb_grid_t *grid, nr_tx_burst_t *burst)
{
  if (!grid || !burst || !burst->tx[0]) {
    return -1;
  }
  (void)grid;
  burst->nsamps = 4096;
  burst->tx_ant = 1;
  for (uint32_t i = 0; i < burst->nsamps; i++) {
    burst->tx[0][i].r = (i % 16 == 0) ? 2047 : 0;
    burst->tx[0][i].i = 0;
  }
  return 0;
}
