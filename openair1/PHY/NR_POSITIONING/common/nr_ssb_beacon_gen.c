#include "openair1/PHY/NR_POSITIONING/nr_pos_types.h"
#include "openair1/PHY/NR_POSITIONING/nr_pos_api.h"

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
  /* V2: 4-symbol SSB-like burst with PSS on symbol #2. */
  const uint32_t need = nr_v0_ssb_burst_len();
  if (burst->nsamps < need) {
    burst->nsamps = need;
  } else {
    burst->nsamps = need;
  }
  burst->tx_ant = 1;
  (void)nr_v0_ssb_build_burst_iq(1, burst->tx[0], burst->nsamps, 12000.0f);
  return 0;
}
