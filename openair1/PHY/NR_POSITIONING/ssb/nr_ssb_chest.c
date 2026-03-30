#include "openair1/PHY/NR_POSITIONING/nr_pos_types.h"

#include <string.h>

int nr_ssb_ls_estimate(const nr_ssb_grid_t *grid, nr_chest_t *h)
{
  if (!grid || !h) {
    return -1;
  }
  memset(h, 0, sizeof(*h));
  return 0;
}

int nr_ssb_interp_channel(const nr_chest_t *h, nr_chest_full_t *hf)
{
  if (!h || !hf) {
    return -1;
  }
  memset(hf, 0, sizeof(*hf));
  return 0;
}

int nr_ssb_build_cir(const nr_chest_full_t *hf, nr_cir_t *cir)
{
  if (!hf || !cir) {
    return -1;
  }
  memset(cir, 0, sizeof(*cir));
  return 0;
}
