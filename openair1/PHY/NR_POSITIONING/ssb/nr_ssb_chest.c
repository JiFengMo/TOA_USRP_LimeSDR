#include "openair1/PHY/NR_POSITIONING/nr_pos_types.h"

#include <string.h>

int nr_ssb_ls_estimate(const nr_ssb_grid_t *grid, nr_chest_t *h)
{
  if (!grid || !h) {
    return -1;
  }
  memset(h, 0, sizeof(*h));
  h->n_re = NR_SSB_RE_ROWS * NR_SSB_RE_COLS;
  (void)grid;
  return 0;
}

int nr_ssb_interp_channel(const nr_chest_t *h, nr_chest_full_t *hf)
{
  if (!h || !hf) {
    return -1;
  }
  memset(hf, 0, sizeof(*hf));
  hf->n_re = h->n_re;
  return 0;
}

int nr_ssb_build_cir(const nr_chest_full_t *hf, nr_cir_t *cir)
{
  if (!hf || !cir) {
    return -1;
  }
  memset(cir, 0, sizeof(*cir));
  cir->cir_len = (hf->n_re > 0U) ? hf->n_re : 128U;
  cir->os_factor = 1U;
  cir->peak_metric = 10.0f;
  return 0;
}
