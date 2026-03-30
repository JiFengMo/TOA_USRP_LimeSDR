#include "nr/pbch_re.h"
#include "common/error.h"

#include <string.h>

static int dmrs_offset_list(int *offs, uint32_t cap, uint32_t *n_out, uint32_t v)
{
  uint32_t n = 0;
  if (!offs || !n_out || cap == 0)
    return TOA_ERR_INVALID_ARG;

  for (int k = -120; k < 120; ++k) {
    int m = ((k % 4) + 4) % 4;
    if ((uint32_t)m == (v & 0x3)) {
      if (n >= cap)
        break;
      offs[n++] = k;
    }
  }
  *n_out = n;
  return TOA_OK;
}

int nr_pbch_extract_llr(const cf32_t *fft_syms,
                        uint32_t n_sym,
                        uint32_t fft_size,
                        uint32_t pci,
                        uint32_t i_ssb,
                        float *llr_out,
                        uint32_t llr_cap,
                        uint32_t *llr_num)
{
  int offs[64];
  uint32_t noffs = 0;
  uint32_t center;
  uint32_t n = 0;
  uint32_t v;

  if (!fft_syms || !llr_out || !llr_num || n_sym == 0 || fft_size < 256)
    return TOA_ERR_INVALID_ARG;

  *llr_num = 0;
  center = fft_size / 2u;
  v = (pci + i_ssb) & 0x3u;
  dmrs_offset_list(offs, 64, &noffs, v);

  /*
   * Stage-3 PBCH RE proxy:
   * - Use symbols 1/2/3 from the 4-symbol SSB block.
   * - Exclude DMRS REs on symbols 1 and 3.
   */
  for (uint32_t si = 0; si < n_sym; ++si) {
    if (!(si == 1 || si == 2 || si == 3))
      continue;
    for (int k = -120; k < 120; ++k) {
      int idx = (int)center + k;
      int is_dmrs = 0;
      if (idx < 0 || (uint32_t)idx >= fft_size)
        continue;
      if (si == 1 || si == 3) {
        for (uint32_t oi = 0; oi < noffs; ++oi) {
          if (k == offs[oi]) {
            is_dmrs = 1;
            break;
          }
        }
      }
      if (is_dmrs)
        continue;
      if (n + 1 >= llr_cap)
        break;
      llr_out[n++] = fft_syms[si * fft_size + (uint32_t)idx].r;
      llr_out[n++] = fft_syms[si * fft_size + (uint32_t)idx].i;
    }
  }

  *llr_num = n;
  return TOA_OK;
}
