#include "nr/pbch_dmrs.h"
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

static uint32_t xorshift32(uint32_t *state)
{
  uint32_t x = *state;
  x ^= x << 13;
  x ^= x >> 17;
  x ^= x << 5;
  *state = x;
  return x;
}

int nr_generate_pbch_dmrs(uint32_t pci, uint32_t i_ssb, cf32_t *out, uint32_t len)
{
  uint32_t st;
  const float s = 0.7071067811865475f;

  if (!out || len == 0)
    return TOA_ERR_INVALID_ARG;

  st = (pci + 1u) * 1315423911u ^ (i_ssb + 7u) * 2654435761u;
  if (st == 0)
    st = 1u;

  for (uint32_t i = 0; i < len; ++i) {
    uint32_t r = xorshift32(&st);
    int b0 = (r >> 0) & 1;
    int b1 = (r >> 1) & 1;
    out[i].r = b0 ? s : -s;
    out[i].i = b1 ? s : -s;
  }
  return TOA_OK;
}

int nr_pbch_dmrs_correlate(const cf32_t *fft_syms,
                           uint32_t n_sym,
                           uint32_t fft_size,
                           uint32_t pci,
                           uint32_t i_ssb,
                           nr_pbch_dmrs_result_t *res)
{
  cf32_t dmrs[NR_PBCH_DMRS_MAX_RE];
  int offs[64];
  uint32_t noffs = 0;
  cf32_t acc = {0}, acc2 = {0};
  double ex = 0.0, er = 0.0, ex2 = 0.0, er2 = 0.0;
  float best = 0.0f, second = 0.0f;
  uint32_t center, ref_idx = 0;
  uint32_t v = pci & 0x3;
  static const uint32_t dmrs_syms[] = {1, 3};

  if (!fft_syms || !res || n_sym == 0 || fft_size < 256)
    return TOA_ERR_INVALID_ARG;

  memset(res, 0, sizeof(*res));
  nr_generate_pbch_dmrs(pci, i_ssb, dmrs, NR_PBCH_DMRS_MAX_RE);
  dmrs_offset_list(offs, 64, &noffs, v);
  center = fft_size / 2;

  for (uint32_t si = 0; si < 2; ++si) {
    uint32_t sym = dmrs_syms[si];
    if (sym >= n_sym)
      continue;
    for (uint32_t oi = 0; oi < noffs; ++oi) {
      int k = (int)center + offs[oi];
      cf32_t a, b, a2;
      if (k < 0 || (uint32_t)k >= fft_size || ref_idx >= NR_PBCH_DMRS_MAX_RE)
        continue;
      a = dmrs[ref_idx];
      b = fft_syms[sym * fft_size + (uint32_t)k];
      a2 = dmrs[(ref_idx + 1) % NR_PBCH_DMRS_MAX_RE];
      /* best: conj(a)*b, second proxy: conj(a shifted)*b */
      acc.r += a.r * b.r + a.i * b.i;
      acc.i += a.r * b.i - a.i * b.r;
      ex += (double)b.r * (double)b.r + (double)b.i * (double)b.i;
      er += (double)a.r * (double)a.r + (double)a.i * (double)a.i;

      acc2.r += a2.r * b.r + a2.i * b.i;
      acc2.i += a2.r * b.i - a2.i * b.r;
      ex2 += (double)b.r * (double)b.r + (double)b.i * (double)b.i;
      er2 += (double)a2.r * (double)a2.r + (double)a2.i * (double)a2.i;
      ref_idx++;
    }
  }

  if (ex > 0.0 && er > 0.0)
    best = (float)((acc.r * acc.r + acc.i * acc.i) / (ex * er));
  if (ex2 > 0.0 && er2 > 0.0)
    second = (float)((acc2.r * acc2.r + acc2.i * acc2.i) / (ex2 * er2));

  res->valid = (best > 0.0f);
  res->metric = best;
  res->peak_ratio = (second > 0.0f) ? (best / second) : best;
  res->re_count = ref_idx;
  return TOA_OK;
}
