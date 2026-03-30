#include "nr/nr_sequence.h"
#include "common/error.h"

#include <math.h>

int nr_generate_pss(uint32_t nid2, cf32_t *out, uint32_t len)
{
  static const int u_table[3] = {25, 29, 34};
  int u;

  if (!out || len < NR_PSS_LEN || nid2 > 2)
    return TOA_ERR_INVALID_ARG;

  u = u_table[nid2];
  for (uint32_t n = 0; n < NR_PSS_LEN; ++n) {
    double ph = -M_PI * (double)u * (double)n * ((double)n + 1.0) / 127.0;
    out[n].r = (float)cos(ph);
    out[n].i = (float)sin(ph);
  }

  return TOA_OK;
}

int nr_generate_sss(uint32_t nid1, uint32_t nid2, cf32_t *out, uint32_t len)
{
  uint8_t x0[127 + 7];
  uint8_t x1[127 + 7];
  uint32_t m0, m1;

  if (!out || len < NR_SSS_LEN || nid2 > 2 || nid1 > 335)
    return TOA_ERR_INVALID_ARG;

  /* x0(n+7)=x0(n+4)+x0(n), x1(n+7)=x1(n+1)+x1(n) over GF(2) */
  for (uint32_t i = 0; i < 127 + 7; ++i) {
    x0[i] = 0;
    x1[i] = 0;
  }
  x0[0] = 1;
  x1[0] = 1;
  for (uint32_t n = 0; n < 127; ++n) {
    x0[n + 7] = (uint8_t)((x0[n + 4] + x0[n]) & 1);
    x1[n + 7] = (uint8_t)((x1[n + 1] + x1[n]) & 1);
  }

  m0 = 15 * (nid1 / 112) + 5 * nid2;
  m1 = nid1 % 112;

  for (uint32_t n = 0; n < NR_SSS_LEN; ++n) {
    int b0 = x0[(n + m0) % 127];
    int b1 = x1[(n + m1) % 127];
    float v = (1 - 2 * b0) * (1 - 2 * b1);
    out[n].r = v;
    out[n].i = 0.0f;
  }

  return TOA_OK;
}
