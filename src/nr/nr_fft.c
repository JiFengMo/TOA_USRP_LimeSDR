#include "nr/nr_fft.h"
#include "common/error.h"

#include <math.h>

int nr_fft_dft(const cf32_t *in, cf32_t *out, uint32_t n)
{
  if (!in || !out || n == 0)
    return TOA_ERR_INVALID_ARG;

  for (uint32_t k = 0; k < n; ++k) {
    double sr = 0.0;
    double si = 0.0;
    for (uint32_t t = 0; t < n; ++t) {
      double ph = -2.0 * M_PI * (double)k * (double)t / (double)n;
      double c = cos(ph);
      double s = sin(ph);
      sr += (double)in[t].r * c - (double)in[t].i * s;
      si += (double)in[t].r * s + (double)in[t].i * c;
    }
    out[k].r = (float)sr;
    out[k].i = (float)si;
  }
  return TOA_OK;
}
