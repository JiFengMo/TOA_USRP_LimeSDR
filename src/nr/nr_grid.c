#include "nr/nr_grid.h"
#include "nr/nr_fft.h"
#include "common/error.h"

int nr_extract_symbol_fft(const cf32_t *samples,
                          uint32_t sample_count,
                          uint32_t start_pos,
                          uint32_t cp_len,
                          uint32_t fft_size,
                          cf32_t *out_fft)
{
  uint32_t sym0;

  if (!samples || !out_fft || fft_size == 0)
    return TOA_ERR_INVALID_ARG;

  sym0 = start_pos + cp_len;
  if (sym0 + fft_size > sample_count)
    return TOA_ERR_INVALID_ARG;

  return nr_fft_dft(samples + sym0, out_fft, fft_size);
}
