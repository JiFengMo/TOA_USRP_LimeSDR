#ifndef NR_FFT_H
#define NR_FFT_H

#include <stdint.h>
#include "common/cf32.h"

int nr_fft_dft(const cf32_t *in, cf32_t *out, uint32_t n);

#endif
