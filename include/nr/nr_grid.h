#ifndef NR_GRID_H
#define NR_GRID_H

#include <stdint.h>
#include "common/cf32.h"

int nr_extract_symbol_fft(const cf32_t *samples,
                          uint32_t sample_count,
                          uint32_t start_pos,
                          uint32_t cp_len,
                          uint32_t fft_size,
                          cf32_t *out_fft);

#endif
