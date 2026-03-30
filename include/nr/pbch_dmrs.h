#ifndef PBCH_DMRS_H
#define PBCH_DMRS_H

#include <stdint.h>
#include "common/cf32.h"

#define NR_PBCH_DMRS_MAX_RE 180

typedef struct {
  int valid;
  float metric;
  float peak_ratio;
  uint32_t re_count;
} nr_pbch_dmrs_result_t;

int nr_generate_pbch_dmrs(uint32_t pci, uint32_t i_ssb, cf32_t *out, uint32_t len);

int nr_pbch_dmrs_correlate(const cf32_t *fft_syms,
                           uint32_t n_sym,
                           uint32_t fft_size,
                           uint32_t pci,
                           uint32_t i_ssb,
                           nr_pbch_dmrs_result_t *res);

#endif
