#ifndef NR_PBCH_RE_H
#define NR_PBCH_RE_H

#include <stdint.h>
#include "common/cf32.h"

/*
 * Extract PBCH-like REs and produce QPSK LLRs.
 * This is a simplified Stage-3 implementation and will be refined.
 */
int nr_pbch_extract_llr(const cf32_t *fft_syms,
                        uint32_t n_sym,
                        uint32_t fft_size,
                        uint32_t pci,
                        uint32_t i_ssb,
                        float *llr_out,
                        uint32_t llr_cap,
                        uint32_t *llr_num);

#endif
