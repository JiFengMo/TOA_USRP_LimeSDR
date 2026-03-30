#ifndef PBCH_DECODE_H
#define PBCH_DECODE_H

#include <stdint.h>
#include "common/cf32.h"
#include "nr/nr_types.h"

typedef struct {
  int valid_input;
  int crc_ok;
  uint8_t payload[32];
  uint32_t payload_bits;
  float quality_metric;
} nr_pbch_decode_result_t;

/*
 * PBCH decode knobs (proto chain). Apps read env/CLI and fill this;
 * nr_core does not call getenv.
 */
typedef struct {
  int enable_polar_bitrev;
  int enable_pbch_descramble;
  int enable_polar_info_bitrev;
  int crc_bit_order; /* 0: MSB-first payload then CRC; 1: experimental bit order */
} nr_pbch_decode_cfg_t;

void nr_pbch_decode_cfg_default(nr_pbch_decode_cfg_t *cfg);

int nr_pbch_decode_from_dmrs(const nr_ssb_result_t *ssb_res,
                             const cf32_t *fft_syms,
                             uint32_t n_sym,
                             uint32_t fft_size,
                             float dmrs_metric,
                             float dmrs_peak_ratio,
                             uint32_t dmrs_re_count,
                             uint32_t i_ssb,
                             const nr_pbch_decode_cfg_t *cfg,
                             nr_pbch_decode_result_t *out);

#endif
