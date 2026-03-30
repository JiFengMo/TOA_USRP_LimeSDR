#ifndef NR_MIB_H
#define NR_MIB_H

#include <stdint.h>
#include "nr/pbch_decode.h"

typedef struct {
  int valid;
  uint16_t sfn;
  uint8_t scs_common;
  uint8_t dmrs_type_a_pos;
  uint8_t pdcch_cfg_sib1;
  uint8_t cell_barred;
  uint8_t intra_freq_reselection;
} nr_mib_t;

int nr_mib_unpack(const nr_pbch_decode_result_t *pbch, nr_mib_t *mib);

#endif
