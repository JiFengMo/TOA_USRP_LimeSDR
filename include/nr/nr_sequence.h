#ifndef NR_SEQUENCE_H
#define NR_SEQUENCE_H

#include <stdint.h>
#include "common/cf32.h"

#define NR_PSS_LEN 127
#define NR_SSS_LEN 127

int nr_generate_pss(uint32_t nid2, cf32_t *out, uint32_t len);
int nr_generate_sss(uint32_t nid1, uint32_t nid2, cf32_t *out, uint32_t len);

#endif
