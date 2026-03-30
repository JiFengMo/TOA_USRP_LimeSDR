#ifndef NR_POLAR_H
#define NR_POLAR_H

#include <stdint.h>

int nr_polar_build_frozen_set(uint32_t n, uint32_t k, uint8_t *frozen);
int nr_polar_sc_decode(const float *llr, uint32_t n, const uint8_t *frozen, uint8_t *u_hat);

#endif
