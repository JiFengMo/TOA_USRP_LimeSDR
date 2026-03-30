#ifndef COMMON_CF32_H
#define COMMON_CF32_H

/* Complex float IQ sample (DSP / NR / recorder; not radio-specific). */
typedef struct {
  float r;
  float i;
} cf32_t;

#endif
