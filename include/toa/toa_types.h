#ifndef TOA_TYPES_H
#define TOA_TYPES_H

#include <stdint.h>
#include "common/cf32.h"

typedef struct {
  uint32_t gnb_id;
  double x_m;
  double y_m;
  double z_m;
} toa_gnb_position_t;

typedef struct {
  uint32_t gnb_id;
  int64_t toa_ns;
  float snr_db;
  float confidence;
} toa_obs_t;

typedef struct {
  const cf32_t *samples;
  uint64_t frame_id;
  uint32_t sample_count;
  double sample_rate_hz;
  double center_freq_hz;
  double rx_gain_db;
  int64_t hw_timestamp_ns;
} toa_frame_t;

#endif
