#ifndef NR_TYPES_H
#define NR_TYPES_H

#include <stdint.h>

typedef struct {
  double sample_rate_hz;
  uint32_t search_step;
  uint32_t track_half_window;
  float min_metric;
  float min_pss_peak_ratio;
  int sss_enable;
  int32_t sss_lag_samples;
  uint32_t sss_search_half_window;
  float min_sss_metric;
  float min_sss_peak_ratio;
  int lock_nid2;
  int lock_nid1;
  int sss_derotate_cfo;
  float sss_derotate_min_cfo_hz;
} nr_ssb_cfg_t;

typedef struct {
  int found;
  int nid1;
  int nid2;
  int pci;
  uint32_t peak_pos;
  uint32_t sss_pos;
  int64_t toa_ns;
  float metric;
  float pss_peak_ratio;
  float sss_metric;
  float sss_peak_ratio;
  float cfo_hz;
} nr_ssb_result_t;

typedef struct {
  nr_ssb_cfg_t cfg;
  int has_lock;
  uint32_t last_peak_pos;
  int last_nid2;
  int last_nid1;
} nr_ssb_sync_ctx_t;

#endif
