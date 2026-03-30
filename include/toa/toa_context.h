#ifndef TOA_CONTEXT_H
#define TOA_CONTEXT_H

#include <stdint.h>

typedef struct {
  double sample_rate_hz;
  double center_freq_hz;
  double prs_scs_hz;
  uint16_t pci;
  uint32_t n_rb;
  uint32_t prs_ref_len;
  int enable_cfo;
  uint32_t search_start;
  uint32_t search_len;
  uint32_t search_step;
  uint32_t track_half_window;
  double min_confidence;
  double min_snr_db;
} toa_context_t;

int toa_context_init(toa_context_t *ctx);
void toa_context_reset(toa_context_t *ctx);

#endif
