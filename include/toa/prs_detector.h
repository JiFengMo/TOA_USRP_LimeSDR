#ifndef PRS_DETECTOR_H
#define PRS_DETECTOR_H

#include <stdint.h>
#include "toa/toa_context.h"
#include "toa/toa_types.h"

typedef struct {
  int initialized;
  toa_context_t cfg;
  cf32_t *ref;
  uint32_t ref_len;
  int has_last_peak;
  uint32_t last_peak_pos;
} prs_detector_t;

int prs_detector_init(prs_detector_t *det, const toa_context_t *cfg);
int prs_detector_process(prs_detector_t *det,
                         const toa_frame_t *frame,
                         toa_obs_t *out_obs,
                         uint32_t out_obs_cap,
                         uint32_t *out_obs_num);
void prs_detector_free(prs_detector_t *det);

#endif
