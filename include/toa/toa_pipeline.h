#ifndef TOA_PIPELINE_H
#define TOA_PIPELINE_H

#include <stdint.h>
#include "toa/prs_detector.h"
#include "toa/toa_context.h"
#include "toa/toa_types.h"

typedef struct {
  int initialized;
  toa_context_t ctx;
  prs_detector_t prs_det;
  uint64_t frames_processed;
} toa_pipeline_t;

int toa_pipeline_init(toa_pipeline_t *pl, const toa_context_t *cfg);
int toa_pipeline_process_frame(toa_pipeline_t *pl,
                               const toa_frame_t *frame,
                               toa_obs_t *out_obs,
                               uint32_t out_obs_cap,
                               uint32_t *out_obs_num);
void toa_pipeline_free(toa_pipeline_t *pl);

#endif
