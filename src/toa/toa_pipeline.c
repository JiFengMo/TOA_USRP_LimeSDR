#include "toa/toa_pipeline.h"

#include "common/error.h"

#include <string.h>

int toa_pipeline_init(toa_pipeline_t *pl, const toa_context_t *cfg)
{
  int ret;

  if (!pl || !cfg)
    return TOA_ERR_INVALID_ARG;

  memset(pl, 0, sizeof(*pl));
  pl->ctx = *cfg;

  ret = prs_detector_init(&pl->prs_det, &pl->ctx);
  if (ret != TOA_OK)
    return ret;

  pl->initialized = 1;
  return TOA_OK;
}

int toa_pipeline_process_frame(toa_pipeline_t *pl,
                               const toa_frame_t *frame,
                               toa_obs_t *out_obs,
                               uint32_t out_obs_cap,
                               uint32_t *out_obs_num)
{
  int ret;

  if (!pl || !pl->initialized || !frame || !out_obs_num)
    return TOA_ERR_INVALID_ARG;

  ret = prs_detector_process(&pl->prs_det, frame, out_obs, out_obs_cap, out_obs_num);
  if (ret != TOA_OK)
    return ret;

  pl->frames_processed++;
  return TOA_OK;
}

void toa_pipeline_free(toa_pipeline_t *pl)
{
  if (!pl)
    return;

  prs_detector_free(&pl->prs_det);
  memset(pl, 0, sizeof(*pl));
}
