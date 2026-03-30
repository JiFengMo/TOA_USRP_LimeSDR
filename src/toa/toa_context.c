#include "toa/toa_context.h"
#include "common/error.h"

#include <string.h>

int toa_context_init(toa_context_t *ctx)
{
  if (!ctx)
    return TOA_ERR_INVALID_ARG;

  memset(ctx, 0, sizeof(*ctx));
  ctx->sample_rate_hz = 30.72e6;
  ctx->center_freq_hz = 2.593e9;
  ctx->prs_scs_hz = 30e3;
  ctx->pci = 0;
  ctx->n_rb = 106;
  ctx->prs_ref_len = 1024;
  ctx->enable_cfo = 0;
  ctx->search_start = 0;
  ctx->search_len = 0;   /* 0 means full */
  ctx->search_step = 1;
  ctx->track_half_window = 256;
  ctx->min_confidence = 0.05;
  ctx->min_snr_db = 0.0;
  return TOA_OK;
}

void toa_context_reset(toa_context_t *ctx)
{
  if (!ctx)
    return;
  memset(ctx, 0, sizeof(*ctx));
}
