#include "nr/ssb_sync.h"
#include "nr/nr_sequence.h"
#include "common/error.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

static float cabs2_cf32(cf32_t x)
{
  return x.r * x.r + x.i * x.i;
}

static cf32_t conj_mul(cf32_t a, cf32_t b)
{
  /* conj(a) * b */
  cf32_t y;
  y.r = a.r * b.r + a.i * b.i;
  y.i = a.r * b.i - a.i * b.r;
  return y;
}

static float metric_norm_corr(const cf32_t *x, const cf32_t *ref, uint32_t L, uint32_t pos)
{
  cf32_t acc = {0};
  double ex = 0.0, er = 0.0;

  for (uint32_t k = 0; k < L; ++k) {
    cf32_t a = ref[k];
    cf32_t b = x[pos + k];
    cf32_t p = conj_mul(a, b);
    acc.r += p.r;
    acc.i += p.i;
    ex += (double)b.r * (double)b.r + (double)b.i * (double)b.i;
    er += (double)a.r * (double)a.r + (double)a.i * (double)a.i;
  }

  if (ex <= 0.0 || er <= 0.0)
    return 0.0f;
  return (float)(cabs2_cf32(acc) / (ex * er));
}

static float estimate_cfo_hz_lag1(const cf32_t *x, uint32_t n, double fs)
{
  double sr = 0.0, si = 0.0;
  if (!x || n < 2 || fs <= 0.0)
    return 0.0f;

  for (uint32_t i = 1; i < n; ++i) {
    double ar = x[i].r;
    double ai = x[i].i;
    double br = x[i - 1].r;
    double bi = -x[i - 1].i;
    sr += ar * br - ai * bi;
    si += ar * bi + ai * br;
  }

  return (float)((atan2(si, sr) * fs) / (2.0 * M_PI));
}

int nr_ssb_sync_init(nr_ssb_sync_ctx_t *ctx, const nr_ssb_cfg_t *cfg)
{
  if (!ctx || !cfg)
    return TOA_ERR_INVALID_ARG;

  memset(ctx, 0, sizeof(*ctx));
  ctx->cfg = *cfg;
  if (ctx->cfg.search_step == 0)
    ctx->cfg.search_step = 1;
  if (ctx->cfg.sss_search_half_window == 0)
    ctx->cfg.sss_search_half_window = 32;
  if (ctx->cfg.min_pss_peak_ratio <= 0.0f)
    ctx->cfg.min_pss_peak_ratio = 1.0f;
  if (ctx->cfg.min_sss_peak_ratio <= 0.0f)
    ctx->cfg.min_sss_peak_ratio = 1.0f;
  ctx->last_nid2 = -1;
  ctx->last_nid1 = -1;
  return TOA_OK;
}

void nr_ssb_sync_reset(nr_ssb_sync_ctx_t *ctx)
{
  if (!ctx)
    return;
  ctx->has_lock = 0;
  ctx->last_peak_pos = 0;
  ctx->last_nid2 = -1;
  ctx->last_nid1 = -1;
}

int nr_ssb_sync_scan_block(nr_ssb_sync_ctx_t *ctx,
                           const cf32_t *samples,
                           uint32_t sample_count,
                           int64_t block_time_ns,
                           nr_ssb_result_t *result)
{
  cf32_t pss[3][NR_PSS_LEN];
  cf32_t sss_ref[NR_SSS_LEN];
  float best_metric = -1.0f;
  float second_metric = -1.0f;
  uint32_t best_pos = 0;
  int best_nid2 = -1;
  uint32_t start = 0, end, step;

  if (!ctx || !samples || !result)
    return TOA_ERR_INVALID_ARG;

  memset(result, 0, sizeof(*result));
  result->nid1 = -1;
  result->nid2 = -1;
  result->pci = -1;
  result->sss_pos = 0;
  result->sss_metric = 0.0f;
  result->pss_peak_ratio = 0.0f;
  result->sss_peak_ratio = 0.0f;

  if (sample_count < NR_PSS_LEN)
    return TOA_OK;

  end = sample_count - NR_PSS_LEN + 1;
  step = ctx->cfg.search_step;

  if (ctx->has_lock && ctx->cfg.track_half_window > 0) {
    uint32_t left = (ctx->last_peak_pos > ctx->cfg.track_half_window)
                      ? (ctx->last_peak_pos - ctx->cfg.track_half_window)
                      : 0;
    uint32_t right = ctx->last_peak_pos + ctx->cfg.track_half_window + 1;
    if (right > end)
      right = end;
    start = left;
    end = right;
  }

  for (uint32_t n2 = 0; n2 < 3; ++n2)
    (void)nr_generate_pss(n2, pss[n2], NR_PSS_LEN);

  for (uint32_t n2 = 0; n2 < 3; ++n2) {
    if (ctx->cfg.lock_nid2 && ctx->last_nid2 >= 0 && (int)n2 != ctx->last_nid2)
      continue;
    for (uint32_t pos = start; pos < end; pos += step) {
      float m = metric_norm_corr(samples, pss[n2], NR_PSS_LEN, pos);
      if (m > best_metric) {
        second_metric = best_metric;
        best_metric = m;
        best_pos = pos;
        best_nid2 = (int)n2;
      } else if (m > second_metric) {
        second_metric = m;
      }
    }
  }

  {
    float pss_pr = (second_metric > 0.0f) ? (best_metric / second_metric) : best_metric;

    if (best_metric < ctx->cfg.min_metric || best_nid2 < 0 || pss_pr < ctx->cfg.min_pss_peak_ratio)
      return TOA_OK;

    result->found = 1;
    result->pss_peak_ratio = pss_pr;
  }

  result->nid2 = best_nid2;
  result->peak_pos = best_pos;
  result->toa_ns = block_time_ns + (int64_t)((1e9 * (double)best_pos) / ctx->cfg.sample_rate_hz);
  result->metric = best_metric;
  result->cfo_hz = estimate_cfo_hz_lag1(samples + best_pos, NR_PSS_LEN, ctx->cfg.sample_rate_hz);

  if (ctx->cfg.sss_enable) {
    int64_t nominal = (int64_t)best_pos + (int64_t)ctx->cfg.sss_lag_samples;
    uint32_t sss_start, sss_end;
    float best_sss_metric = -1.0f;
    float second_sss_metric = -1.0f;
    uint32_t best_sss_pos = 0;
    int best_nid1 = -1;
    const cf32_t *sss_work = samples;
    cf32_t *derot = NULL;

    if (nominal < 0)
      nominal = 0;
    if ((uint32_t)nominal >= sample_count)
      nominal = sample_count - 1;

    sss_start = (nominal > (int64_t)ctx->cfg.sss_search_half_window)
                  ? (uint32_t)(nominal - (int64_t)ctx->cfg.sss_search_half_window)
                  : 0;
    sss_end = (uint32_t)nominal + ctx->cfg.sss_search_half_window + 1;
    if (sss_end > sample_count - NR_SSS_LEN + 1)
      sss_end = sample_count - NR_SSS_LEN + 1;

    if (ctx->cfg.sss_derotate_cfo && ctx->cfg.sample_rate_hz > 0.0 && sample_count > 0) {
      float cfo_abs = result->cfo_hz >= 0.0f ? result->cfo_hz : -result->cfo_hz;
      int cfo_ok = (ctx->cfg.sss_derotate_min_cfo_hz <= 0.0f) ||
                   (cfo_abs >= ctx->cfg.sss_derotate_min_cfo_hz);
      derot = NULL;
      if (cfo_ok)
        derot = (cf32_t *)malloc(sizeof(cf32_t) * (size_t)sample_count);
      if (derot) {
        const double fs = ctx->cfg.sample_rate_hz;
        const float cfo = result->cfo_hz;
        for (uint32_t i = 0; i < sample_count; ++i) {
          double t = ((double)(int64_t)i - (double)(int64_t)best_pos) / fs;
          double ph = -2.0 * M_PI * (double)cfo * t;
          double cr = cos(ph);
          double si = sin(ph);
          double ir = (double)samples[i].r;
          double ii = (double)samples[i].i;
          derot[i].r = (float)(ir * cr - ii * si);
          derot[i].i = (float)(ir * si + ii * cr);
        }
        sss_work = derot;
      }
    }

    if (sss_start < sss_end) {
      for (uint32_t nid1 = 0; nid1 < 336; ++nid1) {
        if (ctx->cfg.lock_nid1 && ctx->last_nid1 >= 0 && (int)nid1 != ctx->last_nid1)
          continue;
        (void)nr_generate_sss(nid1, (uint32_t)best_nid2, sss_ref, NR_SSS_LEN);
        for (uint32_t pos = sss_start; pos < sss_end; ++pos) {
          float m = metric_norm_corr(sss_work, sss_ref, NR_SSS_LEN, pos);
          if (m > best_sss_metric) {
            second_sss_metric = best_sss_metric;
            best_sss_metric = m;
            best_sss_pos = pos;
            best_nid1 = (int)nid1;
          } else if (m > second_sss_metric) {
            second_sss_metric = m;
          }
        }
      }
    }

    free(derot);

    result->sss_peak_ratio = (second_sss_metric > 0.0f) ? (best_sss_metric / second_sss_metric) : best_sss_metric;
    if (best_nid1 >= 0 &&
        best_sss_metric >= ctx->cfg.min_sss_metric &&
        result->sss_peak_ratio >= ctx->cfg.min_sss_peak_ratio) {
      result->nid1 = best_nid1;
      result->pci = 3 * best_nid1 + best_nid2;
      result->sss_pos = best_sss_pos;
      result->sss_metric = best_sss_metric;
    }
  }

  ctx->has_lock = 1;
  ctx->last_peak_pos = best_pos;
  ctx->last_nid2 = result->nid2;
  if (result->nid1 >= 0)
    ctx->last_nid1 = result->nid1;
  return TOA_OK;
}
