#include "toa/prs_detector.h"
#include "common/error.h"
#include "common/log.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

static uint32_t xorshift32(uint32_t *state)
{
  uint32_t x = *state;
  x ^= x << 13;
  x ^= x >> 17;
  x ^= x << 5;
  *state = x;
  return x;
}

static void generate_qpsk_ref(cf32_t *ref, uint32_t len, uint32_t seed)
{
  uint32_t st = seed ? seed : 1u;
  const float s = 0.7071067811865475f; /* 1/sqrt(2) */
  for (uint32_t i = 0; i < len; ++i) {
    uint32_t r = xorshift32(&st);
    int b0 = (r >> 0) & 1;
    int b1 = (r >> 1) & 1;
    ref[i].r = b0 ? s : -s;
    ref[i].i = b1 ? s : -s;
  }
}

static float cabs2f_cf32(cf32_t x)
{
  return x.r * x.r + x.i * x.i;
}

static cf32_t cf32_add(cf32_t a, cf32_t b)
{
  cf32_t y;
  y.r = a.r + b.r;
  y.i = a.i + b.i;
  return y;
}

static cf32_t conj_of(cf32_t a)
{
  cf32_t y = a;
  y.i = -y.i;
  return y;
}

static cf32_t conj_mul(cf32_t a_conj, cf32_t b)
{
  cf32_t y;
  y.r = a_conj.r * b.r + a_conj.i * b.i;
  y.i = a_conj.r * b.i - a_conj.i * b.r;
  return y;
}

static double estimate_cfo_hz_lag1(const cf32_t *x, uint32_t n, double fs)
{
  if (!x || n < 2 || fs <= 0.0)
    return 0.0;

  double sr = 0.0, si = 0.0;
  for (uint32_t i = 1; i < n; ++i) {
    /* x[i] * conj(x[i-1]) */
    double ar = x[i].r;
    double ai = x[i].i;
    double br = x[i - 1].r;
    double bi = -x[i - 1].i;
    sr += ar * br - ai * bi;
    si += ar * bi + ai * br;
  }
  double phase = atan2(si, sr);
  return (phase * fs) / (2.0 * M_PI);
}

static void cfo_compensate_inplace(cf32_t *x, uint32_t n, double fs, double cfo_hz)
{
  if (!x || n == 0 || fs <= 0.0)
    return;

  const double w = -2.0 * M_PI * cfo_hz / fs; /* per-sample phase step */
  double ph = 0.0;
  for (uint32_t i = 0; i < n; ++i) {
    double c = cos(ph);
    double s = sin(ph);
    float r = x[i].r;
    float im = x[i].i;
    x[i].r = (float)(r * c - im * s);
    x[i].i = (float)(r * s + im * c);
    ph += w;
  }
}

static float metric_norm_xcorr(const cf32_t *x, const cf32_t *ref, uint32_t L, uint32_t pos)
{
  cf32_t acc = (cf32_t){0};
  double ex = 0.0;
  double er = 0.0;

  for (uint32_t k = 0; k < L; ++k) {
    cf32_t a = conj_of(ref[k]);
    cf32_t b = x[pos + k];
    cf32_t p = conj_mul(a, b);
    acc = cf32_add(acc, p);
    ex += (double)b.r * (double)b.r + (double)b.i * (double)b.i;
    er += (double)ref[k].r * (double)ref[k].r + (double)ref[k].i * (double)ref[k].i;
  }

  double denom = ex * er;
  if (denom <= 0.0)
    return 0.0f;

  return (float)(cabs2f_cf32(acc) / denom);
}

int prs_detector_init(prs_detector_t *det, const toa_context_t *cfg)
{
  if (!det || !cfg)
    return TOA_ERR_INVALID_ARG;

  memset(det, 0, sizeof(*det));
  det->cfg = *cfg;
  det->ref_len = cfg->prs_ref_len ? cfg->prs_ref_len : 1024;
  det->ref = (cf32_t *)calloc(det->ref_len, sizeof(cf32_t));
  if (!det->ref)
    return TOA_ERR_NO_MEMORY;

  generate_qpsk_ref(det->ref, det->ref_len, (uint32_t)cfg->pci + 1u);
  det->initialized = 1;
  return TOA_OK;
}

int prs_detector_process(prs_detector_t *det,
                         const toa_frame_t *frame,
                         toa_obs_t *out_obs,
                         uint32_t out_obs_cap,
                         uint32_t *out_obs_num)
{
  if (!det || !det->initialized || !frame || !out_obs_num)
    return TOA_ERR_INVALID_ARG;

  *out_obs_num = 0;
  if (!out_obs || out_obs_cap == 0)
    return TOA_OK;

  if (!frame->samples || frame->sample_count < det->ref_len)
    return TOA_OK;

  const uint32_t n = frame->sample_count;
  const uint32_t L = det->ref_len;
  const uint32_t npos = n - L + 1;

  double fs = (frame->sample_rate_hz > 0.0) ? frame->sample_rate_hz : det->cfg.sample_rate_hz;
  if (fs <= 0.0) {
    log_msg(LOG_LEVEL_WARN, "PRS", "invalid fs, cannot compute toa");
    return TOA_OK;
  }

  /* Optional CFO estimation/compensation on a temporary copy */
  const cf32_t *x = frame->samples;
  cf32_t *xbuf = NULL;
  double cfo_hz = 0.0;
  if (det->cfg.enable_cfo) {
    xbuf = (cf32_t *)calloc(n, sizeof(cf32_t));
    if (!xbuf)
      return TOA_ERR_NO_MEMORY;
    memcpy(xbuf, frame->samples, n * sizeof(cf32_t));
    cfo_hz = estimate_cfo_hz_lag1(xbuf, n, fs);
    cfo_compensate_inplace(xbuf, n, fs, cfo_hz);
    x = xbuf;
  }

  /* Search window/step; optionally refine with tracked peak window */
  uint32_t start = det->cfg.search_start;
  uint32_t end = npos;
  if (start >= npos)
    start = 0;
  if (det->cfg.search_len > 0) {
    uint32_t e = start + det->cfg.search_len;
    if (e < end)
      end = e;
  }
  uint32_t step = det->cfg.search_step ? det->cfg.search_step : 1;

  if (det->has_last_peak && det->cfg.track_half_window > 0) {
    uint32_t left = (det->last_peak_pos > det->cfg.track_half_window)
                      ? (det->last_peak_pos - det->cfg.track_half_window)
                      : 0;
    uint32_t right = det->last_peak_pos + det->cfg.track_half_window + 1;
    if (right > npos)
      right = npos;
    if (left > start)
      start = left;
    if (right < end)
      end = right;
    if (start >= end) {
      start = 0;
      end = npos;
    }
  }

  float best = -1.0f;
  float second = -1.0f;
  uint32_t best_pos = 0;

  for (uint32_t pos = start; pos < end; pos += step) {
    float metric = metric_norm_xcorr(x, det->ref, L, pos);
    if (metric > best) {
      second = best;
      best = metric;
      best_pos = pos;
    } else if (metric > second) {
      second = metric;
    }
  }

  if (best <= 0.0f)
    goto out_ok;

  /* 3-point parabolic interpolation for sub-sample TOA */
  float delta = 0.0f;
  if (step == 1) {
    float m1 = (best_pos > 0) ? metric_norm_xcorr(x, det->ref, L, best_pos - 1) : best;
    float m2 = best;
    float m3 = (best_pos + 1 < npos) ? metric_norm_xcorr(x, det->ref, L, best_pos + 1) : best;
    float denom = (m1 - 2.0f * m2 + m3);
    if (fabsf(denom) > 1e-12f) {
      delta = 0.5f * (m1 - m3) / denom;
      if (delta > 0.5f)
        delta = 0.5f;
      if (delta < -0.5f)
        delta = -0.5f;
    }
  }

  float peak_ratio = (second > 0.0f) ? (best / second) : best;
  float snr_db = 10.0f * log10f(peak_ratio + 1e-12f);
  float conf = (peak_ratio > 1.0f) ? (1.0f - 1.0f / peak_ratio) : 0.0f;

  double pos_refined = (double)best_pos + (double)delta;
  int64_t toa_ns = frame->hw_timestamp_ns + (int64_t)((1e9 * pos_refined) / fs);

  out_obs[0].gnb_id = 0;
  out_obs[0].toa_ns = toa_ns;
  out_obs[0].snr_db = snr_db;
  out_obs[0].confidence = conf;
  if (conf >= (float)det->cfg.min_confidence && snr_db >= (float)det->cfg.min_snr_db) {
    *out_obs_num = 1;
    det->has_last_peak = 1;
    det->last_peak_pos = best_pos;
  } else {
    *out_obs_num = 0;
  }

out_ok:
  free(xbuf);
  return TOA_OK;
}

void prs_detector_free(prs_detector_t *det)
{
  if (!det)
    return;
  free(det->ref);
  memset(det, 0, sizeof(*det));
}
