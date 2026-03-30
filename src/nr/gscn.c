#include "nr/gscn.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

#include "common/error.h"

static toa_error_t fill_n41_raster(uint32_t g_first,
                                  uint32_t g_last,
                                  uint32_t g_step,
                                  uint32_t *tmp,
                                  size_t tmp_cap,
                                  size_t *n_tmp)
{
  size_t m = 0;

  if (!tmp || !n_tmp || tmp_cap == 0 || g_step == 0u)
    return TOA_ERR_INVALID_ARG;
  *n_tmp = 0;

  for (uint64_t g = g_first; g <= (uint64_t)g_last && m < tmp_cap; g += (uint64_t)g_step) {
    double hz;

    if (nr_gscn_fr1_lt_3ghz_to_ss_ref_hz((uint32_t)g, &hz) != TOA_OK)
      continue;
    (void)hz;
    tmp[m++] = (uint32_t)g;
  }

  *n_tmp = m;
  return m > 0 ? TOA_OK : TOA_ERR_INVALID_ARG;
}

static void filter_hz_window(const uint32_t *in,
                            size_t n_in,
                            double lo_hz,
                            double hi_hz,
                            uint32_t *out,
                            size_t out_cap,
                            size_t *n_out)
{
  size_t j = 0;

  for (size_t i = 0; i < n_in && j < out_cap; i++) {
    double hz;

    if (nr_gscn_fr1_lt_3ghz_to_ss_ref_hz(in[i], &hz) != TOA_OK)
      continue;
    if (hz < lo_hz || hz > hi_hz)
      continue;
    out[j++] = in[i];
  }
  *n_out = j;
}

static void subsample_stride(const uint32_t *in,
                            size_t n_in,
                            uint32_t stride,
                            uint32_t *out,
                            size_t out_cap,
                            size_t *n_out)
{
  size_t j = 0;

  if (stride == 0u)
    stride = 1u;
  for (size_t i = 0; i < n_in && j < out_cap; i += (size_t)stride)
    out[j++] = in[i];
  *n_out = j;
}

toa_error_t nr_gscn_fr1_lt_3ghz_to_ss_ref_hz(uint32_t gscn, double *hz_out)
{
  uint32_t N;
  uint32_t M;

  if (!hz_out)
    return TOA_ERR_INVALID_ARG;
  if (gscn < NR_GSCN_FR1_LT_3G_MIN || gscn > NR_GSCN_FR1_LT_3G_MAX)
    return TOA_ERR_INVALID_ARG;

  switch (gscn % 3u) {
  case 0u:
    M = 3u;
    N = gscn / 3u;
    break;
  case 1u:
    M = 5u;
    N = (gscn - 1u) / 3u;
    break;
  default:
    M = 1u;
    N = (gscn + 1u) / 3u;
    break;
  }

  if (N < 1u || N > 2499u)
    return TOA_ERR_INVALID_ARG;

  *hz_out = (1200.0 * (double)N + 50.0 * (double)M) * 1000.0;
  return TOA_OK;
}

toa_error_t nr_gscn_fr1_collect_in_freq_window(double fmin_hz,
                                               double fmax_hz,
                                               uint32_t stride,
                                               uint32_t *out,
                                               size_t out_cap,
                                               size_t *n_out)
{
  uint32_t tmp[8192];
  size_t m = 0;
  size_t j = 0;

  if (!out || !n_out || out_cap == 0)
    return TOA_ERR_INVALID_ARG;
  if (fmin_hz > fmax_hz || stride == 0u)
    return TOA_ERR_INVALID_ARG;

  *n_out = 0;

  for (uint32_t g = NR_GSCN_FR1_LT_3G_MIN; g <= NR_GSCN_FR1_LT_3G_MAX; ++g) {
    double hz;
    if (nr_gscn_fr1_lt_3ghz_to_ss_ref_hz(g, &hz) != TOA_OK)
      continue;
    if (hz < fmin_hz || hz > fmax_hz)
      continue;
    if (m >= sizeof(tmp) / sizeof(tmp[0]))
      break;
    tmp[m++] = g;
  }

  for (size_t i = 0; i < m && j < out_cap; i += (size_t)stride)
    out[j++] = tmp[i];

  *n_out = j;
  return TOA_OK;
}

toa_error_t nr_gscn_fr1_collect_range(uint32_t gscn_first,
                                      uint32_t gscn_last,
                                      uint32_t stride,
                                      uint32_t *out,
                                      size_t out_cap,
                                      size_t *n_out)
{
  size_t j = 0;

  if (!out || !n_out || out_cap == 0 || stride == 0u)
    return TOA_ERR_INVALID_ARG;

  *n_out = 0;

  if (gscn_first > gscn_last)
    return TOA_ERR_INVALID_ARG;

  if (gscn_first < NR_GSCN_FR1_LT_3G_MIN)
    gscn_first = NR_GSCN_FR1_LT_3G_MIN;
  if (gscn_last > NR_GSCN_FR1_LT_3G_MAX)
    gscn_last = NR_GSCN_FR1_LT_3G_MAX;

  for (uint64_t g = gscn_first; g <= (uint64_t)gscn_last && j < out_cap;
       g += (uint64_t)stride) {
    double hz;
    uint32_t gg = (uint32_t)g;

    if (nr_gscn_fr1_lt_3ghz_to_ss_ref_hz(gg, &hz) != TOA_OK)
      continue;
    (void)hz;
    out[j++] = gg;
  }

  *n_out = j;
  return TOA_OK;
}

toa_error_t nr_gscn_collect_for_profile(nr_gscn_profile_t profile,
                                        double ss_ref_lo_hz,
                                        double ss_ref_hi_hz,
                                        uint32_t list_stride,
                                        uint32_t *out,
                                        size_t out_cap,
                                        size_t *n_out)
{
  uint32_t tmp[8192];
  size_t m = 0;
  size_t j = 0;

  if (!out || !n_out || out_cap == 0)
    return TOA_ERR_INVALID_ARG;
  if (list_stride == 0u)
    return TOA_ERR_INVALID_ARG;

  *n_out = 0;

  switch (profile) {
  case NR_GSCN_PROFILE_FR1_FREQ_WINDOW:
    return nr_gscn_fr1_collect_in_freq_window(ss_ref_lo_hz,
                                              ss_ref_hi_hz,
                                              list_stride,
                                              out,
                                              out_cap,
                                              n_out);
  case NR_GSCN_PROFILE_TS38104_N41_SCS30K:
    if (fill_n41_raster(NR_GSCN_N41_SCS30K_FIRST,
                        NR_GSCN_N41_SCS30K_LAST,
                        NR_GSCN_SYNC_RASTER_STEP,
                        tmp,
                        sizeof(tmp) / sizeof(tmp[0]),
                        &m) != TOA_OK)
      return TOA_ERR_INVALID_ARG;
    break;
  case NR_GSCN_PROFILE_TS38104_N41_SCS15K:
    if (fill_n41_raster(NR_GSCN_N41_SCS15K_FIRST,
                        NR_GSCN_N41_SCS15K_LAST,
                        NR_GSCN_SYNC_RASTER_STEP,
                        tmp,
                        sizeof(tmp) / sizeof(tmp[0]),
                        &m) != TOA_OK)
      return TOA_ERR_INVALID_ARG;
    break;
  default:
    return TOA_ERR_INVALID_ARG;
  }

  if (ss_ref_lo_hz < ss_ref_hi_hz) {
    uint32_t tmp2[8192];
    size_t m2 = 0;

    filter_hz_window(tmp,
                     m,
                     ss_ref_lo_hz,
                     ss_ref_hi_hz,
                     tmp2,
                     sizeof(tmp2) / sizeof(tmp2[0]),
                     &m2);
    memcpy(tmp, tmp2, m2 * sizeof(uint32_t));
    m = m2;
    if (m == 0)
      return TOA_OK;
  }

  subsample_stride(tmp, m, list_stride, out, out_cap, &j);
  *n_out = j;
  return TOA_OK;
}

toa_error_t nr_gscn_visible_in_passband(double center_hz,
                                        double bandwidth_hz,
                                        const uint32_t *candidates,
                                        size_t n_candidates,
                                        uint32_t *out,
                                        size_t out_cap,
                                        size_t *n_out)
{
  double lo;
  double hi;
  size_t j = 0;

  if (!candidates || !out || !n_out || out_cap == 0)
    return TOA_ERR_INVALID_ARG;
  if (bandwidth_hz <= 0.0)
    return TOA_ERR_INVALID_ARG;

  *n_out = 0;
  lo = center_hz - 0.5 * bandwidth_hz;
  hi = center_hz + 0.5 * bandwidth_hz;

  for (size_t i = 0; i < n_candidates && j < out_cap; i++) {
    double ss;

    if (nr_gscn_fr1_lt_3ghz_to_ss_ref_hz(candidates[i], &ss) != TOA_OK)
      continue;
    if (ss >= lo && ss <= hi)
      out[j++] = candidates[i];
  }

  *n_out = j;
  return TOA_OK;
}

toa_error_t nr_gscn_tile_centers_hz(double cover_lo_hz,
                                   double cover_hi_hz,
                                   double bw_hz,
                                   double overlap_frac,
                                   double *centers_out,
                                   size_t centers_cap,
                                   size_t *n_centers)
{
  double half;
  double step;
  double fc;

  if (!centers_out || !n_centers || centers_cap == 0)
    return TOA_ERR_INVALID_ARG;
  if (cover_hi_hz <= cover_lo_hz || bw_hz <= 0.0)
    return TOA_ERR_INVALID_ARG;

  if (overlap_frac < 0.0)
    overlap_frac = 0.0;
  if (overlap_frac > 0.9)
    overlap_frac = 0.9;

  *n_centers = 0;

  if (cover_hi_hz - cover_lo_hz <= bw_hz + 1e-6) {
    centers_out[0] = 0.5 * (cover_lo_hz + cover_hi_hz);
    *n_centers = 1;
    return TOA_OK;
  }

  half = 0.5 * bw_hz;
  step = bw_hz * (1.0 - overlap_frac);
  if (step <= 0.0)
    return TOA_ERR_INVALID_ARG;

  fc = cover_lo_hz + half;
  while (fc - half < cover_hi_hz - 1e-3 && *n_centers < centers_cap) {
    centers_out[*n_centers] = fc;
    (*n_centers)++;
    fc += step;
  }

  if (*n_centers == 0u)
    return TOA_ERR_INVALID_ARG;
  return TOA_OK;
}

toa_error_t nr_gscn_snap_to_n41_scs30k_raster(uint32_t g_in, uint32_t *g_out)
{
  const uint32_t g0 = NR_GSCN_N41_SCS30K_FIRST;
  const uint32_t g1 = NR_GSCN_N41_SCS30K_LAST;
  long k;
  long kmax;

  if (!g_out)
    return TOA_ERR_INVALID_ARG;

  if (g_in <= g0) {
    *g_out = g0;
    return TOA_OK;
  }
  if (g_in >= g1) {
    *g_out = g1;
    return TOA_OK;
  }

  kmax = (long)(g1 - g0) / 3L;
  k = lround(((double)g_in - (double)g0) / 3.0);
  if (k < 0L)
    k = 0L;
  if (k > kmax)
    k = kmax;

  *g_out = g0 + (uint32_t)(k * 3L);
  return TOA_OK;
}
