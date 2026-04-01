#include "openair1/PHY/NR_POSITIONING/nr_pos_types.h"
#include "openair1/PHY/NR_POSITIONING/nr_pos_api.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define NR_V0_MIN_PWR 100.0
#define NR_V0_FS_HZ_FALLBACK 30720000.0
#define NR_V0_PSS_LEN 127U

static float nr_v0_corr_metric_comp(const c16_t *x,
                                    uint32_t pss_start,
                                    const float *pss_i,
                                    const float *pss_q,
                                    uint32_t ref_len,
                                    float cfo_hz,
                                    float fs_hz)
{
  const double w = 2.0 * M_PI * (double)cfo_hz / (double)fs_hz;
  double cr = 0.0;
  double ci = 0.0;
  double px = 0.0;
  double pp = 0.0;
  for (uint32_t k = 0; k < ref_len; k++) {
    double xr = (double)x[pss_start + k].r;
    double xq = (double)x[pss_start + k].i;
    if (cfo_hz != 0.0f) {
      double ph = w * (double)k;
      double c = cos(ph);
      double s = sin(ph);
      double tr = xr * c + xq * s;
      double tq = -xr * s + xq * c;
      xr = tr;
      xq = tq;
    }
    double pr = (double)pss_i[k];
    double pq = (double)pss_q[k];
    cr += xr * pr + xq * pq;
    ci += xq * pr - xr * pq;
    px += xr * xr + xq * xq;
    pp += pr * pr + pq * pq;
  }
  return (float)(sqrt(cr * cr + ci * ci) / (sqrt(px * pp) + 1e-9));
}

static float nr_v0_estimate_cfo_hz(const c16_t *x,
                                   uint32_t pss_start,
                                   uint32_t len,
                                   float fs_hz)
{
  const uint32_t nfft = nr_v0_ssb_nfft(fs_hz);
  uint32_t cp = 0U;
  double sr = 0.0;
  double si = 0.0;
  if (!x || len <= nfft) {
    return 0.0f;
  }
  cp = len - nfft;
  if (cp == 0U) {
    cp = nr_v0_ssb_cp_len(fs_hz);
  }
  for (uint32_t n = 0U; n < cp; n++) {
    const double ar = (double)x[pss_start + n].r;
    const double ai = (double)x[pss_start + n].i;
    const double br = (double)x[pss_start + nfft + n].r;
    const double bi = (double)x[pss_start + nfft + n].i;
    sr += ar * br + ai * bi;
    si += ai * br - ar * bi;
  }
  double ph = atan2(si, sr);
  return (float)(ph * fs_hz / (2.0 * M_PI * (double)nfft));
}

static int nr_v0_extract_sss_bins(const c16_t *x, uint32_t sss_start,
                                  uint32_t nsamps, uint32_t nfft, uint32_t cp,
                                  float cfo_hz, float fs_hz,
                                  float *rx_i, float *rx_q)
{
  if (!x || !rx_i || !rx_q) {
    return -1;
  }
  if ((uint64_t)sss_start + cp + nfft > nsamps) {
    return -1;
  }
  const double w = 2.0 * M_PI * (double)cfo_hz / (double)fs_hz;
  const uint32_t fft_start = sss_start + cp;
  for (uint32_t m = 0; m < NR_V0_PSS_LEN; m++) {
    const int k = (int)m - 64;
    double sr = 0.0;
    double si = 0.0;
    for (uint32_t n = 0; n < nfft; n++) {
      double xr = (double)x[fft_start + n].r;
      double xq = (double)x[fft_start + n].i;
      if (cfo_hz != 0.0f) {
        const double ph_cfo = w * (double)n;
        const double cc = cos(ph_cfo);
        const double ss = sin(ph_cfo);
        const double tr = xr * cc + xq * ss;
        const double tq = -xr * ss + xq * cc;
        xr = tr;
        xq = tq;
      }
      const double ph = -2.0 * M_PI * (double)k * (double)n / (double)nfft;
      const double c = cos(ph);
      const double s = sin(ph);
      sr += xr * c - xq * s;
      si += xr * s + xq * c;
    }
    rx_i[m] = (float)(sr / (double)nfft);
    rx_q[m] = (float)(si / (double)nfft);
  }
  return 0;
}

static int nr_pbch_dmrs_rel_valid(uint8_t sym, int rel, uint8_t v)
{
  if (rel < 0 || rel >= 240) {
    return 0;
  }
  if ((((uint32_t)rel) & 3U) != v) {
    return 0;
  }
  if (sym == 1U || sym == 3U) {
    return 1;
  }
  if (sym == 2U) {
    return (rel < 48 || rel >= 192);
  }
  return 0;
}

static void nr_pbch_smooth_dmrs_channel(const cf32_t h_raw[NR_SSB_RE_ROWS][NR_SSB_RE_COLS],
                                        const uint8_t h_valid[NR_SSB_RE_ROWS][NR_SSB_RE_COLS],
                                        cf32_t h_out[NR_SSB_RE_ROWS][NR_SSB_RE_COLS])
{
  memset(h_out, 0, sizeof(cf32_t) * NR_SSB_RE_ROWS * NR_SSB_RE_COLS);
  for (uint8_t sym = 0U; sym < NR_SSB_RE_ROWS; sym++) {
    for (uint16_t rel = 0U; rel < NR_SSB_RE_COLS; rel++) {
      float wsum = 0.0f;
      cf32_t acc = {0};
      if (!h_valid[sym][rel]) {
        continue;
      }
      for (int dt = -1; dt <= 1; dt++) {
        const int sym2 = (int)sym + dt;
        const float wt = (dt == 0) ? 0.75f : 0.125f;
        if (sym2 < 0 || sym2 >= NR_SSB_RE_ROWS) {
          continue;
        }
        for (int df = -4; df <= 4; df += 4) {
          const int rel2 = (int)rel + df;
          const float wf = (df == 0) ? 0.7f : 0.15f;
          const float w = wt * wf;
          if (rel2 < 0 || rel2 >= NR_SSB_RE_COLS) {
            continue;
          }
          if (!h_valid[sym2][rel2]) {
            continue;
          }
          acc.r += w * h_raw[sym2][rel2].r;
          acc.i += w * h_raw[sym2][rel2].i;
          wsum += w;
        }
      }
      if (wsum > 0.0f) {
        h_out[sym][rel].r = acc.r / wsum;
        h_out[sym][rel].i = acc.i / wsum;
      } else {
        h_out[sym][rel] = h_raw[sym][rel];
      }
    }
  }
}

static int nr_ssb_extract_window_shifted(const nr_iq_block_t *blk,
                                         const nr_sync_state_t *sync,
                                         int32_t delta_samp,
                                         nr_ssb_window_t *win)
{
  int64_t start = 0;
  uint32_t want = 0U;
  uint32_t rem = 0U;
  if (!blk || !sync || !win) {
    return -1;
  }
  memset(win, 0, sizeof(*win));
  start = (int64_t)sync->coarse_offset_samp +
          sync->cum_tracking_shift_samp +
          (int64_t)delta_samp;
  if (start < 0) {
    start = 0;
  }
  if ((uint64_t)start >= blk->nsamps) {
    start = 0;
  }
  win->start_samp = (uint32_t)start;
  want = nr_v0_ssb_burst_len_fs(blk->fs_hz);
  rem = blk->nsamps - win->start_samp;
  win->len_samp = (rem > want) ? want : rem;
  return 0;
}

static float nr_pbch_dmrs_metric_estimate(const nr_ssb_grid_t *grid,
                                          uint16_t pci,
                                          uint8_t ssb_idx,
                                          uint8_t v,
                                          cf32_t h_est[NR_SSB_RE_ROWS][NR_SSB_RE_COLS],
                                          uint8_t h_valid[NR_SSB_RE_ROWS][NR_SSB_RE_COLS])
{
  cf32_t h_raw[NR_SSB_RE_ROWS][NR_SSB_RE_COLS];
  uint16_t dmrs_rel[144];
  uint8_t dmrs_sym[144];
  float dmrs_i[144];
  float dmrs_q[144];
  double cr = 0.0;
  double ci = 0.0;
  double px = 0.0;
  const double pr = 144.0;

  memset(h_est, 0, sizeof(cf32_t) * NR_SSB_RE_ROWS * NR_SSB_RE_COLS);
  memset(h_raw, 0, sizeof(h_raw));
  memset(h_valid, 0, sizeof(uint8_t) * NR_SSB_RE_ROWS * NR_SSB_RE_COLS);
  if (!grid || !grid->valid) {
    return -1.0f;
  }
  if (nr_pbch_dmrs_re_positions(v, dmrs_rel, dmrs_sym, 144U) != 144U) {
    return -1.0f;
  }
  if (nr_v0_pbch_dmrs_build(pci, ssb_idx, 0, dmrs_i, dmrs_q, 144U) != 144) {
    return -1.0f;
  }

  for (uint32_t m = 0U; m < 144U; m++) {
    const uint8_t sym = dmrs_sym[m];
    const uint16_t rel = dmrs_rel[m];
    const cf32_t y = grid->re[sym][rel];
    const float xr = dmrs_i[m];
    const float xi = dmrs_q[m];
    const float den = xr * xr + xi * xi + 1.0e-6f;
    cr += (double)y.r * (double)xr + (double)y.i * (double)xi;
    ci += (double)y.i * (double)xr - (double)y.r * (double)xi;
    px += (double)y.r * (double)y.r + (double)y.i * (double)y.i;
    h_raw[sym][rel].r = (y.r * xr + y.i * xi) / den;
    h_raw[sym][rel].i = (y.i * xr - y.r * xi) / den;
    h_valid[sym][rel] = 1U;
  }

  nr_pbch_smooth_dmrs_channel(h_raw, h_valid, h_est);

  return (float)(sqrt(cr * cr + ci * ci) / (sqrt(px * pr) + 1e-9));
}

static int nr_pbch_interp_h(const cf32_t h_est[NR_SSB_RE_ROWS][NR_SSB_RE_COLS],
                            const uint8_t h_valid[NR_SSB_RE_ROWS][NR_SSB_RE_COLS],
                            uint8_t sym,
                            uint16_t rel,
                            cf32_t *hout)
{
  int left = -1;
  int right = -1;
  if (!h_est || !h_valid || !hout || sym >= NR_SSB_RE_ROWS || rel >= NR_SSB_RE_COLS) {
    return -1;
  }
  for (int k = (int)rel; k >= 0; k--) {
    if (h_valid[sym][k]) {
      left = k;
      break;
    }
  }
  for (int k = (int)rel; k < NR_SSB_RE_COLS; k++) {
    if (h_valid[sym][k]) {
      right = k;
      break;
    }
  }
  if (left < 0 && right < 0) {
    return -1;
  }
  if (left < 0) {
    *hout = h_est[sym][right];
    return 0;
  }
  if (right < 0 || right == left) {
    *hout = h_est[sym][left];
    return 0;
  }
  {
    const float a = (float)((int)rel - left) / (float)(right - left);
    hout->r = (1.0f - a) * h_est[sym][left].r + a * h_est[sym][right].r;
    hout->i = (1.0f - a) * h_est[sym][left].i + a * h_est[sym][right].i;
  }
  return 0;
}

static int nr_pbch_build_llr_from_grid(const nr_ssb_grid_t *grid,
                                       const cf32_t h_est[NR_SSB_RE_ROWS][NR_SSB_RE_COLS],
                                       const uint8_t h_valid[NR_SSB_RE_ROWS][NR_SSB_RE_COLS],
                                       uint8_t v,
                                       float noise_var,
                                       float cpe_rad,
                                       float *llr)
{
  uint16_t data_rel[432];
  uint8_t data_sym[432];
  const float cph = cosf(-cpe_rad);
  const float sph = sinf(-cpe_rad);
  if (!grid || !grid->valid || !llr) {
    return -1;
  }
  if (nr_pbch_data_re_positions(v, data_rel, data_sym, 432U) != 432U) {
    return -1;
  }
  for (uint32_t m = 0U; m < 432U; m++) {
    const uint8_t sym = data_sym[m];
    const uint16_t rel = data_rel[m];
    const cf32_t y = grid->re[sym][rel];
    cf32_t h = {0};
    float den = 0.0f;
    float zr = 0.0f;
    float zi = 0.0f;
    if (nr_pbch_interp_h(h_est, h_valid, sym, rel, &h) != 0) {
      return -1;
    }
    den = h.r * h.r + h.i * h.i + 1.0e-6f;
    zr = (y.r * h.r + y.i * h.i) / den;
    zi = (y.i * h.r - y.r * h.i) / den;
    {
      const float rr = zr * cph - zi * sph;
      const float ri = zr * sph + zi * cph;
      const float gain = 2.0f / fmaxf(noise_var, 0.05f);
      llr[2U * m] = gain * rr;
      llr[2U * m + 1U] = gain * ri;
    }
  }
  return 0;
}

static float nr_pbch_estimate_noise_cpe(const nr_ssb_grid_t *grid,
                                        uint16_t pci,
                                        uint8_t ssb_idx,
                                        uint8_t v,
                                        const cf32_t h_est[NR_SSB_RE_ROWS][NR_SSB_RE_COLS],
                                        const uint8_t h_valid[NR_SSB_RE_ROWS][NR_SSB_RE_COLS],
                                        float *cpe_rad)
{
  uint16_t dmrs_rel[144];
  uint8_t dmrs_sym[144];
  float dmrs_i[144];
  float dmrs_q[144];
  double err_pow = 0.0;
  double sig_pow = 0.0;
  double cr = 0.0;
  double ci = 0.0;
  uint32_t used = 0U;
  if (cpe_rad) {
    *cpe_rad = 0.0f;
  }
  if (!grid || !grid->valid) {
    return 1.0f;
  }
  if (nr_pbch_dmrs_re_positions(v, dmrs_rel, dmrs_sym, 144U) != 144U) {
    return 1.0f;
  }
  if (nr_v0_pbch_dmrs_build(pci, ssb_idx, 0, dmrs_i, dmrs_q, 144U) != 144) {
    return 1.0f;
  }
  for (uint32_t m = 0U; m < 144U; m++) {
    const uint8_t sym = dmrs_sym[m];
    const uint16_t rel = dmrs_rel[m];
    const cf32_t y = grid->re[sym][rel];
    const cf32_t h = h_est[sym][rel];
    const float xr = dmrs_i[m];
    const float xi = dmrs_q[m];
    const float pr = h.r * xr - h.i * xi;
    const float pi = h.r * xi + h.i * xr;
    const float er = y.r - pr;
    const float ei = y.i - pi;
    if (!h_valid[sym][rel]) {
      continue;
    }
    err_pow += (double)er * (double)er + (double)ei * (double)ei;
    sig_pow += (double)pr * (double)pr + (double)pi * (double)pi;
    cr += (double)y.r * (double)pr + (double)y.i * (double)pi;
    ci += (double)y.i * (double)pr - (double)y.r * (double)pi;
    used++;
  }
  if (used == 0U) {
    return 1.0f;
  }
  if (cpe_rad) {
    *cpe_rad = (float)atan2(ci, cr);
  }
  return (float)((err_pow / (double)used) / ((sig_pow / (double)used) + 1.0e-6));
}

typedef struct {
  uint16_t pci;
  uint8_t grid_idx;
  int16_t sss_delta_bias;
  int32_t timing_delta;
  float cfo_delta_hz;
  uint8_t ssb_idx;
  float metric;
} nr_pbch_hyp_t;

static void nr_pbch_insert_hyp(nr_pbch_hyp_t *hyps, uint32_t max_hyps,
                               uint32_t *count, nr_pbch_hyp_t cand)
{
  uint32_t n = 0U;
  uint32_t pos = 0U;
  if (!hyps || !count || max_hyps == 0U) {
    return;
  }
  n = (*count < max_hyps) ? *count : max_hyps;
  while (pos < n && hyps[pos].metric >= cand.metric) {
    pos++;
  }
  if (pos >= max_hyps) {
    return;
  }
  if (n < max_hyps) {
    n++;
    *count = n;
  }
  for (uint32_t i = n - 1U; i > pos; i--) {
    hyps[i] = hyps[i - 1U];
  }
  hyps[pos] = cand;
}

int nr_ssb_pss_search(const nr_iq_block_t *blk, nr_pss_hit_t *hits, int max_hits)
{
  if (!blk || !hits || max_hits <= 0) {
    return -1;
  }
  const float fs_hz = (blk->fs_hz > 0.0) ? (float)blk->fs_hz : NR_V0_FS_HZ_FALLBACK;
  const uint32_t ref_len = nr_v0_ssb_sym_len_fs(fs_hz);
  memset(hits, 0, sizeof(nr_pss_hit_t) * (size_t)max_hits);
  if (!blk->rx[0] || blk->nsamps < ref_len + 1U) {
    return -1;
  }

  const c16_t *x = blk->rx[0];
  double avg_pwr = 0.0;
  for (uint32_t n = 0; n < blk->nsamps; n++) {
    const double xr = (double)x[n].r;
    const double xq = (double)x[n].i;
    avg_pwr += xr * xr + xq * xq;
  }
  avg_pwr /= (double)blk->nsamps;
  if (avg_pwr < NR_V0_MIN_PWR) {
    return -1;
  }

  float *pss_i[3] = {0};
  float *pss_q[3] = {0};
  for (int nid2 = 0; nid2 < 3; nid2++) {
    pss_i[nid2] = (float *)malloc(sizeof(float) * (size_t)ref_len);
    pss_q[nid2] = (float *)malloc(sizeof(float) * (size_t)ref_len);
    if (!pss_i[nid2] || !pss_q[nid2]) {
      for (int j = 0; j <= nid2; j++) {
        free(pss_i[j]);
        free(pss_q[j]);
      }
      return -1;
    }
    nr_v0_pss_build_td_f_fs(nid2, fs_hz, pss_i[nid2], pss_q[nid2], ref_len);
  }

  int found = 0;
  double metric_sum = 0.0;
  double metric_sq = 0.0;
  uint64_t metric_cnt = 0;
  /* Scan PSS symbol0 start directly (do not require full 4-symbol burst fit). */
  for (uint32_t n = 0; n + ref_len < blk->nsamps; n++) {
    const uint32_t pss_start = n;
    for (int nid2 = 0; nid2 < 3; nid2++) {
      float metric = nr_v0_corr_metric_comp(x, pss_start, pss_i[nid2], pss_q[nid2],
                                            ref_len, 0.0f, fs_hz);
      metric_sum += metric;
      metric_sq += (double)metric * (double)metric;
      metric_cnt++;
      int insert = found < max_hits ? found : (max_hits - 1);
      if (insert >= 0) {
        /* keep hits sorted by metric descending */
        while (insert > 0 && hits[insert - 1].metric < metric) {
          if (insert < max_hits) {
            hits[insert] = hits[insert - 1];
          }
          insert--;
        }
        if (insert < max_hits) {
          hits[insert].peak_samp = (int32_t)n;
          hits[insert].coarse_cfo_hz = 0.0f;
          hits[insert].metric = metric;
          hits[insert].nid2 = (uint8_t)nid2;
          if (found < max_hits) {
            found++;
          }
        }
      }
    }
  }
  if (found <= 0) {
    for (int nid2 = 0; nid2 < 3; nid2++) {
      free(pss_i[nid2]);
      free(pss_q[nid2]);
    }
    return -1;
  }
  double mean = (metric_cnt > 0) ? (metric_sum / (double)metric_cnt) : 0.0;
  double var = (metric_cnt > 1)
                   ? (metric_sq / (double)metric_cnt - mean * mean)
                   : 0.0;
  if (var < 0.0) {
    var = 0.0;
  }
  const float adaptive_th = (float)fmax(0.12, mean + 4.0 * sqrt(var));
  if (hits[0].metric < adaptive_th) {
    for (int nid2 = 0; nid2 < 3; nid2++) {
      free(pss_i[nid2]);
      free(pss_q[nid2]);
    }
    return -1;
  }
  if (found >= 2 && hits[1].metric > 0.0f) {
    /* Additive anti-false-alarm discriminator: winner must be clearly better
     * than runner-up when scanning many hypotheses. */
    const int32_t sep = hits[0].peak_samp - hits[1].peak_samp;
    const int32_t min_sep = (int32_t)(nr_v0_ssb_cp_len(fs_hz) / 2U);
    if ((sep >= min_sep || sep <= -min_sep) &&
        hits[0].metric < (1.03f * hits[1].metric)) {
      for (int nid2 = 0; nid2 < 3; nid2++) {
        free(pss_i[nid2]);
        free(pss_q[nid2]);
      }
      return -1;
    }
  }

  /* Estimate coarse CFO then re-score top candidate with compensation. */
  uint32_t pss0 = (uint32_t)hits[0].peak_samp;
  float cfo = nr_v0_estimate_cfo_hz(x, pss0, ref_len, fs_hz);
  if (cfo > 15000.0f) cfo = 15000.0f;
  if (cfo < -15000.0f) cfo = -15000.0f;
  float best_m = -1.0f;
  uint8_t best_nid2 = hits[0].nid2;
  for (int nid2 = 0; nid2 < 3; nid2++) {
    float m = nr_v0_corr_metric_comp(x, pss0, pss_i[nid2], pss_q[nid2], ref_len, cfo, fs_hz);
    if (m > best_m) {
      best_m = m;
      best_nid2 = (uint8_t)nid2;
    }
  }
  /* Local sharpness check around the winning position to reject flat/noisy peaks. */
  {
    const uint32_t probe = nr_v0_ssb_cp_len(fs_hz) / 2U;
    if (probe > 0U && pss0 > probe && (pss0 + probe + ref_len) < blk->nsamps) {
      float m_l = nr_v0_corr_metric_comp(x, pss0 - probe, pss_i[best_nid2],
                                         pss_q[best_nid2], ref_len, cfo, fs_hz);
      float m_r = nr_v0_corr_metric_comp(x, pss0 + probe, pss_i[best_nid2],
                                         pss_q[best_nid2], ref_len, cfo, fs_hz);
    float side = (m_l > m_r) ? m_l : m_r;
      if (best_m < 1.005f * side) {
        for (int nid2 = 0; nid2 < 3; nid2++) {
          free(pss_i[nid2]);
          free(pss_q[nid2]);
        }
        return -1;
      }
    }
  }
  hits[0].nid2 = best_nid2;
  hits[0].metric = best_m;
  hits[0].coarse_cfo_hz = cfo;
  for (int nid2 = 0; nid2 < 3; nid2++) {
    free(pss_i[nid2]);
    free(pss_q[nid2]);
  }
  return 0;
}

typedef struct {
  uint16_t nid1;
  int16_t delta;
  float metric;
} nr_sss_hyp_t;

static void nr_sss_insert_hyp(nr_sss_hyp_t *hyps, uint32_t max_hyps, nr_sss_hyp_t cand)
{
  uint32_t pos = 0U;
  uint32_t n = 0U;
  if (!hyps || max_hyps == 0U) {
    return;
  }
  while (n < max_hyps && hyps[n].metric > 0.0f) {
    if (hyps[n].nid1 == cand.nid1) {
      if (cand.metric > hyps[n].metric) {
        hyps[n] = cand;
      }
      return;
    }
    n++;
  }
  while (pos < n && hyps[pos].metric >= cand.metric) {
    pos++;
  }
  if (pos >= max_hyps) {
    return;
  }
  if (n < max_hyps) {
    n++;
  }
  for (uint32_t i = n - 1U; i > pos; i--) {
    hyps[i] = hyps[i - 1U];
  }
  hyps[pos] = cand;
}

int nr_ssb_refine_sync(const nr_iq_block_t *blk, const nr_pss_hit_t *hit,
                       nr_sync_state_t *sync)
{
  enum { MAX_SSS_HYPS = 4 };
  nr_sss_hyp_t sss_hyps[MAX_SSS_HYPS];
  if (!blk || !hit || !sync) {
    return -1;
  }
  const float fs_hz = (blk->fs_hz > 0.0) ? (float)blk->fs_hz : NR_V0_FS_HZ_FALLBACK;
  memset(sync, 0, sizeof(*sync));
  memset(sss_hyps, 0, sizeof(sss_hyps));

  const uint32_t ref_len = nr_v0_ssb_sym_len_fs(fs_hz);
  const uint32_t sym_len = ref_len;
  if (hit->metric < 0.12f) {
    sync->locked = 0;
    return -1;
  }

  const c16_t *x = blk->rx[0];
  const float cfo_hz = hit->coarse_cfo_hz;
  const uint8_t nid2 = hit->nid2;

  enum { NID1_MAX = 336 };
  int best_nid1 = 0;
  float best_metric = -1.0f;
  int best_sss_delta = 0;
  double metric_sum = 0.0;
  double metric_sq = 0.0;
  uint64_t cnt = 0;
  float rx_i[NR_V0_PSS_LEN];
  float rx_q[NR_V0_PSS_LEN];
  float sss_ref[NR_V0_PSS_LEN];
  const uint32_t nfft = nr_v0_ssb_nfft(fs_hz);
  const uint32_t cp = nr_v0_ssb_cp_len(fs_hz);

  const uint32_t nominal_sss_start = (uint32_t)hit->peak_samp + 2U * sym_len;
  const int delta_max = (int)(cp / 2U);
  const int delta_step = (int)((cp / 4U) ? (cp / 4U) : 1U);
  for (int delta = -delta_max; delta <= delta_max; delta += delta_step) {
    int64_t sss_i64 = (int64_t)nominal_sss_start + (int64_t)delta;
    if (sss_i64 < 0) {
      continue;
    }
    uint32_t sss_start = (uint32_t)sss_i64;
    if (nr_v0_extract_sss_bins(x, sss_start, blk->nsamps, nfft, cp,
                               cfo_hz, fs_hz, rx_i, rx_q) != 0) {
      continue;
    }

    for (int nid1 = 0; nid1 < NID1_MAX; nid1++) {
      double cr = 0.0;
      double ci = 0.0;
      double px = 0.0;
      double pp = 0.0;
      nr_v0_sss_build_fd(nid1, nid2, sss_ref, NR_V0_PSS_LEN);
      for (uint32_t m = 0; m < NR_V0_PSS_LEN; m++) {
        const double xr = (double)rx_i[m];
        const double xq = (double)rx_q[m];
        const double sr = (double)sss_ref[m];
        cr += xr * sr;
        ci += xq * sr;
        px += xr * xr + xq * xq;
        pp += sr * sr;
      }
      const float metric =
          (float)(sqrt(cr * cr + ci * ci) / (sqrt(px * pp) + 1e-9));
      metric_sum += metric;
      metric_sq += (double)metric * (double)metric;
      cnt++;
      if (metric > best_metric) {
        best_metric = metric;
        best_nid1 = nid1;
        best_sss_delta = delta;
      }
      nr_sss_insert_hyp(sss_hyps, MAX_SSS_HYPS,
                        (nr_sss_hyp_t){.nid1 = (uint16_t)nid1, .delta = (int16_t)delta, .metric = metric});
    }
  }
  if (cnt == 0) {
    sync->locked = 0;
    return -1;
  }

  double mean = metric_sum / (double)cnt;
  double var = metric_sq / (double)cnt - mean * mean;
  if (var < 0.0) var = 0.0;
  const float adaptive_th = (float)fmax(0.15, mean + 4.0 * sqrt(var));

  if (best_metric < adaptive_th) {
    sync->locked = 0;
    return -1;
  }

  /* Candidate-only output: final lock is decided in provider after
   * multi-stage consistency/PBCH confirmation gates. */
  sync->locked = 0;
  /* Fold part of SSS timing refinement back into coarse timing state. */
  sync->coarse_offset_samp = hit->peak_samp + (best_sss_delta / 2);
  sync->frac_offset_samp = 0.0f;
  sync->cfo_hz = hit->coarse_cfo_hz;
  sync->nid2 = nid2;
  sync->nid1 = (uint16_t)best_nid1;
  sync->pci = (uint16_t)(3U * (uint16_t)best_nid1 + (uint16_t)nid2);
  sync->pci_full = sync->pci;
  sync->ssb_index = 0;
  sync->sfn = 0;
  sync->slot = 0;
  sync->snr_db = 10.0f * log10f(best_metric + 1.0f);
  sync->pss_metric = hit->metric;
  sync->pbch_ok = 0;
  sync->pbch_confirmed = 0;
  sync->mib_ok = 0;
  sync->mib_payload = 0U;
  sync->pbch_metric = 0.0f;
  sync->pbch_metric_second = 0.0f;
  sync->pbch_fail_stage = NR_PBCH_FAIL_NONE;
  sync->lock_confidence = best_metric;
  sync->last_gscn = -1;
  sync->overflow_seen = 0;
  sync->pci_hyp_count = 0U;
  for (uint32_t i = 0U; i < MAX_SSS_HYPS; i++) {
    if (sss_hyps[i].metric <= 0.0f) {
      continue;
    }
    sync->pci_hyp[sync->pci_hyp_count] = (uint16_t)(3U * sss_hyps[i].nid1 + (uint16_t)nid2);
    sync->pci_hyp_delta[sync->pci_hyp_count] = sss_hyps[i].delta;
    sync->pci_hyp_metric[sync->pci_hyp_count] = sss_hyps[i].metric;
    sync->pci_hyp_count++;
  }
  if (sync->pci_hyp_count == 0U) {
    sync->pci_hyp[0] = sync->pci;
    sync->pci_hyp_delta[0] = (int16_t)best_sss_delta;
    sync->pci_hyp_metric[0] = best_metric;
    sync->pci_hyp_count = 1U;
  }
  return 0;
}

int nr_ssb_pbch_decode(const nr_iq_block_t *blk, nr_sync_state_t *sync)
{
  static const int32_t timing_deltas[] = {0, -1, 1, -2, 2, -3, 3};
  static const float cfo_deltas_hz[] = {0.0f, -500.0f, 500.0f, -1000.0f, 1000.0f,
                                        -2000.0f, 2000.0f, -3500.0f, 3500.0f};
  static const float phase_rot_deg[] = {0.0f, -15.0f, 15.0f, -30.0f, 30.0f};
  static const uint8_t k_pbch_ssb_candidates = 8U;
  enum { MAX_PBCH_HYPS = 16 };
  static uint32_t pbch_dbg_cnt = 0U;
  nr_ssb_grid_t grids[(sizeof(timing_deltas) / sizeof(timing_deltas[0])) *
                      (sizeof(cfo_deltas_hz) / sizeof(cfo_deltas_hz[0]))];
  nr_pbch_hyp_t hyps[MAX_PBCH_HYPS];
  const uint8_t pci_hyp_count = (sync->pci_hyp_count > 0U) ? sync->pci_hyp_count : 1U;
  uint32_t hyp_count = 0U;
  int32_t best_timing_delta = 0;
  int16_t best_sss_delta_bias = 0;
  float best_cfo_delta_hz = 0.0f;
  uint16_t best_pci = sync->pci;
  if (!blk || !sync) {
    return -1;
  }
  float best_metric = -1.0f;
  float second_metric = -1.0f;
  uint8_t best_ssb_idx = 0U;
  memset(grids, 0, sizeof(grids));
  memset(hyps, 0, sizeof(hyps));

  sync->pbch_ok = 0;
  sync->pbch_confirmed = 0;
  sync->mib_ok = 0;
  sync->mib_payload = 0U;
  sync->pbch_metric = 0.0f;
  sync->pbch_metric_second = 0.0f;
  sync->pbch_fail_stage = NR_PBCH_FAIL_NONE;

  for (uint32_t p = 0U; p < pci_hyp_count; p++) {
    const uint16_t pci = (sync->pci_hyp_count > 0U) ? sync->pci_hyp[p] : sync->pci;
    const int16_t sss_delta_bias =
        (sync->pci_hyp_count > 0U) ? (int16_t)((sync->pci_hyp_delta[p] - sync->pci_hyp_delta[0]) / 2) : 0;
    const uint8_t v = (uint8_t)(pci & 3U);
    for (uint32_t g = 0U; g < (sizeof(timing_deltas) / sizeof(timing_deltas[0])); g++) {
      for (uint32_t c = 0U; c < (sizeof(cfo_deltas_hz) / sizeof(cfo_deltas_hz[0])); c++) {
        nr_ssb_window_t win;
        const int32_t total_delta = timing_deltas[g] + (int32_t)sss_delta_bias;
        const uint32_t grid_idx = g * (uint32_t)(sizeof(cfo_deltas_hz) / sizeof(cfo_deltas_hz[0])) + c;
        if (nr_ssb_extract_window_shifted(blk, sync, total_delta, &win) != 0) {
          continue;
        }
        if (nr_ssb_demod(blk, &win, sync->cfo_hz + cfo_deltas_hz[c], &grids[grid_idx]) != 0 ||
            !grids[grid_idx].valid) {
          continue;
        }
        for (uint8_t ssb_idx = 0U; ssb_idx < k_pbch_ssb_candidates; ssb_idx++) {
          cf32_t h_est[NR_SSB_RE_ROWS][NR_SSB_RE_COLS];
          uint8_t h_valid[NR_SSB_RE_ROWS][NR_SSB_RE_COLS];
          const float metric =
              nr_pbch_dmrs_metric_estimate(&grids[grid_idx], pci, ssb_idx, v, h_est, h_valid);
          if (metric > best_metric) {
            second_metric = best_metric;
            best_metric = metric;
            best_pci = pci;
            best_ssb_idx = ssb_idx;
            best_sss_delta_bias = sss_delta_bias;
            best_timing_delta = timing_deltas[g];
            best_cfo_delta_hz = cfo_deltas_hz[c];
          } else if (metric > second_metric) {
            second_metric = metric;
          }
          if (metric >= 0.10f) {
            nr_pbch_hyp_t hyp;
            hyp.pci = pci;
            hyp.grid_idx = (uint8_t)grid_idx;
            hyp.sss_delta_bias = sss_delta_bias;
            hyp.timing_delta = timing_deltas[g];
            hyp.cfo_delta_hz = cfo_deltas_hz[c];
            hyp.ssb_idx = ssb_idx;
            hyp.metric = metric;
            nr_pbch_insert_hyp(hyps, MAX_PBCH_HYPS, &hyp_count, hyp);
          }
        }
      }
    }
  }

  sync->pbch_metric = best_metric;
  sync->pbch_metric_second = second_metric;
  sync->pci = best_pci;
  sync->pci_full = best_pci;
  sync->nid2 = (uint8_t)(best_pci % 3U);
  sync->nid1 = (uint16_t)(best_pci / 3U);
  sync->ssb_index = best_ssb_idx;
  if (best_metric < 0.14f) {
    sync->pbch_fail_stage = NR_PBCH_FAIL_DMRS_WEAK;
    if ((pbch_dbg_cnt++ % 25U) == 0U) {
      printf("pbch_dmrs: weak pci=%u ssb=%u best=%.3f second=%.3f ds=%d dt=%d pss=%.3f snr=%.2f\n",
             (unsigned)sync->pci, (unsigned)best_ssb_idx,
             best_metric, second_metric, best_sss_delta_bias, best_timing_delta,
             sync->pss_metric, sync->snr_db);
    }
    return -1;
  }
  for (uint32_t h = 0U; h < hyp_count; h++) {
    cf32_t h_est[NR_SSB_RE_ROWS][NR_SSB_RE_COLS];
    uint8_t h_valid[NR_SSB_RE_ROWS][NR_SSB_RE_COLS];
    float llr[864];
    float llr_base[864];
    float noise_var = 1.0f;
    float cpe_rad = 0.0f;
    const nr_pbch_hyp_t hyp = hyps[h];
    const uint8_t v = (uint8_t)(hyp.pci & 3U);
    if (nr_pbch_dmrs_metric_estimate(&grids[hyp.grid_idx], hyp.pci, hyp.ssb_idx, v,
                                     h_est, h_valid) < 0.0f) {
      continue;
    }
    noise_var = nr_pbch_estimate_noise_cpe(&grids[hyp.grid_idx], hyp.pci, hyp.ssb_idx,
                                           v, h_est, h_valid, &cpe_rad);
    if (nr_pbch_build_llr_from_grid(&grids[hyp.grid_idx], h_est, h_valid, v,
                                    noise_var, cpe_rad, llr_base) != 0) {
      continue;
    }
    sync->pci = hyp.pci;
    sync->pci_full = hyp.pci;
    sync->nid2 = (uint8_t)(hyp.pci % 3U);
    sync->nid1 = (uint16_t)(hyp.pci / 3U);
    sync->ssb_index = hyp.ssb_idx;
    for (uint32_t pr = 0U; pr < (sizeof(phase_rot_deg) / sizeof(phase_rot_deg[0])); pr++) {
      const float ph = phase_rot_deg[pr] * (float)M_PI / 180.0f;
      const float cph = cosf(ph);
      const float sph = sinf(ph);
      for (uint32_t variant = 0U; variant < 8U; variant++) {
      memcpy(llr, llr_base, sizeof(llr));
      for (uint32_t m = 0U; m < 432U; m++) {
        float xr = llr_base[2U * m] * 0.5f;
        float xi = llr_base[2U * m + 1U] * 0.5f;
        {
          const float tr = xr * cph - xi * sph;
          const float ti = xr * sph + xi * cph;
          xr = tr;
          xi = ti;
        }
        switch (variant) {
          case 0U:
            break;
          case 1U:
            xi = -xi;
            break;
          case 2U:
            xr = -xr;
            break;
          case 3U:
            xr = -xr;
            xi = -xi;
            break;
          case 4U: {
            const float t = xr;
            xr = xi;
            xi = t;
            break;
          }
          case 5U: {
            const float t = xr;
            xr = xi;
            xi = -t;
            break;
          }
          case 6U: {
            const float t = xr;
            xr = -xi;
            xi = t;
            break;
          }
          default: {
            const float t = xr;
            xr = -xi;
            xi = -t;
            break;
          }
        }
        llr[2U * m] = 2.0f * xr;
        llr[2U * m + 1U] = 2.0f * xi;
      }
        if (nr_pbch_bch_decode(llr, 864U, sync) == 0) {
          sync->pbch_ok = 1;
          sync->pbch_fail_stage = NR_PBCH_FAIL_NONE;
          sync->coarse_offset_samp += hyp.sss_delta_bias;
          sync->coarse_offset_samp += hyp.timing_delta;
          sync->cfo_hz += hyp.cfo_delta_hz;
          sync->pbch_metric = hyp.metric;
          sync->pbch_metric_second = second_metric;
          if (hyp.metric > sync->lock_confidence) {
            sync->lock_confidence = hyp.metric;
          }
          printf("pbch_bch: ok pci=%u ssb=%u dmrs=%.3f dt=%d dcfo=%.1f dph=%.1f pss=%.3f snr=%.2f sfn=%u mib=0x%06x\n",
                 (unsigned)sync->pci, (unsigned)sync->ssb_index,
                 hyp.metric, hyp.timing_delta, hyp.cfo_delta_hz, phase_rot_deg[pr],
                 sync->pss_metric, sync->snr_db,
                 (unsigned)sync->sfn, (unsigned)sync->mib_payload);
          return 0;
        }
      }
    }
  }

  if (second_metric > 0.0f && best_metric < 1.01f * second_metric) {
    sync->pbch_fail_stage = NR_PBCH_FAIL_DMRS_AMBIG;
    if ((pbch_dbg_cnt++ % 25U) == 0U) {
      printf("pbch_dmrs: ambiguous pci=%u ssb=%u best=%.3f second=%.3f ds=%d dt=%d dcfo=%.1f pss=%.3f snr=%.2f\n",
             (unsigned)sync->pci, (unsigned)best_ssb_idx,
             best_metric, second_metric, best_sss_delta_bias, best_timing_delta, best_cfo_delta_hz,
             sync->pss_metric, sync->snr_db);
    }
    return -1;
  }

  sync->pbch_fail_stage = NR_PBCH_FAIL_BCH;
  if ((pbch_dbg_cnt++ % 25U) == 0U) {
    printf("pbch_bch: crc-fail pci=%u ssb=%u dmrs=%.3f second=%.3f ds=%d dt=%d dcfo=%.1f pss=%.3f snr=%.2f\n",
           (unsigned)sync->pci, (unsigned)best_ssb_idx,
           best_metric, second_metric, best_sss_delta_bias, best_timing_delta, best_cfo_delta_hz,
           sync->pss_metric, sync->snr_db);
  }
  return -1;
}

int nr_ssb_track_cfo(const nr_iq_block_t *blk, nr_sync_state_t *sync)
{
  if (!blk || !sync) {
    return -1;
  }
  (void)blk;
  sync->cfo_hz *= 0.99f;
  return 0;
}

int nr_ssb_track_timing(const nr_iq_block_t *blk, nr_sync_state_t *sync,
                        int *sample_shift)
{
  if (!blk || !sync || !sample_shift) {
    return -1;
  }
  *sample_shift = 0;
  sync->cum_tracking_shift_samp += *sample_shift;
  sync->frac_offset_samp *= 0.95f;
  (void)blk;
  return 0;
}

int nr_ssb_check_lost_lock(const nr_sync_state_t *sync)
{
  if (!sync) {
    return 1;
  }
  if (!sync->locked) {
    return 1;
  }
  /* Lock keeper: keep only reasonably consistent SSB timing/CFO/quality.
   * PBCH/MIB validity is tracked separately and must not masquerade as lock. */
  if (sync->snr_db < 0.60f) {
    return 1;
  }
  if (fabsf(sync->cfo_hz) > 2.0e5f) {
    return 1;
  }
  return 0;
}
