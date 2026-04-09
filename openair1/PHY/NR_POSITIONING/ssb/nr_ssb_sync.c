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
  /* CP-based estimate: angle(sum CP[n]*conj(tail[n])) = -2*pi*f_cfo*N/fs,
   * so the raw ph gives -f_cfo.  Return +f_cfo so that the OFDM demod
   * correction  e^{-j*2*pi*cfo_hz*t}  actually removes the offset.        */
  return (float)(-ph * fs_hz / (2.0 * M_PI * (double)nfft));
}

static int nr_sss_extract_fd_from_grid(const nr_ssb_grid_t *grid,
                                       uint8_t nid2,
                                       float *rx_i,
                                       float *rx_q)
{
  float pss_ref[NR_V0_PSS_LEN];
  if (!grid || !grid->valid || !rx_i || !rx_q) {
    return -1;
  }
  nr_v0_pss_build_fd((int)nid2, pss_ref, NR_V0_PSS_LEN);
  for (uint32_t m = 0U; m < NR_V0_PSS_LEN; m++) {
    const uint16_t rel = (uint16_t)(56U + m);
    const cf32_t pss = grid->re[0][rel];
    const cf32_t sss = grid->re[2][rel];
    const float ref = pss_ref[m];
    /* Match OAI's idea: derive H* from PSS then apply it to SSS before
     * searching Nid1, which removes most channel/CPE distortion. */
    const float hconj_r = pss.r * ref;
    const float hconj_i = -pss.i * ref;
    rx_i[m] = sss.r * hconj_r - sss.i * hconj_i;
    rx_q[m] = sss.r * hconj_i + sss.i * hconj_r;
  }
  return 0;
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

typedef struct {
  uint16_t pci;
  uint8_t grid_idx;
  int16_t sss_delta_bias;
  int32_t timing_delta;
  float cfo_delta_hz;
  uint8_t ssb_idx;
  float metric;
} nr_pbch_hyp_t;

static void nr_pbch_sync_apply_hyp(nr_sync_state_t *sync,
                                   const nr_pbch_hyp_t *hyp,
                                   float second_metric)
{
  if (!sync || !hyp) {
    return;
  }
  sync->pci = hyp->pci;
  sync->pci_full = hyp->pci;
  sync->nid2 = (uint8_t)(hyp->pci % 3U);
  sync->nid1 = (uint16_t)(hyp->pci / 3U);
  sync->ssb_index = hyp->ssb_idx;
  sync->pbch_metric = hyp->metric;
  sync->pbch_metric_second = second_metric;
  sync->pbch_best_sss_delta_bias = hyp->sss_delta_bias;
  sync->pbch_best_timing_delta = (int16_t)hyp->timing_delta;
  sync->pbch_best_cfo_delta_hz = hyp->cfo_delta_hz;
}

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

static void nr_pss_insert_hit(nr_pss_hit_t *hits,
                              int max_hits,
                              uint32_t merge_dist,
                              const nr_pss_hit_t *cand,
                              int *count)
{
  int n = 0;
  int pos = 0;
  if (!hits || !cand || !count || max_hits <= 0) {
    return;
  }

  n = (*count < max_hits) ? *count : max_hits;
  for (int i = 0; i < n; i++) {
    const int32_t d = hits[i].peak_samp - cand->peak_samp;
    if ((uint32_t)((d < 0) ? -d : d) <= merge_dist) {
      if (cand->metric > hits[i].metric) {
        hits[i] = *cand;
      }
      while (i > 0 && hits[i - 1].metric < hits[i].metric) {
        nr_pss_hit_t tmp = hits[i - 1];
        hits[i - 1] = hits[i];
        hits[i] = tmp;
        i--;
      }
      while (i + 1 < n && hits[i + 1].metric > hits[i].metric) {
        nr_pss_hit_t tmp = hits[i + 1];
        hits[i + 1] = hits[i];
        hits[i] = tmp;
        i++;
      }
      return;
    }
  }

  pos = (n < max_hits) ? n : (max_hits - 1);
  while (pos > 0 && hits[pos - 1].metric < cand->metric) {
    if (pos < max_hits) {
      hits[pos] = hits[pos - 1];
    }
    pos--;
  }
  if (pos < max_hits) {
    hits[pos] = *cand;
    if (n < max_hits) {
      (*count)++;
    }
  }
}

int nr_ssb_pss_search(const nr_iq_block_t *blk, nr_pss_hit_t *hits, int max_hits)
{
  if (!blk || !hits || max_hits <= 0) {
    return -1;
  }
  const float fs_hz = (blk->fs_hz > 0.0) ? (float)blk->fs_hz : NR_V0_FS_HZ_FALLBACK;
  const uint32_t ref_len = nr_v0_ssb_sym_len_fs(fs_hz);
  const uint32_t merge_dist = nr_v0_ssb_cp_len(fs_hz);
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
      nr_pss_hit_t cand = {
          .peak_samp = (int32_t)n,
          .coarse_cfo_hz = 0.0f,
          .metric = metric,
          .nid2 = (uint8_t)nid2};
      nr_pss_insert_hit(hits, max_hits, merge_dist, &cand, &found);
    }
  }
  if (found <= 0) {
    for (int nid2 = 0; nid2 < 3; nid2++) {
      free(pss_i[nid2]);
      free(pss_q[nid2]);
    }
    return -1;
  }
  if (hits[0].metric < 0.12f) {
    for (int nid2 = 0; nid2 < 3; nid2++) {
      free(pss_i[nid2]);
      free(pss_q[nid2]);
    }
    return -1;
  }

  /* Estimate coarse CFO and re-score EVERY hit with compensation. */
  for (int h = 0; h < found && h < max_hits; h++) {
    if (hits[h].metric < 0.12f) {
      continue;
    }
    uint32_t pss_h = (uint32_t)hits[h].peak_samp;
    float cfo_h = nr_v0_estimate_cfo_hz(x, pss_h, ref_len, fs_hz);
    if (cfo_h > 15000.0f) cfo_h = 15000.0f;
    if (cfo_h < -15000.0f) cfo_h = -15000.0f;
    float best_m = -1.0f;
    uint8_t best_nid2 = hits[h].nid2;
    for (int nid2 = 0; nid2 < 3; nid2++) {
      float m = nr_v0_corr_metric_comp(x, pss_h, pss_i[nid2], pss_q[nid2],
                                       ref_len, cfo_h, fs_hz);
      if (m > best_m) {
        best_m = m;
        best_nid2 = (uint8_t)nid2;
      }
    }
    hits[h].nid2 = best_nid2;
    hits[h].metric = best_m;
    hits[h].coarse_cfo_hz = cfo_h;
  }
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
  enum { MAX_SSS_HYPS = NR_MAX_PCI_HYPS };
  nr_sss_hyp_t sss_hyps[MAX_SSS_HYPS];
  int delta_list[5];
  uint32_t delta_count = 0U;
  if (!blk || !hit || !sync) {
    return -1;
  }
  const float fs_hz = (blk->fs_hz > 0.0) ? (float)blk->fs_hz : NR_V0_FS_HZ_FALLBACK;
  memset(sync, 0, sizeof(*sync));
  memset(sss_hyps, 0, sizeof(sss_hyps));

  if (hit->metric < 0.12f) {
    sync->locked = 0;
    return -1;
  }

  const float cfo_hz = hit->coarse_cfo_hz;
  const uint8_t nid2 = hit->nid2;

  enum { NID1_MAX = 336 };
  int best_nid1 = 0;
  float best_metric = -1.0f;
  float second_metric = -1.0f;
  int best_sss_delta = 0;
  double metric_sum = 0.0;
  double metric_sq = 0.0;
  uint64_t cnt = 0;
  float rx_i[NR_V0_PSS_LEN];
  float rx_q[NR_V0_PSS_LEN];
  float sss_ref[NR_V0_PSS_LEN];
  const uint32_t cp = nr_v0_ssb_cp_len(fs_hz);

  const int delta_step = (int)((cp / 4U) ? (cp / 4U) : 1U);
  delta_list[delta_count++] = 0;
  delta_list[delta_count++] = -delta_step;
  delta_list[delta_count++] = delta_step;
  delta_list[delta_count++] = -2 * delta_step;
  delta_list[delta_count++] = 2 * delta_step;
  sync->coarse_offset_samp = hit->peak_samp;
  sync->cum_tracking_shift_samp = 0;
  for (uint32_t di = 0U; di < delta_count; di++) {
    const int delta = delta_list[di];
    nr_ssb_window_t fd_win;
    nr_ssb_grid_t fd_grid;
    if (nr_ssb_extract_window_shifted(blk, sync, delta, &fd_win) == 0 &&
        nr_ssb_demod(blk, &fd_win, cfo_hz, &fd_grid) == 0 &&
        fd_grid.valid &&
        nr_sss_extract_fd_from_grid(&fd_grid, nid2, rx_i, rx_q) == 0) {
      for (int nid1 = 0; nid1 < NID1_MAX; nid1++) {
        float metric = -1.0f;
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
        metric = (float)(sqrt(cr * cr + ci * ci) / (sqrt(px * pp) + 1e-9));
        metric_sum += metric;
        metric_sq += (double)metric * (double)metric;
        cnt++;
        if (metric > best_metric) {
          second_metric = best_metric;
          best_metric = metric;
          best_nid1 = nid1;
          best_sss_delta = delta;
        } else if (metric > second_metric) {
          second_metric = metric;
        }
        nr_sss_insert_hyp(sss_hyps, MAX_SSS_HYPS,
                          (nr_sss_hyp_t){.nid1 = (uint16_t)nid1, .delta = (int16_t)delta, .metric = metric});
      }
    }
  }
  if (cnt == 0) {
    sync->locked = 0;
    return -1;
  }

  if (best_metric < 0.12f) {
    sync->locked = 0;
    return -1;
  }

  /* Candidate-only output: final lock is decided in provider after
   * multi-stage consistency/PBCH confirmation gates. */
  sync->locked = 0;
  /* Fold part of SSS timing refinement back into coarse timing state. */
  sync->coarse_offset_samp = hit->peak_samp + best_sss_delta;
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

static void nr_dbg_grid_quality(const nr_ssb_grid_t *grid, uint8_t nid2,
                                uint16_t pci, float cfo_hz)
{
  float pss_ref[127];
  float sss_ref[127];
  double cr = 0.0, ci = 0.0, px = 0.0, pp = 0.0;
  double sym_pow[4] = {0};
  if (!grid || !grid->valid) {
    return;
  }
  nr_v0_pss_build_fd((int)nid2, pss_ref, 127U);
  for (uint32_t m = 0; m < 127U; m++) {
    const float yr = grid->re[0][56U + m].r;
    const float yi = grid->re[0][56U + m].i;
    cr += (double)yr * (double)pss_ref[m];
    ci += (double)yi * (double)pss_ref[m];
    px += (double)yr * yr + (double)yi * yi;
    pp += (double)pss_ref[m] * (double)pss_ref[m];
  }
  float pss_corr = (float)(sqrt(cr * cr + ci * ci) / (sqrt(px * pp) + 1e-9));
  float pss_phase = (float)(atan2(ci, cr) * 180.0 / M_PI);

  double scr = 0.0, sci = 0.0, spx = 0.0, spp = 0.0;
  uint16_t nid1 = pci / 3;
  nr_v0_sss_build_fd((int)nid1, (int)nid2, sss_ref, 127U);
  for (uint32_t m = 0; m < 127U; m++) {
    const float yr = grid->re[2][56U + m].r;
    const float yi = grid->re[2][56U + m].i;
    scr += (double)yr * (double)sss_ref[m];
    sci += (double)yi * (double)sss_ref[m];
    spx += (double)yr * yr + (double)yi * yi;
    spp += (double)sss_ref[m] * (double)sss_ref[m];
  }
  float sss_corr = (float)(sqrt(scr * scr + sci * sci) / (sqrt(spx * spp) + 1e-9));

  for (uint8_t sym = 0; sym < 4; sym++) {
    for (uint16_t rel = 0; rel < 240U; rel++) {
      sym_pow[sym] += (double)grid->re[sym][rel].r * grid->re[sym][rel].r +
                      (double)grid->re[sym][rel].i * grid->re[sym][rel].i;
    }
    sym_pow[sym] /= 240.0;
  }

  printf("GRID_DBG: pci=%u cfo=%.1f pss_corr=%.4f sss_corr=%.4f pss_ph=%.1f "
         "sym_pow=[%.1f,%.1f,%.1f,%.1f]\n",
         (unsigned)pci, cfo_hz, pss_corr, sss_corr, pss_phase,
         sym_pow[0], sym_pow[1], sym_pow[2], sym_pow[3]);
}

int nr_ssb_pbch_decode(const nr_iq_block_t *blk, nr_sync_state_t *sync)
{
  static const int32_t timing_deltas[] = {0, -1, 1, -2, 2, -3, 3};
  static const float cfo_deltas_hz[] = {0.0f, -500.0f, 500.0f};
  static const uint8_t k_pbch_ssb_candidates = 8U;
  enum { MAX_PBCH_HYPS = 8 };
  static uint32_t pbch_dbg_cnt = 0U;
  nr_ssb_grid_t grids[(sizeof(timing_deltas) / sizeof(timing_deltas[0])) *
                      (sizeof(cfo_deltas_hz) / sizeof(cfo_deltas_hz[0]))];
  nr_pbch_hyp_t hyps[MAX_PBCH_HYPS];
  nr_pbch_hyp_t best_hyp;
  nr_pbch_hyp_t best_llr_hyp;
  uint8_t pci_hyp_count = (sync->pci_hyp_count > 0U) ? sync->pci_hyp_count : 1U;
  uint32_t timing_count = sizeof(timing_deltas) / sizeof(timing_deltas[0]);
  uint32_t cfo_count = sizeof(cfo_deltas_hz) / sizeof(cfo_deltas_hz[0]);
  uint8_t ssb_candidate_count = k_pbch_ssb_candidates;
  uint32_t decode_hyp_count = 0U;
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
  float best_llr_metric = -1.0f;
  uint8_t best_ssb_idx = 0U;
  float best_noise_var = 1.0f;
  float best_cpe_rad = 0.0f;
  int best_llr_valid = 0;
  memset(grids, 0, sizeof(grids));
  memset(hyps, 0, sizeof(hyps));
  memset(&best_hyp, 0, sizeof(best_hyp));
  memset(&best_llr_hyp, 0, sizeof(best_llr_hyp));

  if (pci_hyp_count > MAX_PBCH_HYPS) {
    pci_hyp_count = MAX_PBCH_HYPS;
  }
  ssb_candidate_count = k_pbch_ssb_candidates;

  sync->pbch_ok = 0;
  sync->pbch_confirmed = 0;
  sync->mib_ok = 0;
  sync->mib_payload = 0U;
  sync->pbch_metric = 0.0f;
  sync->pbch_metric_second = 0.0f;
  sync->pbch_fail_stage = NR_PBCH_FAIL_NONE;
  sync->pbch_llr_valid = 0U;
  sync->pbch_best_sss_delta_bias = 0;
  sync->pbch_best_timing_delta = 0;
  sync->pbch_best_cfo_delta_hz = 0.0f;
  sync->pbch_best_phase_deg = 0.0f;
  sync->pbch_noise_var = 1.0f;
  sync->pbch_cpe_rad = 0.0f;
  memset(sync->pbch_llr, 0, sizeof(sync->pbch_llr));

  for (uint32_t p = 0U; p < pci_hyp_count; p++) {
    const uint16_t pci = (sync->pci_hyp_count > 0U) ? sync->pci_hyp[p] : sync->pci;
    const int16_t sss_delta_bias =
        (sync->pci_hyp_count > 0U) ? (int16_t)(sync->pci_hyp_delta[p] - sync->pci_hyp_delta[0]) : 0;
    for (uint32_t g = 0U; g < timing_count; g++) {
      for (uint32_t c = 0U; c < cfo_count; c++) {
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
        if (p == 0U && g == 0U && c == 0U) {
          nr_dbg_grid_quality(&grids[grid_idx], sync->nid2, pci, sync->cfo_hz);
        }
        float best_ssb_metric_for_pci = -1.0f;
        uint8_t best_ssb_for_pci = 0;
        for (uint8_t ssb_idx = 0U; ssb_idx < ssb_candidate_count; ssb_idx++) {
          float metric = -1.0f;
          if (nr_ssb_pbch_prepare_frontend(&grids[grid_idx], pci, ssb_idx,
                                           &metric, NULL, NULL, NULL) != 0) {
            continue;
          }
          if (metric > best_ssb_metric_for_pci) {
            best_ssb_metric_for_pci = metric;
            best_ssb_for_pci = ssb_idx;
          }
          if (metric > best_metric) {
            second_metric = best_metric;
            best_metric = metric;
            best_pci = pci;
            best_ssb_idx = ssb_idx;
            best_sss_delta_bias = sss_delta_bias;
            best_timing_delta = timing_deltas[g];
            best_cfo_delta_hz = cfo_deltas_hz[c];
            best_hyp = (nr_pbch_hyp_t){
                .pci = pci,
                .grid_idx = (uint8_t)grid_idx,
                .sss_delta_bias = sss_delta_bias,
                .timing_delta = timing_deltas[g],
                .cfo_delta_hz = cfo_deltas_hz[c],
                .ssb_idx = ssb_idx,
                .metric = metric};
          } else if (metric > second_metric) {
            second_metric = metric;
          }
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
        if (p == 0U) {
          printf("  pci0_scan: pci=%u dt=%d dcfo=%.0f best_ssb=%u dmrs=%.4f\n",
                 (unsigned)pci, timing_deltas[g], cfo_deltas_hz[c],
                 (unsigned)best_ssb_for_pci, best_ssb_metric_for_pci);
        }
      }
    }
  }

  if (best_metric >= 0.0f) {
    nr_pbch_sync_apply_hyp(sync, &best_hyp, second_metric);
  } else {
    sync->pbch_metric = best_metric;
    sync->pbch_metric_second = second_metric;
    sync->pci = best_pci;
    sync->pci_full = best_pci;
    sync->nid2 = (uint8_t)(best_pci % 3U);
    sync->nid1 = (uint16_t)(best_pci / 3U);
    sync->ssb_index = best_ssb_idx;
  }
  decode_hyp_count = hyp_count;

  printf("pbch_hyps: count=%u decode=%u best_pci=%u best_ssb=%u best_dmrs=%.3f cfo=%.1f\n",
         hyp_count, decode_hyp_count,
         (unsigned)best_pci, (unsigned)best_ssb_idx,
         best_metric, sync->cfo_hz);
  for (uint32_t d = 0U; d < hyp_count && d < MAX_PBCH_HYPS; d++) {
    printf("  hyp[%u]: pci=%u ssb=%u dmrs=%.3f ds=%d dt=%d dcfo=%.1f\n",
           d, (unsigned)hyps[d].pci, (unsigned)hyps[d].ssb_idx,
           hyps[d].metric, hyps[d].sss_delta_bias,
           hyps[d].timing_delta, hyps[d].cfo_delta_hz);
  }

  for (uint32_t h = 0U; h < decode_hyp_count; h++) {
    float llr_base[864];
    float noise_var = 1.0f;
    float cpe_rad = 0.0f;
    float dmrs_metric = -1.0f;
    const nr_pbch_hyp_t hyp = hyps[h];
    float hyp_metric = hyp.metric;
    nr_ssb_grid_t decode_grid;
    {
      nr_ssb_window_t dwin;
      const int32_t dtotal = hyp.timing_delta + (int32_t)hyp.sss_delta_bias;
      if (nr_ssb_extract_window_shifted(blk, sync, dtotal, &dwin) != 0 ||
          nr_ssb_demod(blk, &dwin, sync->cfo_hz + hyp.cfo_delta_hz, &decode_grid) != 0 ||
          !decode_grid.valid) {
        continue;
      }
    }
    if (nr_ssb_pbch_prepare_frontend(&decode_grid, hyp.pci, hyp.ssb_idx,
                                     &dmrs_metric, &noise_var, &cpe_rad, llr_base) != 0) {
      continue;
    }
    if (dmrs_metric > hyp_metric) {
      hyp_metric = dmrs_metric;
    }
    if (!best_llr_valid || hyp_metric > best_llr_metric) {
      best_llr_metric = hyp_metric;
      best_llr_hyp = hyp;
      best_llr_hyp.metric = hyp_metric;
      best_noise_var = noise_var;
      best_cpe_rad = cpe_rad;
      best_llr_valid = 1;
      memcpy(sync->pbch_llr, llr_base, sizeof(llr_base));
      sync->pbch_llr_valid = 1U;
      sync->pbch_noise_var = noise_var;
      sync->pbch_cpe_rad = cpe_rad;
    }
    printf("  pbch_try[%u]: pci=%u ssb=%u dmrs=%.3f noise=%.4f cpe=%.3f dt=%d dcfo=%.1f\n",
           h, (unsigned)hyp.pci, (unsigned)hyp.ssb_idx,
           dmrs_metric, noise_var, cpe_rad, hyp.timing_delta, hyp.cfo_delta_hz);
    {
      nr_sync_state_t trial = *sync;
      nr_pbch_sync_apply_hyp(&trial, &hyp, second_metric);
      if (nr_pbch_bch_decode(llr_base, 864U, &trial) == 0) {
        *sync = trial;
        sync->pbch_ok = 1;
        sync->pbch_fail_stage = NR_PBCH_FAIL_NONE;
        sync->coarse_offset_samp += hyp.sss_delta_bias;
        sync->coarse_offset_samp += hyp.timing_delta;
        sync->cfo_hz += hyp.cfo_delta_hz;
        sync->pbch_llr_valid = 1U;
        sync->pbch_noise_var = noise_var;
        sync->pbch_cpe_rad = cpe_rad;
        memcpy(sync->pbch_llr, llr_base, sizeof(llr_base));
        if (hyp.metric > sync->lock_confidence) {
          sync->lock_confidence = hyp.metric;
        }
        printf("pbch_bch: ok pci=%u ssb=%u dmrs=%.3f dt=%d dcfo=%.1f pss=%.3f snr=%.2f sfn=%u mib=0x%06x\n",
               (unsigned)sync->pci, (unsigned)sync->ssb_index,
               hyp.metric, hyp.timing_delta, hyp.cfo_delta_hz,
               sync->pss_metric, sync->snr_db,
               (unsigned)sync->sfn, (unsigned)sync->mib_payload);
        return 0;
      }
    }
  }

  if (best_llr_valid) {
    nr_pbch_sync_apply_hyp(sync, &best_llr_hyp, second_metric);
    sync->coarse_offset_samp += best_llr_hyp.sss_delta_bias;
    sync->coarse_offset_samp += best_llr_hyp.timing_delta;
    sync->cfo_hz += best_llr_hyp.cfo_delta_hz;
    sync->pbch_llr_valid = 1U;
    sync->pbch_noise_var = best_noise_var;
    sync->pbch_cpe_rad = best_cpe_rad;
  } else if (best_metric >= 0.0f) {
    nr_pbch_sync_apply_hyp(sync, &best_hyp, second_metric);
    sync->coarse_offset_samp += best_hyp.sss_delta_bias;
    sync->coarse_offset_samp += best_hyp.timing_delta;
    sync->cfo_hz += best_hyp.cfo_delta_hz;
  }

  if (decode_hyp_count == 0U) {
    sync->pbch_fail_stage = NR_PBCH_FAIL_DEMOD;
  } else if (best_metric < 0.25f) {
    sync->pbch_fail_stage = NR_PBCH_FAIL_DMRS_WEAK;
  } else if (second_metric > 0.0f && best_metric < (second_metric * 1.03f)) {
    sync->pbch_fail_stage = NR_PBCH_FAIL_DMRS_AMBIG;
  } else {
    sync->pbch_fail_stage = NR_PBCH_FAIL_BCH;
  }
  pbch_dbg_cnt++;
  printf("pbch_bch: crc-fail pci=%u ssb=%u dmrs=%.3f second=%.3f ds=%d dt=%d dcfo=%.1f pss=%.3f snr=%.2f\n",
         (unsigned)sync->pci, (unsigned)best_ssb_idx,
         best_metric, second_metric, best_sss_delta_bias, best_timing_delta, best_cfo_delta_hz,
         sync->pss_metric, sync->snr_db);
  return -1;
}

int nr_ssb_track_cfo(const nr_iq_block_t *blk, nr_sync_state_t *sync)
{
  nr_ssb_window_t win;
  nr_ssb_grid_t grid;
  const float fs_hz = (blk && blk->fs_hz > 0.0) ? (float)blk->fs_hz : NR_V0_FS_HZ_FALLBACK;
  const uint32_t ref_len = nr_v0_ssb_sym_len_fs(fs_hz);
  float cp_cfo = 0.0f;
  if (!blk || !sync) {
    return -1;
  }
  if (!blk->rx[0]) {
    return -1;
  }
  if (sync->coarse_offset_samp >= 0 &&
      (uint32_t)sync->coarse_offset_samp + ref_len < blk->nsamps) {
    cp_cfo = nr_v0_estimate_cfo_hz(blk->rx[0],
                                   (uint32_t)sync->coarse_offset_samp,
                                   ref_len,
                                   fs_hz);
    if (cp_cfo > 15000.0f) cp_cfo = 15000.0f;
    if (cp_cfo < -15000.0f) cp_cfo = -15000.0f;
  }
  sync->cfo_hz = 0.82f * sync->cfo_hz + 0.18f * cp_cfo;

  if (sync->pci >= 1008U || nr_ssb_extract_window(blk, sync, &win) != 0 ||
      nr_ssb_demod(blk, &win, sync->cfo_hz, &grid) != 0 || !grid.valid) {
    return 0;
  }
  {
    float metric = -1.0f;
    float noise_var = 1.0f;
    float cpe_rad = 0.0f;
    if (nr_ssb_pbch_prepare_frontend(&grid, sync->pci, sync->ssb_index,
                                     &metric, &noise_var, &cpe_rad, NULL) != 0) {
      return 0;
    }
    if (metric > 0.0f) {
      const float t_ref = (float)nr_v0_ssb_burst_len_fs(fs_hz) / fs_hz;
      const float resid_cfo =
          (fabsf(t_ref) > 1.0e-6f) ? (cpe_rad / (2.0f * (float)M_PI * t_ref)) : 0.0f;
      const float gain = 0.20f;
      sync->cfo_hz += gain * resid_cfo;
      sync->pbch_cpe_rad = cpe_rad;
      sync->pbch_noise_var = noise_var;
    }
  }
  return 0;
}

int nr_ssb_track_timing(const nr_iq_block_t *blk, nr_sync_state_t *sync,
                        int *sample_shift)
{
  static const int deltas[] = {0, -1, 1, -2, 2};
  float metrics[sizeof(deltas) / sizeof(deltas[0])];
  int best_idx = 0;
  nr_ssb_grid_t grid;
  if (!blk || !sync || !sample_shift) {
    return -1;
  }
  memset(metrics, 0, sizeof(metrics));
  *sample_shift = 0;
  if (sync->pci < 1008U) {
    for (uint32_t i = 0U; i < (sizeof(deltas) / sizeof(deltas[0])); i++) {
      nr_ssb_window_t win;
      if (nr_ssb_extract_window_shifted(blk, sync, deltas[i], &win) != 0) {
        continue;
      }
      if (nr_ssb_demod(blk, &win, sync->cfo_hz, &grid) != 0 || !grid.valid) {
        continue;
      }
      if (nr_ssb_pbch_prepare_frontend(&grid, sync->pci, sync->ssb_index,
                                       &metrics[i], NULL, NULL, NULL) != 0) {
        metrics[i] = 0.0f;
      }
      if (metrics[i] > metrics[best_idx]) {
        best_idx = (int)i;
      }
    }
    *sample_shift = deltas[best_idx];
    if (best_idx > 0 &&
        best_idx + 1 < (int)(sizeof(deltas) / sizeof(deltas[0])) &&
        metrics[best_idx] > 0.0f) {
      const float lm = metrics[best_idx - 1];
      const float cm = metrics[best_idx];
      const float rm = metrics[best_idx + 1];
      const float den = lm - 2.0f * cm + rm;
      if (fabsf(den) > 1.0e-6f) {
        const float frac = 0.5f * (lm - rm) / den;
        sync->frac_offset_samp = 0.70f * sync->frac_offset_samp + 0.30f * frac;
      } else {
        sync->frac_offset_samp *= 0.85f;
      }
    } else {
      sync->frac_offset_samp *= 0.85f;
    }
  }
  sync->cum_tracking_shift_samp += *sample_shift;
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
