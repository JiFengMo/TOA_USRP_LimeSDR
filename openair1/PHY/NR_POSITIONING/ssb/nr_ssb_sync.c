#include "openair1/PHY/NR_POSITIONING/nr_pos_types.h"
#include "openair1/PHY/NR_POSITIONING/nr_pos_api.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

#define NR_V0_MIN_PWR 100.0
#define NR_V0_FS_HZ_FALLBACK 30720000.0

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
  double sr = 0.0;
  double si = 0.0;
  for (uint32_t n = 1; n < len; n++) {
    double ar = (double)x[pss_start + n].r;
    double ai = (double)x[pss_start + n].i;
    double br = (double)x[pss_start + n - 1U].r;
    double bi = (double)x[pss_start + n - 1U].i;
    sr += ar * br + ai * bi;
    si += ai * br - ar * bi;
  }
  double ph = atan2(si, sr);
  return (float)(ph * fs_hz / (2.0 * M_PI));
}

int nr_ssb_pss_search(const nr_iq_block_t *blk, nr_pss_hit_t *hits, int max_hits)
{
  if (!blk || !hits || max_hits <= 0) {
    return -1;
  }
  const uint32_t ref_len = nr_v0_pss_td_len();
  memset(hits, 0, sizeof(nr_pss_hit_t) * (size_t)max_hits);
  if (!blk->rx[0] || blk->nsamps < ref_len + 1U) {
    return -1;
  }

  const c16_t *x = blk->rx[0];
  const float fs_hz = (blk->fs_hz > 0.0) ? (float)blk->fs_hz : NR_V0_FS_HZ_FALLBACK;
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

  float pss_i[3][512];
  float pss_q[3][512];
  if (ref_len > 512U) {
    return -1;
  }
  for (int nid2 = 0; nid2 < 3; nid2++) {
    nr_v0_pss_build_td_f(nid2, pss_i[nid2], pss_q[nid2], ref_len);
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
    return -1;
  }
  double mean = (metric_cnt > 0) ? (metric_sum / (double)metric_cnt) : 0.0;
  double var = (metric_cnt > 1)
                   ? (metric_sq / (double)metric_cnt - mean * mean)
                   : 0.0;
  if (var < 0.0) {
    var = 0.0;
  }
  const float adaptive_th = (float)fmax(0.10, mean + 4.0 * sqrt(var));
  if (hits[0].metric < adaptive_th) {
    return -1;
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
  hits[0].nid2 = best_nid2;
  hits[0].metric = best_m;
  hits[0].coarse_cfo_hz = cfo;
  return 0;
}

int nr_ssb_refine_sync(const nr_iq_block_t *blk, const nr_pss_hit_t *hit,
                       nr_sync_state_t *sync)
{
  if (!blk || !hit || !sync) {
    return -1;
  }
  const float fs_hz = (blk->fs_hz > 0.0) ? (float)blk->fs_hz : NR_V0_FS_HZ_FALLBACK;
  memset(sync, 0, sizeof(*sync));

  const uint32_t ref_len = nr_v0_pss_td_len();
  const uint32_t sym_len = nr_v0_ssb_sym_len();
  if (hit->metric < 0.12f) {
    sync->locked = 0;
    return -1;
  }

  const c16_t *x = blk->rx[0];
  const float cfo_hz = hit->coarse_cfo_hz;
  const uint8_t nid2 = hit->nid2;

  /* Lazy cache: SSS TD refs for given nid2. */
  enum { NID1_MAX = 336, SYM_REF_MAX = 512 };
  static uint8_t sss_cache_valid[3] = {0, 0, 0};
  static float sss_i_cache[3][NID1_MAX][SYM_REF_MAX];
  static float sss_q_cache[3][NID1_MAX][SYM_REF_MAX];

  if (!sss_cache_valid[nid2]) {
    for (int nid1 = 0; nid1 < NID1_MAX; nid1++) {
      nr_v0_sss_build_td_f(nid1, nid2, sss_i_cache[nid2][nid1], sss_q_cache[nid2][nid1], ref_len);
    }
    sss_cache_valid[nid2] = 1;
  }

  int best_nid1 = 0;
  float best_metric = -1.0f;
  int best_sss_delta = 0;
  double metric_sum = 0.0;
  double metric_sq = 0.0;
  uint64_t cnt = 0;

  const uint32_t nominal_sss_start = (uint32_t)hit->peak_samp + 2U * sym_len;
  const int delta_max = (int)(sym_len / 4U); /* robust to CP/timing bias */
  const int delta_step = 4;                  /* keep compute bounded */
  for (int delta = -delta_max; delta <= delta_max; delta += delta_step) {
    int64_t sss_i64 = (int64_t)nominal_sss_start + (int64_t)delta;
    if (sss_i64 < 0) {
      continue;
    }
    uint32_t sss_start = (uint32_t)sss_i64;
    if (sss_start + ref_len >= blk->nsamps) {
      continue;
    }

    for (int nid1 = 0; nid1 < NID1_MAX; nid1++) {
      float metric = nr_v0_corr_metric_comp(
          x, sss_start,
          sss_i_cache[nid2][nid1], sss_q_cache[nid2][nid1],
          ref_len, cfo_hz, fs_hz);
      metric_sum += metric;
      metric_sq += (double)metric * (double)metric;
      cnt++;
      if (metric > best_metric) {
        best_metric = metric;
        best_nid1 = nid1;
        best_sss_delta = delta;
      }
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

  sync->locked = 1;
  /* Fold part of SSS timing refinement back into coarse timing state. */
  sync->coarse_offset_samp = hit->peak_samp + (best_sss_delta / 2);
  sync->frac_offset_samp = 0.0f;
  sync->cfo_hz = hit->coarse_cfo_hz;
  sync->pci = (uint16_t)(3U * (uint16_t)best_nid1 + (uint16_t)nid2);
  sync->ssb_index = 0;
  sync->sfn = 0;
  sync->slot = 0;
  sync->snr_db = 10.0f * log10f(best_metric + 1.0f);
  sync->pss_metric = hit->metric;
  return 0;
}

int nr_ssb_pbch_decode(const nr_iq_block_t *blk, nr_sync_state_t *sync)
{
  if (!blk || !sync) {
    return -1;
  }
  (void)blk;
  sync->sfn = (sync->sfn + 1U) & 1023U;
  return 0;
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
  /* Lock keeper: keep only reasonably consistent CFO/quality.
   * (Use snr_db proxy derived from SSS best_metric in refine_sync.) */
  if (sync->pbch_confirmed == 0U) {
    return 1;
  }
  if (sync->snr_db < 0.20f) {
    return 1;
  }
  if (fabsf(sync->cfo_hz) > 2.0e5f) {
    return 1;
  }
  return 0;
}
