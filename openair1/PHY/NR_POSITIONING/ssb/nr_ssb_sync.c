#include "openair1/PHY/NR_POSITIONING/nr_pos_types.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

#define NR_V0_PSS_LEN 127
#define NR_V0_NFFT 256
#define NR_V0_CP 20
#define NR_V0_PSS_TD_LEN (NR_V0_NFFT + NR_V0_CP)
#define NR_V0_MIN_PWR 5000.0

static void nr_v0_build_pss_seq(int nid2, float *seq)
{
  uint8_t x[NR_V0_PSS_LEN];
  memset(x, 0, sizeof(x));

  /* m-sequence init for PSS */
  x[0] = 1;
  x[1] = 0;
  x[2] = 0;
  x[3] = 0;
  x[4] = 0;
  x[5] = 0;
  x[6] = 0;

  /* x(n+7) = x(n+4) xor x(n), period 127 */
  for (int n = 0; n < NR_V0_PSS_LEN - 7; n++) {
    x[n + 7] = (uint8_t)((x[n + 4] + x[n]) & 1U);
  }

  int shift = (43 * (nid2 % 3)) % NR_V0_PSS_LEN;
  for (int n = 0; n < NR_V0_PSS_LEN; n++) {
    int idx = (n + shift) % NR_V0_PSS_LEN;
    seq[n] = x[idx] ? -1.0f : 1.0f; /* BPSK mapping */
  }
}

static void nr_v0_build_pss_td(int nid2, float *td_i, float *td_q)
{
  float pss[NR_V0_PSS_LEN];
  float Xr[NR_V0_NFFT];
  float Xi[NR_V0_NFFT];
  nr_v0_build_pss_seq(nid2, pss);
  memset(Xr, 0, sizeof(Xr));
  memset(Xi, 0, sizeof(Xi));

  const int k0 = -(NR_V0_PSS_LEN / 2);
  for (int m = 0; m < NR_V0_PSS_LEN; m++) {
    int k = k0 + m;
    int bin = (k >= 0) ? k : (NR_V0_NFFT + k);
    if (bin >= 0 && bin < NR_V0_NFFT) {
      Xr[bin] = pss[m];
    }
  }

  float xr[NR_V0_NFFT];
  float xq[NR_V0_NFFT];
  for (int n = 0; n < NR_V0_NFFT; n++) {
    double sr = 0.0;
    double si = 0.0;
    for (int k = 0; k < NR_V0_NFFT; k++) {
      double ph = 2.0 * M_PI * (double)(k * n) / (double)NR_V0_NFFT;
      sr += Xr[k] * cos(ph) - Xi[k] * sin(ph);
      si += Xr[k] * sin(ph) + Xi[k] * cos(ph);
    }
    xr[n] = (float)(sr / (double)NR_V0_NFFT);
    xq[n] = (float)(si / (double)NR_V0_NFFT);
  }

  for (int n = 0; n < NR_V0_CP; n++) {
    td_i[n] = xr[NR_V0_NFFT - NR_V0_CP + n];
    td_q[n] = xq[NR_V0_NFFT - NR_V0_CP + n];
  }
  for (int n = 0; n < NR_V0_NFFT; n++) {
    td_i[NR_V0_CP + n] = xr[n];
    td_q[NR_V0_CP + n] = xq[n];
  }
}

int nr_ssb_pss_search(const nr_iq_block_t *blk, nr_pss_hit_t *hits, int max_hits)
{
  if (!blk || !hits || max_hits <= 0) {
    return -1;
  }
  memset(hits, 0, sizeof(nr_pss_hit_t) * (size_t)max_hits);
  if (!blk->rx[0] || blk->nsamps < NR_V0_PSS_TD_LEN + 1U) {
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

  float pss_i[3][NR_V0_PSS_TD_LEN];
  float pss_q[3][NR_V0_PSS_TD_LEN];
  for (int nid2 = 0; nid2 < 3; nid2++) {
    nr_v0_build_pss_td(nid2, pss_i[nid2], pss_q[nid2]);
  }

  int found = 0;
  for (uint32_t n = 0; n + NR_V0_PSS_TD_LEN < blk->nsamps; n++) {
    for (int nid2 = 0; nid2 < 3; nid2++) {
      double cr = 0.0;
      double ci = 0.0;
      double px = 0.0;
      double pp = 0.0;
      for (int k = 0; k < NR_V0_PSS_TD_LEN; k++) {
        double xr = (double)x[n + (uint32_t)k].r;
        double xq = (double)x[n + (uint32_t)k].i;
        double pr = (double)pss_i[nid2][k];
        double pq = (double)pss_q[nid2][k];
        cr += xr * pr + xq * pq;
        ci += xq * pr - xr * pq;
        px += xr * xr + xq * xq;
        pp += pr * pr + pq * pq;
      }
      double metric = sqrt(cr * cr + ci * ci) / (sqrt(px * pp) + 1e-9);
      if (metric > 0.20) {
        int insert = found < max_hits ? found : (max_hits - 1);
        if (insert >= 0) {
          /* keep hits sorted by metric descending */
          while (insert > 0 && hits[insert - 1].metric < (float)metric) {
            if (insert < max_hits) {
              hits[insert] = hits[insert - 1];
            }
            insert--;
          }
          if (insert < max_hits) {
            hits[insert].peak_samp = (int32_t)n;
            hits[insert].coarse_cfo_hz = 0.0f;
            hits[insert].metric = (float)metric;
            hits[insert].nid2 = (uint8_t)nid2;
            if (found < max_hits) {
              found++;
            }
          }
        }
      }
    }
  }
  if (found <= 0) {
    return -1;
  }
  return 0;
}

int nr_ssb_refine_sync(const nr_iq_block_t *blk, const nr_pss_hit_t *hit,
                       nr_sync_state_t *sync)
{
  if (!blk || !hit || !sync) {
    return -1;
  }
  memset(sync, 0, sizeof(*sync));
  if (hit->metric < 0.20f) {
    sync->locked = 0;
    return -1;
  }
  sync->locked = 1;
  sync->coarse_offset_samp = hit->peak_samp;
  sync->frac_offset_samp = 0.0f;
  sync->cfo_hz = hit->coarse_cfo_hz;
  /* V1: expose nid2, defer full PCI until SSS path is implemented. */
  sync->pci = hit->nid2;
  sync->ssb_index = 0;
  sync->sfn = 0;
  sync->slot = 0;
  sync->snr_db = 10.0f * log10f(hit->metric + 1.0f);
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
  if (fabsf(sync->cfo_hz) > 5.0e5f) {
    return 1;
  }
  return 0;
}
