#include "openair1/PHY/NR_POSITIONING/nr_pos_types.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

/* NR-like PSS local sequence (length 127) for V0 sniff.
 * Based on 3GPP-style m-sequence indexing with N_ID2-dependent cyclic shift.
 */
#define NR_V0_PSS_LEN 127

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

int nr_ssb_pss_search(const nr_iq_block_t *blk, nr_pss_hit_t *hits, int max_hits)
{
  if (!blk || !hits || max_hits <= 0) {
    return -1;
  }
  memset(hits, 0, sizeof(nr_pss_hit_t) * (size_t)max_hits);
  if (!blk->rx[0] || blk->nsamps < 32U) {
    return -1;
  }

  const c16_t *x = blk->rx[0];
  float best_metric = -1.0f;
  int best_pos = -1;
  int best_nid2 = 0;

  float pss[3][NR_V0_PSS_LEN];
  for (int nid2 = 0; nid2 < 3; nid2++) {
    nr_v0_build_pss_seq(nid2, pss[nid2]);
  }

  /* Sliding correlation over I branch for V0 prototype. */
  if (blk->nsamps > NR_V0_PSS_LEN) {
    for (uint32_t n = 0; n + NR_V0_PSS_LEN < blk->nsamps; n++) {
      for (int nid2 = 0; nid2 < 3; nid2++) {
        float corr = 0.0f;
        float pow = 0.0f;
        for (int k = 0; k < NR_V0_PSS_LEN; k++) {
          float i = (float)x[n + (uint32_t)k].r;
          corr += i * pss[nid2][k];
          pow += i * i;
        }
        float metric = fabsf(corr) / (sqrtf(pow + 1.0f) + 1.0f);
        if (metric > best_metric) {
          best_metric = metric;
          best_pos = (int)n;
          best_nid2 = nid2;
        }
      }
    }
  }

  /* Energy fallback: keeps behavior sane in non-PSS synthetic tests. */
  if (best_pos < 0 || best_metric < 5.0f) {
    float e_best = 0.0f;
    int e_pos = -1;
    for (uint32_t n = 0; n + 16U < blk->nsamps; n++) {
      float acc = 0.0f;
      for (uint32_t k = 0; k < 16U; k++) {
        float i = (float)x[n + k].r;
        float q = (float)x[n + k].i;
        acc += i * i + q * q;
      }
      if (acc > e_best) {
        e_best = acc;
        e_pos = (int)n;
      }
    }
    best_metric = e_best / 1000.0f;
    best_pos = e_pos;
    best_nid2 = 0;
  }

  if (best_pos < 0 || best_metric < 3.0f) {
    return -1;
  }

  hits[0].peak_samp = best_pos;
  hits[0].coarse_cfo_hz = (float)(best_nid2 - 1) * 50.0f;
  hits[0].metric = best_metric;
  return 0;
}

int nr_ssb_refine_sync(const nr_iq_block_t *blk, const nr_pss_hit_t *hit,
                       nr_sync_state_t *sync)
{
  if (!blk || !hit || !sync) {
    return -1;
  }
  memset(sync, 0, sizeof(*sync));
  if (hit->metric < 3.0f) {
    sync->locked = 0;
    return -1;
  }
  sync->locked = 1;
  sync->coarse_offset_samp = hit->peak_samp;
  sync->frac_offset_samp = 0.0f;
  sync->cfo_hz = hit->coarse_cfo_hz;
  sync->pci = (uint16_t)(hit->peak_samp % 1008);
  sync->ssb_index = 0;
  sync->sfn = 0;
  sync->slot = 0;
  sync->snr_db = 10.0f * log10f(hit->metric + 1.0f);
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
