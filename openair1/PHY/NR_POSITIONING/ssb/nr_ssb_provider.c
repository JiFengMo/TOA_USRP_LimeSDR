#include "openair1/PHY/NR_POSITIONING/nr_pos_provider_if.h"
#include "openair1/PHY/NR_POSITIONING/nr_pos_api.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>

static int ssb_init(void *ctx)
{
  (void)ctx;
  return 0;
}

static int ssb_acquire(void *ctx, const nr_iq_block_t *blk, nr_sync_state_t *sync)
{
  (void)ctx;
  static uint32_t dbg_cnt = 0;
  /* Additive PBCH confirmation gate: require consecutive confirmations
   * before exposing lock=1 to upper state machine. */
  static uint32_t pbch_confirm_streak = 0;
  static uint32_t pbch_fail_streak = 0;
  const uint32_t pbch_confirm_need = 2U;
  const uint32_t pbch_fail_tolerate = 3U;
  /* Additive PCI consistency gate: majority vote over a short window. */
  enum { PCI_HIST_LEN = 6 };
  static uint16_t pci_hist[PCI_HIST_LEN] = {0};
  static uint8_t pci_hist_used = 0;
  static uint8_t pci_hist_pos = 0;
  const uint32_t pci_majority_need = 2U;
  /* Additive PCI confidence scorer to tolerate non-consecutive jitter. */
  enum { PCI_SCORE_SLOTS = 12 };
  static uint16_t pci_score_id[PCI_SCORE_SLOTS] = {0};
  static float pci_score_val[PCI_SCORE_SLOTS] = {0.0f};
  /* Additive long-window dominant PCI vote. */
  static uint16_t pci_vote[1008] = {0};
  static int32_t pci_anchor = -1;
  static uint32_t anchor_miss = 0;
  if (!blk || !sync) {
    return -1;
  }

  nr_pss_hit_t hits[4];
  if (nr_ssb_pss_search(blk, hits, 4) != 0) {
    sync->locked = 0;
    pbch_confirm_streak = 0;
    pbch_fail_streak = 0;
    pci_hist_used = 0;
    pci_hist_pos = 0;
    for (int s = 0; s < PCI_SCORE_SLOTS; s++) {
      pci_score_val[s] *= 0.95f;
    }
    if ((dbg_cnt++ % 100U) == 0U) {
      printf("pss_search: not-detected\n");
    }
    return -1;
  }
  if ((dbg_cnt++ % 100U) == 0U) {
    printf("pss_hits: [0]nid2=%u off=%d m=%.3f [1]nid2=%u off=%d m=%.3f [2]nid2=%u off=%d m=%.3f\n",
           (unsigned)hits[0].nid2, hits[0].peak_samp, hits[0].metric,
           (unsigned)hits[1].nid2, hits[1].peak_samp, hits[1].metric,
           (unsigned)hits[2].nid2, hits[2].peak_samp, hits[2].metric);
  }
  /* Try multiple PSS candidates and keep the best refined sync result. */
  nr_sync_state_t best_sync;
  int best_valid = 0;
  for (int i = 0; i < 4; i++) {
    if (hits[i].metric <= 0.0f) {
      continue;
    }
    nr_sync_state_t cand;
    if (nr_ssb_refine_sync(blk, &hits[i], &cand) != 0) {
      continue;
    }
    if (!best_valid || cand.snr_db > best_sync.snr_db) {
      best_sync = cand;
      best_valid = 1;
    }
  }

  if (!best_valid) {
    sync->locked = 0;
    pbch_confirm_streak = 0;
    pbch_fail_streak = 0;
    pci_hist_used = 0;
    pci_hist_pos = 0;
    for (int s = 0; s < PCI_SCORE_SLOTS; s++) {
      pci_score_val[s] *= 0.97f;
    }
    return -1;
  }
  *sync = best_sync;
  if (sync->pci < 1008U) {
    uint16_t add = (sync->snr_db > 0.85f) ? 3U : 1U;
    if ((uint32_t)pci_vote[sync->pci] + add > 60000U) {
      /* Soft decay to avoid saturation and keep responsiveness. */
      for (int v = 0; v < 1008; v++) {
        pci_vote[v] = (uint16_t)(pci_vote[v] >> 1);
      }
    }
    pci_vote[sync->pci] = (uint16_t)(pci_vote[sync->pci] + add);
  }

  uint16_t dom_pci_long = sync->pci;
  uint16_t dom_vote = 0;
  uint16_t sec_vote = 0;
  for (int v = 0; v < 1008; v++) {
    uint16_t c = pci_vote[v];
    if (c > dom_vote) {
      sec_vote = dom_vote;
      dom_vote = c;
      dom_pci_long = (uint16_t)v;
    } else if (c > sec_vote) {
      sec_vote = c;
    }
  }

  if (pci_anchor < 0 && dom_vote >= 4U && dom_vote > (uint16_t)(sec_vote + 1U)) {
    pci_anchor = (int32_t)dom_pci_long;
    anchor_miss = 0;
    printf("pci_anchor: set=%d vote=%u second=%u\n",
           (int)pci_anchor, (unsigned)dom_vote, (unsigned)sec_vote);
  }
  if (pci_anchor >= 0) {
    if (sync->pci != (uint16_t)pci_anchor) {
      anchor_miss++;
    } else {
      anchor_miss = 0;
    }
    if (anchor_miss > 200U) {
      printf("pci_anchor: released=%d miss=%u\n", (int)pci_anchor, (unsigned)anchor_miss);
      pci_anchor = -1;
      anchor_miss = 0;
    }
  }

  pci_hist[pci_hist_pos] = sync->pci;
  pci_hist_pos = (uint8_t)((pci_hist_pos + 1U) % PCI_HIST_LEN);
  if (pci_hist_used < PCI_HIST_LEN) {
    pci_hist_used++;
  }

  uint16_t pci_dom = sync->pci;
  uint32_t pci_dom_cnt = 0;
  for (uint8_t i = 0; i < pci_hist_used; i++) {
    uint16_t p = pci_hist[i];
    uint32_t c = 0;
    for (uint8_t j = 0; j < pci_hist_used; j++) {
      if (pci_hist[j] == p) {
        c++;
      }
    }
    if (c > pci_dom_cnt) {
      pci_dom_cnt = c;
      pci_dom = p;
    }
  }
  if (pci_dom_cnt < pci_majority_need) {
    /* Update PCI confidence scores (EMA-like decay + additive reward by SNR). */
    for (int s = 0; s < PCI_SCORE_SLOTS; s++) {
      pci_score_val[s] *= 0.96f;
    }
    int slot = -1;
    for (int s = 0; s < PCI_SCORE_SLOTS; s++) {
      if (pci_score_val[s] > 0.0f && pci_score_id[s] == sync->pci) {
        slot = s;
        break;
      }
    }
    if (slot < 0) {
      int min_s = 0;
      for (int s = 1; s < PCI_SCORE_SLOTS; s++) {
        if (pci_score_val[s] < pci_score_val[min_s]) {
          min_s = s;
        }
      }
      slot = min_s;
      pci_score_id[slot] = sync->pci;
      pci_score_val[slot] = 0.0f;
    }
    pci_score_val[slot] += (sync->snr_db > 0.0f) ? sync->snr_db : 0.1f;

    int best_s = 0, second_s = 0;
    for (int s = 1; s < PCI_SCORE_SLOTS; s++) {
      if (pci_score_val[s] > pci_score_val[best_s]) {
        second_s = best_s;
        best_s = s;
      } else if (s != best_s && pci_score_val[s] > pci_score_val[second_s]) {
        second_s = s;
      }
    }
    const float best_v = pci_score_val[best_s];
    const float second_v = pci_score_val[second_s];
    if (best_v >= 2.2f && best_v > 1.35f * (second_v + 1e-3f)) {
      sync->pci = pci_score_id[best_s];
      /* Let PBCH gate decide final lock after confidence-selected PCI. */
    } else {
      /* Additive fast-lock fallback: if instantaneous quality is already high,
       * allow lock while continuing to build PCI confidence in background. */
      if (!(sync->snr_db >= 0.72f && hits[0].metric >= 0.16f)) {
        sync->locked = 0;
        if ((dbg_cnt % 100U) == 0U) {
          printf("pci_gate: warmup dom_pci=%u cnt=%u/%u win=%u score_best=%.2f score_2nd=%.2f\n",
                 (unsigned)pci_dom,
                 (unsigned)pci_dom_cnt,
                 (unsigned)pci_majority_need,
                 (unsigned)pci_hist_used,
                 best_v, second_v);
        }
        return -1;
      }
    }
  } else {
    sync->pci = pci_dom;
  }

  /* Long-window dominant PCI gate: when a stable leader emerges, bind lock to it.
   * This suppresses rapid PCI hopping under noisy detections. */
  if (dom_vote >= 8U && dom_vote > (uint16_t)(sec_vote + 2U)) {
    if (sync->pci != dom_pci_long) {
      sync->locked = 0;
      return -1;
    }
    sync->pci = dom_pci_long;
  }
  if (pci_anchor >= 0 && sync->pci != (uint16_t)pci_anchor) {
    sync->locked = 0;
    return -1;
  }

  /* Additive PBCH confirmation (current decode is still lightweight, but this
   * gate reduces one-shot false locks and stabilizes lock transitions). */
  const int pbch_rc = nr_ssb_pbch_decode(blk, sync);
  if (pbch_rc == 0) {
    pbch_confirm_streak++;
    pbch_fail_streak = 0;
  } else {
    pbch_fail_streak++;
    if (pbch_fail_streak >= pbch_fail_tolerate) {
      pbch_confirm_streak = 0;
      pci_hist_used = 0;
      pci_hist_pos = 0;
      for (int s = 0; s < PCI_SCORE_SLOTS; s++) {
        pci_score_val[s] *= 0.9f;
      }
    }
  }
  if (pbch_confirm_streak < pbch_confirm_need) {
    sync->locked = 0;
    sync->pbch_confirmed = 0;
    if ((dbg_cnt % 100U) == 0U) {
      printf("pbch_gate: warmup confirm=%u/%u\n",
             (unsigned)pbch_confirm_streak, (unsigned)pbch_confirm_need);
    }
    return -1;
  }
  sync->pbch_confirmed = 1;

  /* Additive enhancement: only (re-)seed pci_anchor after PBCH confirmed.
   * This binds the long-window PCI vote with the PBCH consistency gate. */
  if (pci_anchor < 0 && dom_vote >= 2U && dom_vote >= sec_vote) {
    pci_anchor = (int32_t)dom_pci_long;
    anchor_miss = 0;
    printf("pci_anchor(PBCH): set=%d vote=%u second=%u\n",
           (int)pci_anchor, (unsigned)dom_vote, (unsigned)sec_vote);
  }

  /* Additive quality gate to suppress low-confidence/random locks. */
  if (fabsf(sync->cfo_hz) >= 14900.0f) {
    sync->locked = 0;
    return -1;
  }
  if (sync->pss_metric < 0.17f) {
    sync->locked = 0;
    return -1;
  }
  if (sync->coarse_offset_samp < 16 ||
      sync->coarse_offset_samp > (int32_t)(blk->nsamps - 16U)) {
    sync->locked = 0;
    return -1;
  }
  return 0;
}

static int ssb_track(void *ctx, const nr_iq_block_t *blk, nr_sync_state_t *sync)
{
  (void)ctx;
  if (!blk || !sync) {
    return -1;
  }

  if (nr_ssb_track_cfo(blk, sync) != 0) {
    sync->locked = 0;
    return -1;
  }
  int sample_shift = 0;
  if (nr_ssb_track_timing(blk, sync, &sample_shift) != 0) {
    sync->locked = 0;
    return -1;
  }
  if (nr_ssb_check_lost_lock(sync) != 0) {
    sync->locked = 0;
  }
  return 0;
}

static int ssb_extract_meas(void *ctx, const nr_iq_block_t *blk,
                            const nr_sync_state_t *sync, nr_toa_meas_t *meas)
{
  (void)ctx;
  if (!blk || !sync || !meas) {
    return -1;
  }

  nr_ssb_window_t win;
  nr_ssb_grid_t grid;
  nr_chest_t h;
  nr_chest_full_t hf;
  nr_cir_t cir;
  int peak_idx = 0;
  double frac = 0.0;

  if (nr_ssb_extract_window(blk, sync, &win) != 0) {
    return -1;
  }
  if (nr_ssb_demod(&win, &grid) != 0) {
    return -1;
  }
  if (nr_ssb_ls_estimate(&grid, &h) != 0) {
    return -1;
  }
  if (nr_ssb_interp_channel(&h, &hf) != 0) {
    return -1;
  }
  if (nr_ssb_build_cir(&hf, &cir) != 0) {
    return -1;
  }
  if (nr_toa_find_integer_peak(&cir, &peak_idx) != 0) {
    return -1;
  }
  if (nr_toa_refine_fractional(&cir, peak_idx, &frac) != 0) {
    return -1;
  }
  if (nr_toa_build_meas(sync, blk, peak_idx, frac, meas) != 0) {
    return -1;
  }
  return 0;
}

static int ssb_dump_trace(void *ctx)
{
  (void)ctx;
  return 0;
}

const nr_pos_provider_if_t nr_ssb_provider = {
    .name = "ssb_provider",
    .init = ssb_init,
    .acquire = ssb_acquire,
    .track = ssb_track,
    .extract_meas = ssb_extract_meas,
    .dump_trace = ssb_dump_trace,
};
