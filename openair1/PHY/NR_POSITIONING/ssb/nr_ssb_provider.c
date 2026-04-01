#include "openair1/PHY/NR_POSITIONING/nr_pos_provider_if.h"
#include "openair1/PHY/NR_POSITIONING/nr_pos_api.h"

#include <errno.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

enum { SSB_CAND_CACHE_SIZE = 48 };

typedef struct {
  int32_t gscn;
  uint16_t pci;
  uint8_t ssb;
  float heat;
  float pbch_best;
  float pss_best;
  uint16_t hits;
  uint8_t fail_stage;
  uint32_t age;
} ssb_cand_cache_entry_t;

static const char *ssb_pbch_fail_stage_name(uint8_t stage)
{
  switch ((nr_pbch_fail_stage_t)stage) {
    case NR_PBCH_FAIL_NONE:
      return "none";
    case NR_PBCH_FAIL_WINDOW:
      return "window";
    case NR_PBCH_FAIL_DEMOD:
      return "demod";
    case NR_PBCH_FAIL_DMRS_WEAK:
      return "dmrs_weak";
    case NR_PBCH_FAIL_DMRS_AMBIG:
      return "dmrs_ambig";
    case NR_PBCH_FAIL_BCH:
      return "bch_crc";
    default:
      return "unknown";
  }
}

static float ssb_candidate_score(const nr_sync_state_t *sync)
{
  float score = 0.0f;
  if (!sync) {
    return 0.0f;
  }
  score = 0.45f * sync->pss_metric + 0.08f * sync->snr_db;
  switch ((nr_pbch_fail_stage_t)sync->pbch_fail_stage) {
    case NR_PBCH_FAIL_BCH:
      score += 1.20f * sync->pbch_metric + 0.12f;
      break;
    case NR_PBCH_FAIL_DMRS_AMBIG:
      score += 0.70f * sync->pbch_metric + 0.04f;
      break;
    case NR_PBCH_FAIL_DMRS_WEAK:
      score += 0.30f * sync->pbch_metric;
      break;
    default:
      score += 0.12f * sync->pbch_metric;
      break;
  }
  if (sync->pbch_ok) {
    score += 2.5f;
  }
  if (sync->mib_ok) {
    score += 4.0f;
  }
  if (score < 0.0f) {
    score = 0.0f;
  }
  return score;
}

static void ssb_candidate_cache_decay(ssb_cand_cache_entry_t *cache, uint32_t now_age)
{
  if (!cache) {
    return;
  }
  for (int i = 0; i < SSB_CAND_CACHE_SIZE; i++) {
    if (cache[i].hits == 0U) {
      continue;
    }
    uint32_t delta = now_age - cache[i].age;
    while (delta > 0U) {
      cache[i].heat *= 0.965f;
      cache[i].pbch_best *= 0.985f;
      cache[i].pss_best *= 0.990f;
      delta--;
    }
    cache[i].age = now_age;
    if (cache[i].heat < 0.04f && cache[i].pbch_best < 0.08f) {
      memset(&cache[i], 0, sizeof(cache[i]));
      cache[i].gscn = -1;
    }
  }
}

static int ssb_candidate_cache_find_slot(ssb_cand_cache_entry_t *cache,
                                         int32_t gscn,
                                         uint16_t pci,
                                         uint8_t ssb,
                                         uint32_t now_age)
{
  int free_i = -1;
  int min_i = 0;
  float min_score = 1.0e9f;
  if (!cache) {
    return -1;
  }
  for (int i = 0; i < SSB_CAND_CACHE_SIZE; i++) {
    if (cache[i].hits == 0U || cache[i].gscn < 0) {
      if (free_i < 0) {
        free_i = i;
      }
      continue;
    }
    if (cache[i].gscn == gscn && cache[i].pci == pci && cache[i].ssb == ssb) {
      return i;
    }
    {
      const uint32_t age_delta = now_age - cache[i].age;
      const float score = cache[i].heat + 0.40f * cache[i].pbch_best - 0.01f * (float)age_delta;
      if (score < min_score) {
        min_score = score;
        min_i = i;
      }
    }
  }
  if (free_i >= 0) {
    return free_i;
  }
  return min_i;
}

static float ssb_candidate_cache_lookup(const ssb_cand_cache_entry_t *cache,
                                        int32_t gscn,
                                        uint16_t pci)
{
  float best = 0.0f;
  if (!cache || gscn < 0) {
    return 0.0f;
  }
  for (int i = 0; i < SSB_CAND_CACHE_SIZE; i++) {
    if (cache[i].hits == 0U || cache[i].gscn != gscn || cache[i].pci != pci) {
      continue;
    }
    {
      const float score = cache[i].heat + 0.35f * cache[i].pbch_best + 0.10f * cache[i].pss_best;
      if (score > best) {
        best = score;
      }
    }
  }
  return best;
}

static void ssb_candidate_cache_update(ssb_cand_cache_entry_t *cache,
                                       int32_t gscn,
                                       const nr_sync_state_t *sync,
                                       uint32_t now_age)
{
  int slot = -1;
  float ev = 0.0f;
  if (!cache || !sync || gscn < 0 || sync->pci >= 1008U) {
    return;
  }
  slot = ssb_candidate_cache_find_slot(cache, gscn, sync->pci, sync->ssb_index, now_age);
  if (slot < 0) {
    return;
  }
  if (cache[slot].hits == 0U || cache[slot].gscn != gscn ||
      cache[slot].pci != sync->pci || cache[slot].ssb != sync->ssb_index) {
    memset(&cache[slot], 0, sizeof(cache[slot]));
    cache[slot].gscn = gscn;
    cache[slot].pci = sync->pci;
    cache[slot].ssb = sync->ssb_index;
  }
  ev = ssb_candidate_score(sync);
  cache[slot].heat = fmaxf(ev, 0.86f * cache[slot].heat + 0.14f * ev);
  if (sync->pbch_metric > cache[slot].pbch_best) {
    cache[slot].pbch_best = sync->pbch_metric;
  } else {
    cache[slot].pbch_best *= 0.995f;
  }
  if (sync->pss_metric > cache[slot].pss_best) {
    cache[slot].pss_best = sync->pss_metric;
  }
  if (cache[slot].hits < 65535U) {
    cache[slot].hits++;
  }
  cache[slot].fail_stage = sync->pbch_fail_stage;
  cache[slot].age = now_age;
}

static void ssb_dump_candidate_iq(const nr_toa_ue_t *ue,
                                  const nr_iq_block_t *blk,
                                  const nr_sync_state_t *sync,
                                  const char *tag)
{
  static uint32_t dump_count = 0U;
  static openair0_timestamp_t last_ts = (openair0_timestamp_t)(~0ULL);
  static uint16_t last_pci = 0U;
  static uint8_t last_stage = 0U;
  const char *dir = NULL;
  char base[1024];
  char iq_path[1152];
  char meta_path[1152];
  FILE *iq_fp = NULL;
  FILE *meta_fp = NULL;

  if (!ue || !blk || !sync || !blk->rx[0] || !ue->app_cfg.iq_dump_enable) {
    return;
  }
  if (dump_count >= 12U) {
    return;
  }
  if (blk->ts_first == last_ts &&
      sync->pci == last_pci &&
      sync->pbch_fail_stage == last_stage) {
    return;
  }

  dir = getenv("NR_TOA_IQ_DUMP_DIR");
  if (!dir || dir[0] == '\0') {
    dir = "/tmp/nr_toa_iq";
  }
  if (mkdir(dir, 0777) != 0 && errno != EEXIST) {
    return;
  }

  snprintf(base, sizeof(base),
           "%s/%s_seq%02u_ts%llu_cf%.0f_fs%.0f_pci%u_ssb%u_pss%03u_dmrs%03u",
           dir,
           tag ? tag : "ssb",
           dump_count,
           (unsigned long long)blk->ts_first,
           ue->app_cfg.center_freq_hz,
           blk->fs_hz,
           (unsigned)sync->pci,
           (unsigned)sync->ssb_index,
           (unsigned)lroundf(sync->pss_metric * 1000.0f),
           (unsigned)lroundf(sync->pbch_metric * 1000.0f));
  snprintf(iq_path, sizeof(iq_path), "%s.c16", base);
  snprintf(meta_path, sizeof(meta_path), "%s.meta.txt", base);

  iq_fp = fopen(iq_path, "wb");
  if (!iq_fp) {
    return;
  }
  if (fwrite(blk->rx[0], sizeof(c16_t), blk->nsamps, iq_fp) != blk->nsamps) {
    fclose(iq_fp);
    (void)remove(iq_path);
    return;
  }
  fclose(iq_fp);

  meta_fp = fopen(meta_path, "w");
  if (meta_fp) {
    fprintf(meta_fp, "tag=%s\n", tag ? tag : "ssb");
    fprintf(meta_fp, "reason=%s\n", ssb_pbch_fail_stage_name(sync->pbch_fail_stage));
    fprintf(meta_fp, "ts_first=%llu\n", (unsigned long long)blk->ts_first);
    fprintf(meta_fp, "abs_samp0=%llu\n", (unsigned long long)blk->abs_samp0);
    fprintf(meta_fp, "nsamps=%u\n", blk->nsamps);
    fprintf(meta_fp, "fs_hz=%.0f\n", blk->fs_hz);
    fprintf(meta_fp, "center_freq_hz=%.0f\n", ue->app_cfg.center_freq_hz);
    fprintf(meta_fp, "rx_gain_db=%.1f\n", ue->app_cfg.rx_gain_db);
    fprintf(meta_fp, "pci=%u\n", (unsigned)sync->pci);
    fprintf(meta_fp, "ssb_index=%u\n", (unsigned)sync->ssb_index);
    fprintf(meta_fp, "coarse_offset_samp=%d\n", sync->coarse_offset_samp);
    fprintf(meta_fp, "cfo_hz=%.2f\n", sync->cfo_hz);
    fprintf(meta_fp, "snr_db=%.2f\n", sync->snr_db);
    fprintf(meta_fp, "pss_metric=%.6f\n", sync->pss_metric);
    fprintf(meta_fp, "pbch_metric=%.6f\n", sync->pbch_metric);
    fprintf(meta_fp, "pbch_metric_second=%.6f\n", sync->pbch_metric_second);
    fprintf(meta_fp, "pbch_ok=%u\n", (unsigned)sync->pbch_ok);
    fprintf(meta_fp, "mib_ok=%u\n", (unsigned)sync->mib_ok);
    fprintf(meta_fp, "mib_payload=0x%06x\n", (unsigned)sync->mib_payload);
    fclose(meta_fp);
  }

  last_ts = blk->ts_first;
  last_pci = sync->pci;
  last_stage = sync->pbch_fail_stage;
  dump_count++;
  printf("iq_dump: saved tag=%s reason=%s iq=%s meta=%s\n",
         tag ? tag : "ssb",
         ssb_pbch_fail_stage_name(sync->pbch_fail_stage),
         iq_path, meta_path);
}

static int ssb_init(void *ctx)
{
  (void)ctx;
  return 0;
}

static int ssb_acquire(void *ctx, const nr_iq_block_t *blk, nr_sync_state_t *sync)
{
  nr_toa_ue_t *ue = (nr_toa_ue_t *)ctx;
  static uint32_t dbg_cnt = 0;
  static uint32_t cand_age = 0U;
  static uint8_t cand_cache_init = 0U;
  /* Track PBCH/MIB separately from SSB lock so we can relax live SSB gating
   * without ever fabricating a successful PBCH/MIB accept. */
  static uint32_t pbch_confirm_streak = 0;
  static uint32_t pbch_fail_streak = 0;
  static uint32_t pbch_probe_round = 0;
  static uint32_t pbch_full_round = 0;
  const uint32_t pbch_confirm_need = 1U;
  const uint32_t pbch_fail_tolerate = 4U;
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
  /* Additive final consistency gate on fully qualified lock outputs. */
  static uint16_t final_pci_last = 0;
  static uint32_t final_pci_streak = 0;
  static ssb_cand_cache_entry_t cand_cache[SSB_CAND_CACHE_SIZE];
  const uint32_t final_pci_need = 1U;
  const float pci_score_lock_need = 1.15f;
  const float pci_score_ratio_need = 1.08f;
  const float relaxed_metric_floor = 0.160f;
  const float relaxed_snr_floor = 0.85f;
  if (!blk || !sync) {
    return -1;
  }
  if (!cand_cache_init) {
    for (int i = 0; i < SSB_CAND_CACHE_SIZE; i++) {
      cand_cache[i].gscn = -1;
    }
    cand_cache_init = 1U;
  }
  cand_age++;
  ssb_candidate_cache_decay(cand_cache, cand_age);

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
  float best_score = -1.0f;
  for (int i = 0; i < 4; i++) {
    if (hits[i].metric <= 0.0f) {
      continue;
    }
    nr_sync_state_t cand;
    nr_sync_state_t probe;
    float score = 0.0f;
    float hist_score = 0.0f;
    const uint32_t probe_tick = pbch_probe_round++;
    if (nr_ssb_refine_sync(blk, &hits[i], &cand) != 0) {
      continue;
    }
    hist_score = ssb_candidate_cache_lookup(cand_cache, blk->rx_gscn, cand.pci);
    score = cand.snr_db + 0.35f * cand.pss_metric + 0.30f * hist_score;
    if (i == 0 &&
        (cand.pss_metric >= 0.22f ||
         cand.snr_db >= 1.15f ||
         hist_score >= 0.24f ||
         ((probe_tick & 3U) == 0U))) {
      probe = cand;
      if (nr_ssb_pbch_decode(blk, &probe) == 0 && probe.pbch_ok) {
        score = 1000.0f + probe.pbch_metric;
        cand = probe;
      } else {
        cand.ssb_index = probe.ssb_index;
        cand.pbch_metric = probe.pbch_metric;
        cand.pbch_metric_second = probe.pbch_metric_second;
        cand.pbch_fail_stage = probe.pbch_fail_stage;
        cand.coarse_offset_samp = probe.coarse_offset_samp;
        score = cand.snr_db + 0.55f * probe.pbch_metric + 0.20f * cand.pss_metric +
                0.40f * hist_score;
      }
    }
    if (!best_valid || score > best_score) {
      best_score = score;
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
  sync->last_gscn = blk->rx_gscn;
  sync->pbch_ok = 0;
  sync->pbch_confirmed = 0;
  sync->mib_ok = 0;
  sync->mib_payload = 0U;
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
  uint16_t sec_pci_long = sync->pci;
  uint16_t dom_vote = 0;
  uint16_t sec_vote = 0;
  for (int v = 0; v < 1008; v++) {
    uint16_t c = pci_vote[v];
    if (c > dom_vote) {
      sec_pci_long = dom_pci_long;
      sec_vote = dom_vote;
      dom_vote = c;
      dom_pci_long = (uint16_t)v;
    } else if (c > sec_vote) {
      sec_vote = c;
      sec_pci_long = (uint16_t)v;
    }
  }
  if (sec_vote >= 4U && sec_vote + 1U >= dom_vote && (dbg_cnt % 50U) == 0U) {
    printf("pci_mix: long_votes top1=%u(%u) top2=%u(%u) cand=%u pss=%.3f snr=%.2f\n",
           (unsigned)dom_pci_long, (unsigned)dom_vote,
           (unsigned)sec_pci_long, (unsigned)sec_vote,
           (unsigned)sync->pci, sync->pss_metric, sync->snr_db);
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
    if (best_v >= pci_score_lock_need &&
        best_v > pci_score_ratio_need * (second_v + 1e-3f)) {
      sync->pci = pci_score_id[best_s];
    } else if (sync->pss_metric >= relaxed_metric_floor &&
               sync->snr_db >= relaxed_snr_floor) {
      if ((dbg_cnt % 100U) == 0U) {
        printf("pci_gate: relaxed pci=%u dom_pci=%u cnt=%u/%u score_best=%.2f score_2nd=%.2f metric=%.3f snr=%.2f\n",
               (unsigned)sync->pci,
               (unsigned)pci_dom,
               (unsigned)pci_dom_cnt,
               (unsigned)pci_majority_need,
               best_v, second_v, sync->pss_metric, sync->snr_db);
      }
    } else {
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
  } else {
    sync->pci = pci_dom;
  }

  /* Long-window vote is now a soft preference only. Same-ARFCN multi-PCI
   * deployments are common enough that hard rejection hurts robustness. */
  if (dom_vote >= 10U &&
      dom_vote > (uint16_t)(sec_vote + 6U) &&
      sync->pci != dom_pci_long &&
      sync->pci != sec_pci_long) {
    sync->pci = dom_pci_long;
    sync->pci_full = sync->pci;
    sync->nid2 = (uint8_t)(sync->pci % 3U);
    sync->nid1 = (uint16_t)(sync->pci / 3U);
  }

  /* Additive quality gate to suppress low-confidence/random locks. */
  if (fabsf(sync->cfo_hz) >= 14900.0f) {
    sync->locked = 0;
    return -1;
  }
  if (sync->pss_metric < relaxed_metric_floor) {
    sync->locked = 0;
    return -1;
  }
  if (sync->coarse_offset_samp < 16 ||
      sync->coarse_offset_samp > (int32_t)(blk->nsamps - 16U)) {
    sync->locked = 0;
    return -1;
  }

  /* Final SSB consistency: require the same PCI to survive all gates twice
   * before exposing lock=1 to the rest of the pipeline. */
  if (sync->pci == final_pci_last) {
    final_pci_streak++;
  } else {
    final_pci_last = sync->pci;
    final_pci_streak = 1;
  }
  if (final_pci_streak < final_pci_need) {
    sync->locked = 0;
    return -1;
  }
  sync->locked = 1;

  /* PBCH/MIB status is reported truthfully and is not allowed to fabricate
   * a stronger lock state than the code can actually verify. */
  const float cand_hist = ssb_candidate_cache_lookup(cand_cache, blk->rx_gscn, sync->pci);
  const int run_full_pbch =
      (sync->pss_metric >= 0.195f) ||
      (sync->snr_db >= 1.05f) ||
      (cand_hist >= 0.22f) ||
      (pci_dom_cnt >= pci_majority_need) ||
      (dom_vote >= 4U && sync->pci == dom_pci_long) ||
      ((pbch_full_round++ % 6U) == 0U);
  const int pbch_rc = run_full_pbch ? nr_ssb_pbch_decode(blk, sync) : -1;
  if (pbch_rc == 0 && sync->pbch_ok) {
    pbch_confirm_streak++;
    pbch_fail_streak = 0;
    sync->pbch_ok = 1;
    if (pbch_confirm_streak >= pbch_confirm_need) {
      sync->pbch_confirmed = 1;
    }
    if (ue && ue->app_cfg.iq_dump_enable && sync->mib_ok) {
      ssb_dump_candidate_iq(ue, blk, sync, "pbch_ok");
    }
    ssb_candidate_cache_update(cand_cache, blk->rx_gscn, sync, cand_age);
  } else {
    pbch_fail_streak++;
    sync->pbch_ok = 0;
    sync->pbch_confirmed = 0;
    sync->mib_ok = 0;
    if (run_full_pbch &&
        (sync->pbch_fail_stage == NR_PBCH_FAIL_BCH ||
         sync->pbch_fail_stage == NR_PBCH_FAIL_DMRS_AMBIG ||
         sync->pbch_fail_stage == NR_PBCH_FAIL_DMRS_WEAK)) {
      ssb_candidate_cache_update(cand_cache, blk->rx_gscn, sync, cand_age);
      if ((dbg_cnt % 50U) == 0U) {
        printf("cand_cache: gscn=%d pci=%u ssb=%u heat=%.3f pbch=%.3f stage=%u\n",
               blk->rx_gscn,
               (unsigned)sync->pci,
               (unsigned)sync->ssb_index,
               ssb_candidate_cache_lookup(cand_cache, blk->rx_gscn, sync->pci),
               sync->pbch_metric,
               (unsigned)sync->pbch_fail_stage);
      }
    }
    if (run_full_pbch && ue && ue->app_cfg.iq_dump_enable &&
        (sync->pbch_fail_stage == NR_PBCH_FAIL_BCH ||
         sync->pbch_fail_stage == NR_PBCH_FAIL_DMRS_WEAK ||
         sync->pbch_fail_stage == NR_PBCH_FAIL_DMRS_AMBIG) &&
        (sync->pss_metric >= 0.18f || sync->pbch_metric >= 0.12f)) {
      ssb_dump_candidate_iq(ue, blk, sync, "pbch_nearmiss");
    }
    if (pbch_fail_streak >= pbch_fail_tolerate) {
      pbch_confirm_streak = 0;
      for (int s = 0; s < PCI_SCORE_SLOTS; s++) {
        pci_score_val[s] *= 0.9f;
      }
    }
    if (!run_full_pbch) {
      sync->pbch_fail_stage = NR_PBCH_FAIL_NONE;
    }
    if ((dbg_cnt % 100U) == 0U) {
      printf("pbch_gate: %s confirm=%u/%u (SSB lock kept, MIB invalid)\n",
             run_full_pbch ? "unavailable" : "deferred",
             (unsigned)pbch_confirm_streak, (unsigned)pbch_confirm_need);
    }
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
  if (!sync->pbch_ok || !sync->pbch_confirmed || !sync->mib_ok) {
    /* Keep TOA pipeline dormant until a real PBCH/MIB decode exists. */
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
  if (nr_ssb_demod(blk, &win, sync->cfo_hz, &grid) != 0) {
    return -1;
  }
  if (nr_ssb_ls_estimate(&grid, sync, &h) != 0) {
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
  meas->peak_metric = cir.peak_metric;
  meas->quality = (cir.peak_metric > 1.0f) ? fminf(cir.peak_metric / 8.0f, 1.0f) : 0.0f;
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
