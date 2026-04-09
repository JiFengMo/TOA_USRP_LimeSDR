#include "openair1/PHY/NR_POSITIONING/nr_pos_provider_if.h"
#include "openair1/PHY/NR_POSITIONING/nr_pos_api.h"

#include <errno.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

enum { SSB_ACQ_MAX_CAND = 4 };

typedef struct {
  nr_sync_state_t sync;
  float rank;
  uint8_t pbch_attempted;
  int pbch_rc;
} ssb_acq_candidate_t;

static int ssb_target_pci_enabled(const nr_toa_ue_t *ue)
{
  return ue && ue->app_cfg.target_pci >= 0 && ue->app_cfg.target_pci < 1008;
}

static void ssb_force_target_pci(nr_sync_state_t *sync, int32_t target_pci)
{
  if (!sync || target_pci < 0 || target_pci >= 1008) {
    return;
  }
  sync->pci = (uint16_t)target_pci;
  sync->pci_full = (uint16_t)target_pci;
  sync->nid2 = (uint8_t)(target_pci % 3);
  sync->nid1 = (uint16_t)(target_pci / 3);
  sync->pci_hyp_count = 1U;
  sync->pci_hyp[0] = (uint16_t)target_pci;
  sync->pci_hyp_delta[0] = 0;
  sync->pci_hyp_metric[0] = sync->pss_metric;
}

static void ssb_acq_insert_candidate(ssb_acq_candidate_t *cands,
                                     uint32_t *count,
                                     uint32_t max_count,
                                     const ssb_acq_candidate_t *cand)
{
  uint32_t n = 0U;
  uint32_t pos = 0U;
  if (!cands || !count || !cand || max_count == 0U) {
    return;
  }
  n = (*count < max_count) ? *count : max_count;
  while (pos < n && cands[pos].rank >= cand->rank) {
    pos++;
  }
  if (pos >= max_count) {
    return;
  }
  if (n < max_count) {
    n++;
    *count = n;
  }
  for (uint32_t i = n - 1U; i > pos; i--) {
    cands[i] = cands[i - 1U];
  }
  cands[pos] = *cand;
}

static float ssb_acq_direct_rank(const nr_sync_state_t *sync)
{
  if (!sync) {
    return 0.0f;
  }
  return sync->pss_metric;
}

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
  const float min_pss_metric = 0.15f;
  const uint32_t pbch_attempt_limit = 4U;
  const int32_t target_pci = ssb_target_pci_enabled(ue) ? ue->app_cfg.target_pci : -1;
  const uint8_t target_nid2 = (uint8_t)((target_pci >= 0) ? (target_pci % 3) : 0);
  if (!blk || !sync) {
    return -1;
  }

  nr_pss_hit_t hits[4];
  if (nr_ssb_pss_search(blk, hits, 4) != 0) {
    sync->locked = 0;
  if ((dbg_cnt++ % 10U) == 0U) {
    printf("pss_search: not-detected\n");
  }
    return -1;
  }
  printf("pss_hits: [0]nid2=%u off=%d m=%.3f cfo=%.1f [1]nid2=%u off=%d m=%.3f [2]nid2=%u off=%d m=%.3f [3]nid2=%u off=%d m=%.3f\n",
         (unsigned)hits[0].nid2, hits[0].peak_samp, hits[0].metric, hits[0].coarse_cfo_hz,
         (unsigned)hits[1].nid2, hits[1].peak_samp, hits[1].metric,
         (unsigned)hits[2].nid2, hits[2].peak_samp, hits[2].metric,
         (unsigned)hits[3].nid2, hits[3].peak_samp, hits[3].metric);
  dbg_cnt++;
  ssb_acq_candidate_t cands[SSB_ACQ_MAX_CAND];
  uint32_t cand_count = 0U;
  nr_sync_state_t best_sync;
  int best_valid = 0;
  uint32_t best_idx = 0U;
  memset(cands, 0, sizeof(cands));
  for (int i = 0; i < 4; i++) {
    if (hits[i].metric <= 0.0f) {
      continue;
    }
    if (target_pci >= 0 && hits[i].nid2 != target_nid2) {
      continue;
    }
    ssb_acq_candidate_t cand;
    memset(&cand, 0, sizeof(cand));
    if (nr_ssb_refine_sync(blk, &hits[i], &cand.sync) != 0) {
      continue;
    }
    if (target_pci >= 0) {
      ssb_force_target_pci(&cand.sync, target_pci);
    }
    cand.rank = ssb_acq_direct_rank(&cand.sync);
    cand.pbch_attempted = 0U;
    cand.pbch_rc = -1;
    ssb_acq_insert_candidate(cands, &cand_count, SSB_ACQ_MAX_CAND, &cand);
  }

  printf("acq_cands: count=%u\n", cand_count);
  for (uint32_t d = 0U; d < cand_count; d++) {
    printf("  cand[%u]: pci=%u nid2=%u nid1=%u off=%d cfo=%.1f rank=%.3f pss=%.3f hyps=%u\n",
           d, (unsigned)cands[d].sync.pci, (unsigned)cands[d].sync.nid2,
           (unsigned)cands[d].sync.nid1,
           cands[d].sync.coarse_offset_samp, cands[d].sync.cfo_hz,
           cands[d].rank, cands[d].sync.pss_metric,
           (unsigned)cands[d].sync.pci_hyp_count);
  }
  for (uint32_t i = 0U; i < cand_count; i++) {
    cands[i].pbch_attempted = 0U;
    cands[i].pbch_rc = -1;
    if (i >= pbch_attempt_limit) {
      continue;
    }
    cands[i].pbch_attempted = 1U;
    cands[i].pbch_rc = nr_ssb_pbch_decode(blk, &cands[i].sync);
    printf("  pbch_cand[%u]: pci=%u rc=%d ok=%u dmrs=%.3f fail=%u\n",
           i, (unsigned)cands[i].sync.pci, cands[i].pbch_rc,
           (unsigned)cands[i].sync.pbch_ok, cands[i].sync.pbch_metric,
           (unsigned)cands[i].sync.pbch_fail_stage);
    if (cands[i].pbch_rc == 0 && cands[i].sync.pbch_ok) {
      cands[i].rank = 1000.0f + cands[i].sync.pbch_metric;
      best_sync = cands[i].sync;
      best_valid = 1;
      best_idx = i;
      break;
    }
  }

  if (!best_valid && cand_count > 0U) {
    uint32_t best_i = 0U;
    for (uint32_t i = 1U; i < cand_count; i++) {
      if (cands[i].rank > cands[best_i].rank) {
        best_i = i;
      }
    }
    best_sync = cands[best_i].sync;
    best_valid = 1;
    best_idx = best_i;
  }

  if (!best_valid) {
    sync->locked = 0;
    return -1;
  }
  *sync = best_sync;
  sync->last_gscn = blk->rx_gscn;

  /* Additive quality gate to suppress low-confidence/random locks. */
  if (fabsf(sync->cfo_hz) >= 14900.0f) {
    sync->locked = 0;
    return -1;
  }
  if (sync->pss_metric < min_pss_metric) {
    sync->locked = 0;
    return -1;
  }
  if (sync->coarse_offset_samp < 16 ||
      sync->coarse_offset_samp > (int32_t)(blk->nsamps - 16U)) {
      sync->locked = 0;
      return -1;
  }

  const int shortlist_pbch_attempted =
      (best_idx < cand_count) ? (int)cands[best_idx].pbch_attempted : 0;
  int pbch_rc = -1;
  if (shortlist_pbch_attempted) {
    pbch_rc = cands[best_idx].pbch_rc;
    if (pbch_rc == 0 && cands[best_idx].sync.pbch_ok) {
      sync->pbch_ok = cands[best_idx].sync.pbch_ok;
      sync->pbch_confirmed = 1;
      sync->mib_ok = cands[best_idx].sync.mib_ok;
      sync->mib_payload = cands[best_idx].sync.mib_payload;
    }
  }
  const int run_full_pbch = (!sync->pbch_ok) && !shortlist_pbch_attempted;
  if (run_full_pbch) {
    sync->pbch_ok = 0;
    sync->pbch_confirmed = 0;
    sync->mib_ok = 0;
    sync->mib_payload = 0U;
    pbch_rc = nr_ssb_pbch_decode(blk, sync);
  }
  if (pbch_rc == 0 && sync->pbch_ok) {
    sync->pbch_ok = 1;
    sync->pbch_confirmed = 1;
    sync->locked = 1;
    if (ue && ue->app_cfg.iq_dump_enable && sync->mib_ok) {
      ssb_dump_candidate_iq(ue, blk, sync, "pbch_ok");
    }
  } else {
    sync->locked = 0;
    sync->pbch_ok = 0;
    sync->pbch_confirmed = 0;
    sync->mib_ok = 0;
    if (ue && ue->app_cfg.iq_dump_enable &&
        (sync->pbch_fail_stage == NR_PBCH_FAIL_BCH ||
         sync->pbch_fail_stage == NR_PBCH_FAIL_DMRS_WEAK ||
         sync->pbch_fail_stage == NR_PBCH_FAIL_DMRS_AMBIG)) {
      ssb_dump_candidate_iq(ue, blk, sync, "pbch_nearmiss");
    }
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
