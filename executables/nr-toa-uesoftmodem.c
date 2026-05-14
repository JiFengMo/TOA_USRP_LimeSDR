#include "executables/nr-toa-softmodem.h"
#include "executables/nr-toa-threads.h"
#include "openair1/PHY/NR_POSITIONING/nr_pos_api.h"

#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <time.h>

volatile int oai_exit;
#define NR_SYNC_LOST_MISS_THRESH 24U
#define NR_RF_SETTLE_READS 2U

typedef struct {
  FILE *fp;
  char path[1024];
  uint8_t enabled;
  uint32_t rows;
  double win_ns[32];
  uint32_t win_head;
  uint32_t win_count;
} nr_rel_delay_logger_t;

static nr_rel_delay_logger_t g_rel_delay_logger;

static const char *nr_mib_scs_common_name(uint8_t scs_common)
{
  switch (scs_common & 0x1U) {
    case 0:
      return "15k";
    case 1:
      return "30k";
    default:
      return "unknown";
  }
}

static void nr_toa_log_type0_css(const nr_type0_css_info_t *css,
                                 uint32_t sfn,
                                 uint32_t slot)
{
  const int active = nr_type0_css_is_monitoring_occasion(css, sfn, slot);

  if (!css || !css->valid) {
    return;
  }

  printf("type0_css: scs_ssb=%ukHz scs_pdcch=%ukHz mux=%u coreset_rbs=%d coreset_sym=%d rb_offset=%d first_sym=%u duration=%u period_slots=%u n0=%u nc=%u sfn_c=%d monitor_now=%d frame=%u slot=%u\n",
         (unsigned)css->scs_ssb_khz,
         (unsigned)css->scs_pdcch_khz,
         (unsigned)css->mux_pattern,
         css->num_rbs,
         css->num_symbols,
         css->rb_offset,
         (unsigned)css->first_symbol_index,
         (unsigned)css->search_space_duration,
         (unsigned)css->search_space_frame_period,
         (unsigned)css->n_0,
         (unsigned)css->n_c,
         css->sfn_c,
         active,
         (unsigned)sfn,
         (unsigned)slot);
}

static void nr_toa_log_sib1_rx_plan(const nr_sib1_rx_plan_t *plan)
{
  if (!plan || !plan->valid) {
    return;
  }

  printf("sib1_rx_plan: si_rnti=0x%04x dci=1_0 css=common refPoint=pointA k_ssb=%u k_ssb_norm=%u coreset0_present=%u bwp_size_rb=%d bwp_start_rb=%s pdcch_scs=%ukHz dlsch_scs=%ukHz first_sym=%u period_slots=%u duration=%u active_now=%u frame=%u slot=%u mapping=%s L=%u R=%u shift=%u dmrs_id=%u\n",
         (unsigned)plan->si_rnti,
         (unsigned)plan->k_ssb,
         (unsigned)plan->k_ssb_norm,
         (unsigned)plan->coreset0_present,
         plan->bwp_size_rb,
         plan->bwp_start_rb_valid ? "known" : "unknown",
         (unsigned)plan->pdcch_scs_khz,
         (unsigned)plan->dlsch_scs_khz,
         (unsigned)plan->first_symbol_index,
         (unsigned)plan->monitoring_period_slots,
         (unsigned)plan->monitoring_duration_slots,
         (unsigned)plan->monitoring_active_now,
         (unsigned)plan->frame,
         (unsigned)plan->slot,
         plan->cce_reg_mapping_interleaved ? "interleaved" : "non-interleaved",
         (unsigned)plan->reg_bundle_size,
         (unsigned)plan->interleaver_size,
         (unsigned)plan->shift_index,
         (unsigned)plan->dmrs_scrambling_id);
  if (plan->bwp_start_rb_valid) {
    printf("sib1_rx_plan_detail: bwp_start_rb=%d coreset_num_rbs=%d coreset_num_symbols=%d coreset_rb_offset=%d ssb_start_sc_rel_dc=%d coreset0_sc_rel_dc=[%d,%d] sym_bitmap=0x%04x n0=%u nc=%u cand_count=%u\n",
           plan->bwp_start_rb,
           plan->coreset_num_rbs,
           plan->coreset_num_symbols,
           plan->coreset_rb_offset,
           plan->ssb_start_sc_rel_dc,
           plan->coreset0_start_sc_rel_dc,
           plan->coreset0_end_sc_rel_dc,
           (unsigned)plan->monitoring_symbol_bitmap,
           (unsigned)plan->n0,
           (unsigned)plan->nc,
           (unsigned)plan->candidate_count);
  } else {
    printf("sib1_rx_plan_detail: bwp_start_rb=unknown coreset_num_rbs=%d coreset_num_symbols=%d coreset_rb_offset=%d ssb_start_sc_rel_dc=%d coreset0_sc_rel_dc=[%d,%d] sym_bitmap=0x%04x n0=%u nc=%u cand_count=%u\n",
           plan->coreset_num_rbs,
           plan->coreset_num_symbols,
           plan->coreset_rb_offset,
           plan->ssb_start_sc_rel_dc,
           plan->coreset0_start_sc_rel_dc,
           plan->coreset0_end_sc_rel_dc,
           (unsigned)plan->monitoring_symbol_bitmap,
           (unsigned)plan->n0,
           (unsigned)plan->nc,
           (unsigned)plan->candidate_count);
  }
  for (uint8_t i = 0U; i < plan->candidate_count; i++) {
    printf("sib1_dci_cand[%u]: L=%u first_cce=%u\n",
           (unsigned)i,
           (unsigned)plan->candidates[i].aggregation_level,
           (unsigned)plan->candidates[i].first_cce);
  }
}

static int nr_toa_find_known_cell(const nr_toa_ue_t *UE, uint16_t pci, double center_freq_hz)
{
  if (!UE) {
    return -1;
  }
  for (uint32_t i = 0U; i < UE->known_cell_count; i++) {
    const nr_cell_ctx_t *cell = &UE->known_cells[i];
    if (cell->valid && cell->pci == pci &&
        fabs(cell->center_freq_hz - center_freq_hz) < 1.0) {
      return (int)i;
    }
  }
  return -1;
}

static void nr_toa_update_cell_ctx(nr_toa_ue_t *UE,
                                   const nr_iq_block_t *blk,
                                   const nr_sync_state_t *sync)
{
  int idx = -1;
  nr_cell_ctx_t *cell = NULL;
  const uint32_t scs_khz = nr_scs_khz_from_mib(&sync->mib);
  const int ssb_start_symbol_abs =
      nr_get_ssb_start_symbol_abs(scs_khz, UE ? UE->app_cfg.nr_band : -1, sync->ssb_index);

  if (!UE || !blk || !sync || !sync->locked || !sync->mib_ok) {
    return;
  }

  idx = nr_toa_find_known_cell(UE, sync->pci, blk->rx_freq_hz);
  if (idx < 0) {
    if (UE->known_cell_count < NR_TOA_MAX_SCAN_TARGETS) {
      idx = (int)UE->known_cell_count++;
    } else {
      idx = 0;
    }
  }

  cell = &UE->known_cells[idx];
  memset(cell, 0, sizeof(*cell));
  cell->valid = 1U;
  cell->center_freq_hz = blk->rx_freq_hz;
  cell->gscn = blk->rx_gscn;
  cell->nr_band = UE->app_cfg.nr_band;
  cell->pci = sync->pci;
  cell->ssb_index = sync->ssb_index;
  cell->sfn = sync->sfn;
  cell->ssb_start_symbol_abs = (ssb_start_symbol_abs >= 0) ? (uint16_t)ssb_start_symbol_abs : 0U;
  cell->ssb_start_slot = (ssb_start_symbol_abs >= 0) ? (uint16_t)(ssb_start_symbol_abs / 14) : 0U;
  cell->cfo_hz = sync->cfo_hz;
  cell->snr_db = sync->snr_db;
  cell->pss_metric = sync->pss_metric;
  cell->mib_payload = sync->mib_payload;
  cell->mib = sync->mib;
  (void)nr_type0_css_from_mib(&sync->mib,
                              sync->ssb_index,
                              sync->sfn,
                              UE->app_cfg.ssb_period_ms,
                              &cell->type0_css);
  (void)nr_sib1_rx_plan_from_cell(cell, sync->slot, &cell->sib1_rx);
  (void)nr_cell_build_timing_ref(blk, sync, cell);
  UE->serving_cell = *cell;
}

static void nr_toa_log_cell_ctx(const nr_cell_ctx_t *cell,
                                const nr_iq_block_t *blk,
                                uint16_t slot)
{
  nr_sib1_rx_plan_t live_plan;
  nr_coreset0_monitor_window_t live_win;
  nr_coreset0_grid_t live_grid;
  nr_coreset0_dmrs_obs_t live_dmrs;
  nr_pdcch_candidate_obs_t live_cand;
  nr_pdcch_dci10_si_obs_t live_dci;
  nr_sib1_pdsch_grant_t live_grant;
  const nr_sib1_rx_plan_t *plan_to_log = NULL;
  uint32_t log_frame = cell ? cell->sfn : 0U;
  uint16_t log_slot = slot;

  if (!cell || !cell->valid) {
    return;
  }

  memset(&live_plan, 0, sizeof(live_plan));
  memset(&live_win, 0, sizeof(live_win));
  memset(&live_grid, 0, sizeof(live_grid));
  memset(&live_dmrs, 0, sizeof(live_dmrs));
  memset(&live_cand, 0, sizeof(live_cand));
  memset(&live_dci, 0, sizeof(live_dci));
  if (blk && nr_sib1_rx_plan_for_block(cell, blk, &live_plan) == 0) {
    plan_to_log = &live_plan;
  } else {
    plan_to_log = &cell->sib1_rx;
  }
  if (blk && nr_type0_coreset0_monitor_window(cell, blk, &live_win) == 0 && live_win.valid) {
    nr_sib1_rx_plan_t monitor_plan;
    memset(&monitor_plan, 0, sizeof(monitor_plan));
    if (nr_sib1_rx_plan_for_frame_slot(cell, live_win.frame, live_win.slot, &monitor_plan) == 0 &&
        monitor_plan.valid) {
      live_plan = monitor_plan;
      plan_to_log = &live_plan;
    }
  }
  if (plan_to_log && plan_to_log->valid) {
    log_frame = plan_to_log->frame;
    log_slot = (uint16_t)plan_to_log->slot;
  }

  printf("cell_ctx: pci=%u band=%d freq=%.0fHz gscn=%d ssb=%u sfn=%u ssb_start_sym=%u ssb_start_slot=%u timing_ref=%u frame_start_abs=%llu scs_common=%s k_ssb=%u k_ssb_msb=%u dmrsTypeA=%s coreset0=%u searchSpace0=%u cellBarred=%u intraFreqResel=%u mib=0x%06x\n",
         (unsigned)cell->pci,
         cell->nr_band,
         cell->center_freq_hz,
         cell->gscn,
         (unsigned)cell->ssb_index,
         (unsigned)cell->sfn,
         (unsigned)cell->ssb_start_symbol_abs,
         (unsigned)cell->ssb_start_slot,
         (unsigned)cell->timing_ref_valid,
         (unsigned long long)cell->frame_start_abs_samp,
         nr_mib_scs_common_name(cell->mib.subcarrier_spacing_common),
         (unsigned)cell->mib.ssb_subcarrier_offset_full,
         (unsigned)cell->mib.ssb_subcarrier_offset_msb,
         cell->mib.dmrs_typeA_position ? "pos3" : "pos2",
         (unsigned)cell->mib.control_resource_set_zero,
         (unsigned)cell->mib.search_space_zero,
         (unsigned)cell->mib.cell_barred,
         (unsigned)cell->mib.intra_freq_reselection,
         (unsigned)cell->mib_payload);
  nr_toa_log_type0_css(&cell->type0_css, log_frame, log_slot);
  nr_toa_log_sib1_rx_plan(plan_to_log);
  if (live_win.valid) {
    printf("type0_win: frame=%u slot=%u start_samp=%u len_samp=%u first_sym=%u num_sym=%u sc_rel_dc=[%d,%d] num_rbs=%d examined_slots=%u\n",
           (unsigned)live_win.frame,
           (unsigned)live_win.slot,
           (unsigned)live_win.start_samp,
           (unsigned)live_win.len_samp,
           (unsigned)live_win.first_symbol_index,
           (unsigned)live_win.num_symbols,
           live_win.start_sc_rel_dc,
           live_win.end_sc_rel_dc,
           live_win.num_rbs,
           (unsigned)live_win.slots_examined);
  }
  if (blk && nr_type0_coreset0_extract_grid(cell, blk, &live_grid) == 0 && live_grid.valid) {
    printf("coreset0_fd: frame=%u slot=%u sym=%u rbs=%u subcarriers=%u avg_pow=%.4f peak_pow=%.4f sc_rel_dc=[%d,%d]\n",
           (unsigned)live_grid.frame,
           (unsigned)live_grid.slot,
           (unsigned)live_grid.num_symbols,
           (unsigned)live_grid.num_rbs,
           (unsigned)live_grid.num_subcarriers,
           live_grid.avg_re_power,
           live_grid.peak_re_power,
           live_grid.start_sc_rel_dc,
           live_grid.end_sc_rel_dc);
  }
  if (blk && nr_type0_coreset0_extract_dmrs(cell, blk, &live_grid, &live_dmrs) == 0 && live_dmrs.valid) {
    printf("coreset0_dmrs: frame=%u slot=%u sym=%u rbs=%u dmrs_re=%u data_re=%u mapping=%s L=%u R=%u shift=%u dmrs_id=%u avg_dmrs_pow=%.4f avg_data_pow=%.4f avg_h_pow=%.4f corr=%.4f evm=%.4f\n",
           (unsigned)live_dmrs.frame,
           (unsigned)live_dmrs.slot,
           (unsigned)live_dmrs.num_symbols,
           (unsigned)live_dmrs.num_rbs,
           (unsigned)live_dmrs.num_dmrs_re,
           (unsigned)live_dmrs.num_data_re,
           live_dmrs.cce_reg_mapping_interleaved ? "interleaved" : "non-interleaved",
           (unsigned)live_dmrs.reg_bundle_size,
           (unsigned)live_dmrs.interleaver_size,
           (unsigned)live_dmrs.shift_index,
           (unsigned)live_dmrs.dmrs_scrambling_id,
           live_dmrs.avg_dmrs_power,
           live_dmrs.avg_data_power,
           live_dmrs.avg_h_power,
           live_dmrs.corr_mag,
           live_dmrs.evm_rms);
    for (uint8_t cand_idx = 0U;
         plan_to_log && cand_idx < plan_to_log->candidate_count && cand_idx < NR_MAX_PDCCH_CANDIDATES;
         cand_idx++) {
      memset(&live_cand, 0, sizeof(live_cand));
      if (nr_type0_pdcch_extract_candidate(cell, blk, &live_grid, &live_dmrs, cand_idx, &live_cand) == 0 &&
          live_cand.valid) {
        printf("pdcch_cand_obs[%u]: L=%u first_cce=%u n_cce_total=%u regs_per_sym=%u data_re=%u avg_data_pow=%.4f avg_eq_pow=%.4f\n",
               (unsigned)cand_idx,
               (unsigned)live_cand.aggregation_level,
               (unsigned)live_cand.first_cce,
               (unsigned)live_cand.n_cce_total,
               (unsigned)live_cand.num_regs,
               (unsigned)live_cand.num_data_re,
               live_cand.avg_data_power,
               live_cand.avg_eq_power);
        memset(&live_dci, 0, sizeof(live_dci));
        if (nr_type0_pdcch_build_dci10_si_softbits(cell, &live_cand, &live_dci) == 0 && live_dci.valid) {
          printf("pdcch_dci10_si_softbits[%u]: rnti=0x%04x nid=%u E=%u A=%u freq_bits=%u mean_abs=%.4f max_abs=%.4f pos_frac=%.3f polar_decoder=sc ready=%u crc_ok=%u crc_xor=0x%06x payload=0x%010llx fda=%u tda=%u vrb2prb=%u mcs=%u rv=%u si_ind=%u reserved=0x%04x\n",
                 (unsigned)cand_idx,
                 (unsigned)live_dci.rnti,
                 (unsigned)live_dci.scrambling_id,
                 (unsigned)live_dci.encoded_bits,
                 (unsigned)live_dci.dci_payload_bits,
                 (unsigned)live_dci.freq_assignment_bits,
                 live_dci.mean_abs_llr,
                 live_dci.max_abs_llr,
                 live_dci.positive_fraction,
                 (unsigned)live_dci.decoder_ready,
                 (unsigned)live_dci.crc_ok,
                 (unsigned)live_dci.crc_xor,
                 (unsigned long long)live_dci.payload,
                 (unsigned)live_dci.frequency_domain_assignment,
                 (unsigned)live_dci.time_domain_assignment,
                 (unsigned)live_dci.vrb_to_prb_mapping,
                 (unsigned)live_dci.mcs,
                 (unsigned)live_dci.redundancy_version,
                 (unsigned)live_dci.system_info_indicator,
                 (unsigned)live_dci.reserved_bits);
          memset(&live_grant, 0, sizeof(live_grant));
          if (nr_sib1_pdsch_grant_from_dci(cell, &live_dci, &live_grant) == 0 &&
              live_grant.valid) {
            printf("sib1_pdsch_grant[%u]: frame=%u slot=%u pdsch_frame=%u pdsch_slot=%u k0=%u mapping=%s start_sym=%u n_sym=%u bwp_start=%u bwp_size=%u rb_start=%u rb_size=%u rb_start_abs=%u mcs=%u rv=%u si_ind=%u vrb2prb=%u dmrsTypeApos=%u mux=%u dlsch_scs=%ukHz\n",
                   (unsigned)cand_idx,
                   (unsigned)live_grant.frame,
                   (unsigned)live_grant.slot,
                   (unsigned)live_grant.pdsch_frame,
                   (unsigned)live_grant.pdsch_slot,
                   (unsigned)live_grant.k0,
                   live_grant.mapping_type_a ? "A" : "B",
                   (unsigned)live_grant.start_symbol,
                   (unsigned)live_grant.num_symbols,
                   (unsigned)live_grant.bwp_start_rb,
                   (unsigned)live_grant.bwp_size_rb,
                   (unsigned)live_grant.rb_start,
                   (unsigned)live_grant.rb_size,
                   (unsigned)live_grant.rb_start_abs,
                   (unsigned)live_grant.mcs,
                   (unsigned)live_grant.redundancy_version,
                   (unsigned)live_grant.system_info_indicator,
                   (unsigned)live_grant.vrb_to_prb_mapping,
                   (unsigned)live_grant.dmrs_typeA_position,
                   (unsigned)live_grant.mux_pattern,
                   (unsigned)live_grant.dlsch_scs_khz);
          }
        }
      }
    }
  }
}

static void nr_rel_delay_logger_close(void)
{
  if (g_rel_delay_logger.fp) {
    fclose(g_rel_delay_logger.fp);
    g_rel_delay_logger.fp = NULL;
  }
}

static int nr_rel_delay_logger_init(void)
{
  const char *env_path = getenv("NR_TOA_DELAY_CSV");
  time_t now = 0;
  struct tm tm_now;

  if (g_rel_delay_logger.fp || g_rel_delay_logger.enabled) {
    return 0;
  }

  memset(&g_rel_delay_logger, 0, sizeof(g_rel_delay_logger));
  g_rel_delay_logger.enabled = 1U;

  if (env_path && env_path[0] != '\0') {
    snprintf(g_rel_delay_logger.path, sizeof(g_rel_delay_logger.path), "%s", env_path);
  } else {
    now = time(NULL);
    localtime_r(&now, &tm_now);
    snprintf(g_rel_delay_logger.path, sizeof(g_rel_delay_logger.path),
             "/tmp/nr_ssb_relative_delay_%04d%02d%02d_%02d%02d%02d.csv",
             tm_now.tm_year + 1900, tm_now.tm_mon + 1, tm_now.tm_mday,
             tm_now.tm_hour, tm_now.tm_min, tm_now.tm_sec);
  }

  g_rel_delay_logger.fp = fopen(g_rel_delay_logger.path, "w");
  if (!g_rel_delay_logger.fp) {
    g_rel_delay_logger.enabled = 0U;
    return -1;
  }

  fprintf(g_rel_delay_logger.fp,
          "row,ts_first,rx_ts_int,rx_ts_frac,rel_samp,rel_ns,rel_us,ma_ns,jitter_ns,fs_hz,pci,ssb,snr_db,peak_metric,quality,cfo_hz,coarse_offset_samp,cum_tracking_shift_samp,mib_payload\n");
  fflush(g_rel_delay_logger.fp);
  printf("relative_delay_csv: path=%s\n", g_rel_delay_logger.path);
  return 0;
}

static void nr_rel_delay_logger_push(const nr_iq_block_t *blk,
                                     const nr_sync_state_t *sync,
                                     const nr_toa_meas_t *meas)
{
  double rel_samp = 0.0;
  double rel_ns = 0.0;
  double sum = 0.0;
  double mean = 0.0;
  double var = 0.0;
  double jitter_ns = 0.0;

  if (!blk || !sync || !meas) {
    return;
  }
  if (nr_rel_delay_logger_init() != 0 || !g_rel_delay_logger.fp) {
    return;
  }

  rel_samp = ((double)((int64_t)meas->rx_ts_int - (int64_t)blk->ts_first)) + meas->rx_ts_frac;
  rel_ns = rel_samp * 1.0e9 / ((meas->fs_hz > 0.0) ? meas->fs_hz : 30.72e6);

  g_rel_delay_logger.win_ns[g_rel_delay_logger.win_head] = rel_ns;
  g_rel_delay_logger.win_head =
      (g_rel_delay_logger.win_head + 1U) % (uint32_t)(sizeof(g_rel_delay_logger.win_ns) / sizeof(g_rel_delay_logger.win_ns[0]));
  if (g_rel_delay_logger.win_count < (sizeof(g_rel_delay_logger.win_ns) / sizeof(g_rel_delay_logger.win_ns[0]))) {
    g_rel_delay_logger.win_count++;
  }

  for (uint32_t i = 0U; i < g_rel_delay_logger.win_count; i++) {
    sum += g_rel_delay_logger.win_ns[i];
  }
  mean = sum / (double)(g_rel_delay_logger.win_count ? g_rel_delay_logger.win_count : 1U);
  for (uint32_t i = 0U; i < g_rel_delay_logger.win_count; i++) {
    const double d = g_rel_delay_logger.win_ns[i] - mean;
    var += d * d;
  }
  jitter_ns = sqrt(var / (double)(g_rel_delay_logger.win_count ? g_rel_delay_logger.win_count : 1U));

  g_rel_delay_logger.rows++;
  fprintf(g_rel_delay_logger.fp,
          "%u,%llu,%llu,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.0f,%u,%u,%.2f,%.6f,%.6f,%.2f,%d,%lld,0x%06x\n",
          g_rel_delay_logger.rows,
          (unsigned long long)blk->ts_first,
          (unsigned long long)meas->rx_ts_int,
          meas->rx_ts_frac,
          rel_samp,
          rel_ns,
          rel_ns / 1000.0,
          mean,
          jitter_ns,
          meas->fs_hz,
          (unsigned)sync->pci,
          (unsigned)sync->ssb_index,
          sync->snr_db,
          meas->peak_metric,
          meas->quality,
          sync->cfo_hz,
          sync->coarse_offset_samp,
          (long long)sync->cum_tracking_shift_samp,
          (unsigned)sync->mib_payload);
  fflush(g_rel_delay_logger.fp);

  if (g_rel_delay_logger.rows <= 5U || (g_rel_delay_logger.rows % 20U) == 0U) {
    printf("relative_delay: row=%u rel_ns=%.2f ma_ns=%.2f jitter_ns=%.2f pci=%u ssb=%u snr=%.2f peak=%.3f\n",
           g_rel_delay_logger.rows,
           rel_ns,
           mean,
           jitter_ns,
           (unsigned)sync->pci,
           (unsigned)sync->ssb_index,
           sync->snr_db,
           meas->peak_metric);
  }
}

static void nr_toa_sig_handler(int signo)
{
  (void)signo;
  oai_exit = 1;
}

static void nr_toa_process_solver_meas(PHY_VARS_NR_TOA_UE *UE,
                                       const nr_toa_meas_t *meas,
                                       uint64_t meas_job_id);

typedef struct {
  int valid;
  uint64_t ssb_ref_abs;
  uint64_t predicted_abs;
  uint64_t period_samps;
  uint32_t ssb_need;
  int32_t predicted_offset;
} nr_toa_ssb_pred_diag_t;

static int64_t nr_toa_floor_div_i64(int64_t num, int64_t den)
{
  int64_t q = 0;
  int64_t r = 0;

  if (den <= 0) {
    return 0;
  }

  q = num / den;
  r = num % den;
  if (r != 0 && ((r > 0) != (den > 0))) {
    q--;
  }
  return q;
}

static int nr_toa_predict_ssb_offset_from_abs_time(const PHY_VARS_NR_TOA_UE *UE,
                                                   const nr_iq_block_t *blk,
                                                   nr_sync_state_t *sync,
                                                   nr_toa_ssb_pred_diag_t *diag)
{
  const nr_cell_ctx_t *cell = UE ? &UE->serving_cell : NULL;
  const double fs_hz = (blk && blk->fs_hz > 0.0) ? blk->fs_hz :
      (UE ? UE->app_cfg.sample_rate_hz : 0.0);
  const uint32_t scs_khz =
      (cell && cell->type0_css.valid && cell->type0_css.scs_pdcch_khz > 0U)
          ? cell->type0_css.scs_pdcch_khz
          : nr_scs_khz_from_mib(cell ? &cell->mib : NULL);
  const uint32_t ssb_period_ms =
      (UE && UE->app_cfg.ssb_period_ms > 0U) ? UE->app_cfg.ssb_period_ms : 20U;
  const uint64_t period_samps = (uint64_t)llround(fs_hz * (double)ssb_period_ms / 1000.0);
  const uint32_t ssb_need = nr_v0_ssb_burst_len_fs(fs_hz);
  uint64_t ssb_symbol_offset_in_slot = 0U;
  uint64_t ssb_ref_abs = 0U;
  uint64_t candidate_abs = 0U;
  uint32_t frame = 0U;
  uint32_t slot = 0U;

  if (diag) {
    memset(diag, 0, sizeof(*diag));
  }
  if (!UE || !blk || !sync || !cell || !cell->valid || !cell->timing_ref_valid) {
    return -1;
  }
  if (fs_hz <= 0.0 || period_samps == 0U || ssb_need == 0U ||
      blk->nsamps < ssb_need || sync->pci != cell->pci) {
    return -1;
  }
  if (scs_khz == 0U) {
    return -1;
  }

  const uint32_t ssb_symbol_in_slot = (uint32_t)(cell->ssb_start_symbol_abs % 14U);
  for (uint32_t sym = 0U; sym < ssb_symbol_in_slot; sym++) {
    ssb_symbol_offset_in_slot +=
        nr_ofdm_symbol_len_fs(fs_hz, scs_khz, (uint32_t)cell->ssb_start_slot * 14U + sym);
  }

  ssb_ref_abs =
      cell->frame_start_abs_samp +
      nr_slot_start_samp_in_frame_fs(fs_hz, scs_khz, cell->ssb_start_slot) +
      ssb_symbol_offset_in_slot;
  if (diag) {
    diag->ssb_ref_abs = ssb_ref_abs;
    diag->period_samps = period_samps;
    diag->ssb_need = ssb_need;
  }

  const uint64_t blk_start = blk->abs_samp0;
  const uint64_t blk_end = blk->abs_samp0 + blk->nsamps;
  int64_t k = nr_toa_floor_div_i64((int64_t)blk_start - (int64_t)ssb_ref_abs,
                                   (int64_t)period_samps);
  int64_t cand = (int64_t)ssb_ref_abs + k * (int64_t)period_samps;

  while (cand < (int64_t)blk_start) {
    cand += (int64_t)period_samps;
  }
  if (cand < 0 || (uint64_t)cand + ssb_need > blk_end) {
    return -1;
  }

  candidate_abs = (uint64_t)cand;
  if (candidate_abs < blk_start ||
      candidate_abs - blk_start > (uint64_t)INT32_MAX) {
    return -1;
  }

  sync->coarse_offset_samp = (int32_t)(candidate_abs - blk_start);
  sync->cum_tracking_shift_samp = 0;
  if (diag) {
    diag->valid = 1;
    diag->predicted_abs = candidate_abs;
    diag->predicted_offset = sync->coarse_offset_samp;
  }
  if (nr_cell_estimate_frame_slot_from_samp(cell, candidate_abs, &frame, &slot, NULL) == 0) {
    sync->sfn = frame;
    sync->slot = (uint16_t)slot;
  }
  return 0;
}

static void nr_toa_process_measure_block(PHY_VARS_NR_TOA_UE *UE,
                                         const nr_iq_block_t *blk,
                                         nr_sync_state_t *local_sync,
                                         uint64_t job_id)
{
  if (!UE || !blk || !local_sync) {
    return;
  }

  nr_toa_ssb_pred_diag_t pred_diag;
  const int pred_rc =
      nr_toa_predict_ssb_offset_from_abs_time(UE, blk, local_sync, &pred_diag);
  const int32_t pred_offset = local_sync->coarse_offset_samp;
  const float cfo_before = local_sync->cfo_hz;

  if (UE->provider && UE->provider->track) {
    (void)UE->provider->track(UE->provider_ctx, blk, local_sync);
  }
  if (nr_ssb_check_lost_lock(local_sync) != 0) {
    local_sync->locked = 0;
  }

  nr_toa_meas_t meas;
  memset(&meas, 0, sizeof(meas));
  if (UE->provider && UE->provider->extract_meas) {
    (void)UE->provider->extract_meas(UE->provider_ctx, blk, local_sync, &meas);
  }
  if (meas.fs_hz > 0.0 && local_sync->locked && local_sync->mib_ok) {
    nr_rel_delay_logger_push(blk, local_sync, &meas);
  }
  if ((job_id % 20U) == 0U || pred_rc != 0 || !local_sync->locked) {
    printf("track_diag: job=%llu pred=%s pred_offset=%d blk_abs=%llu nsamps=%u period=%llu ssb_need=%u after_offset=%d shift=%lld cfo_before=%.2f cfo_after=%.2f snr=%.2f metric=%.3f locked=%u mib=%u\n",
           (unsigned long long)job_id,
           pred_rc == 0 ? "ok" : "fallback",
           pred_offset,
           (unsigned long long)blk->abs_samp0,
           blk->nsamps,
           (unsigned long long)pred_diag.period_samps,
           pred_diag.ssb_need,
           local_sync->coarse_offset_samp,
           (long long)local_sync->cum_tracking_shift_samp,
           cfo_before,
           local_sync->cfo_hz,
           local_sync->snr_db,
           local_sync->pss_metric,
           (unsigned)local_sync->locked,
           (unsigned)local_sync->mib_ok);
  }
  if (meas.fs_hz > 0.0 && ((job_id % 20U) == 0U || meas.valid)) {
    printf("toa_meas_diag: job=%llu valid=%u pci=%u ssb=%u sfn=%u slot=%u rx_ts=%lld+%.3f tx_ts=%lld toa_ns=%.3f range_m=%.3f quality=%.3f cfo=%.2f\n",
           (unsigned long long)job_id,
           (unsigned)meas.valid,
           (unsigned)meas.pci,
           (unsigned)meas.ssb_index,
           (unsigned)meas.sfn,
           (unsigned)meas.slot,
           (long long)meas.rx_ts_int,
           meas.rx_ts_frac,
           (long long)meas.tx_ts_int,
           meas.toa_ns,
           meas.range_m,
           meas.quality,
           local_sync->cfo_hz);
  }
  nr_toa_process_solver_meas(UE, &meas, job_id);
}

static void *sync_actor_thread(void *arg)
{
  PHY_VARS_NR_TOA_UE *UE = (PHY_VARS_NR_TOA_UE *)arg;
  uint64_t sync_cnt = 0;
  uint64_t detect_total = 0;
  uint64_t nodetect_total = 0;
  uint32_t miss_streak = 0;
  uint8_t prev_locked = 0;
  uint16_t last_acq_pci = 1008U;
  uint64_t acq_pci_change_count = 0;

  while (!oai_exit) {
    nr_iq_block_t *blk = NULL;
    uint64_t sync_job_id = 0;

    pthread_mutex_lock(&UE->sync_mtx);
    while (!oai_exit && UE->sync_q_count == 0U) {
      pthread_cond_wait(&UE->sync_cv, &UE->sync_mtx);
    }
    if (oai_exit) {
      pthread_mutex_unlock(&UE->sync_mtx);
      break;
    }

    blk = UE->sync_q[UE->sync_q_head];
    sync_job_id = UE->sync_q_job_id[UE->sync_q_head];
    UE->sync_q[UE->sync_q_head] = NULL;
    UE->sync_q_head = (UE->sync_q_head + 1U) % (uint32_t)NR_SYNC_Q_DEPTH;
    UE->sync_q_count--;
    UE->sync_job_id = sync_job_id;
    UE->sync_job_blk = blk;
    UE->sync_job_pending = (UE->sync_q_count > 0U) ? 1 : 0;
    nr_sync_state_t local = UE->sync;
    pthread_mutex_unlock(&UE->sync_mtx);

    const int has_real_lock = (local.locked &&
                               local.pbch_ok &&
                               local.pbch_confirmed &&
                               local.mib_ok);
    if (blk && blk->rx[0] && (sync_cnt % 50U) == 0U) {
      double p = 0.0;
      double peak = 0.0;
      for (uint32_t n = 0; n < blk->nsamps; n++) {
        double i = (double)blk->rx[0][n].r;
        double q = (double)blk->rx[0][n].i;
        double e = i * i + q * q;
        p += e;
        if (e > peak) {
          peak = e;
        }
      }
      p /= (double)(blk->nsamps ? blk->nsamps : 1U);
      printf("sync_actor: ts=%llu nsamps=%u avg_pwr=%.1f peak=%.1f\n",
             (unsigned long long)blk->ts_first, blk->nsamps, p, peak);
    }
    if (has_real_lock) {
      nr_toa_process_measure_block(UE, blk, &local, sync_job_id);
      if (!local.locked) {
        miss_streak++;
      } else {
        miss_streak = 0;
      }
    } else if (UE->provider && UE->provider->acquire) {
      int rc = UE->provider->acquire(UE->provider_ctx, blk, &local);
      if (rc != 0) {
        nodetect_total++;
        miss_streak++;
        if (miss_streak >= NR_SYNC_LOST_MISS_THRESH) {
          local.locked = 0;
        }
        if ((nodetect_total % 10U) == 0U) {
          printf("acquire_diag: rc=%d no_detect=%llu miss_streak=%u pci=%u hyp=%u pbch=%u pbch_confirmed=%u mib=%u offset=%d cfo=%.2f snr=%.2f metric=%.3f blk_abs=%llu nsamps=%u\n",
                 rc,
                 (unsigned long long)nodetect_total,
                 (unsigned)miss_streak,
                 (unsigned)local.pci,
                 (unsigned)local.pci_hyp_count,
                 (unsigned)local.pbch_ok,
                 (unsigned)local.pbch_confirmed,
                 (unsigned)local.mib_ok,
                 local.coarse_offset_samp,
                 local.cfo_hz,
                 local.snr_db,
                 local.pss_metric,
                 (unsigned long long)(blk ? blk->abs_samp0 : 0ULL),
                 blk ? blk->nsamps : 0U);
        }
      } else {
        detect_total++;
        miss_streak = 0;
        if (last_acq_pci != 1008U && last_acq_pci != local.pci) {
          acq_pci_change_count++;
        }
        last_acq_pci = local.pci;
        const int true_lock_after_acq = (local.locked &&
                                         local.pbch_ok &&
                                         local.pbch_confirmed &&
                                         local.mib_ok);
        printf("acquire_diag: rc=%d true_lock=%d pci=%u ssb=%u hyp=%u pci_changes=%llu pbch=%u pbch_confirmed=%u mib=%u sfn=%u offset=%d cfo=%.2f snr=%.2f metric=%.3f blk_abs=%llu nsamps=%u\n",
               rc,
               true_lock_after_acq,
               (unsigned)local.pci,
               (unsigned)local.ssb_index,
               (unsigned)local.pci_hyp_count,
               (unsigned long long)acq_pci_change_count,
               (unsigned)local.pbch_ok,
               (unsigned)local.pbch_confirmed,
               (unsigned)local.mib_ok,
               (unsigned)local.sfn,
               local.coarse_offset_samp,
               local.cfo_hz,
               local.snr_db,
               local.pss_metric,
               (unsigned long long)(blk ? blk->abs_samp0 : 0ULL),
               blk ? blk->nsamps : 0U);
        if (local.pci_hyp_count > 1U) {
          printf("acquire_pci_hyp: count=%u h0=%u/%.3f h1=%u/%.3f h2=%u/%.3f h3=%u/%.3f\n",
                 (unsigned)local.pci_hyp_count,
                 (unsigned)local.pci_hyp[0], local.pci_hyp_metric[0],
                 (unsigned)local.pci_hyp[1], local.pci_hyp_metric[1],
                 (unsigned)local.pci_hyp_count > 2U ? (unsigned)local.pci_hyp[2] : 1008U,
                 local.pci_hyp_count > 2U ? local.pci_hyp_metric[2] : 0.0f,
                 (unsigned)local.pci_hyp_count > 3U ? (unsigned)local.pci_hyp[3] : 1008U,
                 local.pci_hyp_count > 3U ? local.pci_hyp_metric[3] : 0.0f);
        }
        if (!true_lock_after_acq) {
          printf("lock_reject: acquire returned success without PBCH/MIB confirmation pci=%u pbch=%u pbch_confirmed=%u mib=%u\n",
                 (unsigned)local.pci,
                 (unsigned)local.pbch_ok,
                 (unsigned)local.pbch_confirmed,
                 (unsigned)local.mib_ok);
          local.locked = 0;
        }
      }
    }
    if (local.locked && local.mib_ok) {
      nr_toa_update_cell_ctx(UE, blk, &local);
    }
    const int real_lock_now = (local.locked &&
                               local.pbch_ok &&
                               local.pbch_confirmed &&
                               local.mib_ok);
    if (!prev_locked && real_lock_now) {
      printf("SSB_LOCK_EVENT: pci=%u freq=%.0fHz gscn=%d target_pci=%d offset=%d cfo=%.2f snr=%.2f metric=%.3f pbch=%u pbch_confirmed=%u mib=%u sfn=%u mib_payload=0x%06x det=%llu nodet=%llu\n",
             (unsigned)local.pci,
             blk ? blk->rx_freq_hz : UE->current_rx_freq_hz,
             blk ? blk->rx_gscn : UE->current_rx_gscn,
             blk ? blk->target_pci : UE->current_target_pci,
             local.coarse_offset_samp,
             local.cfo_hz,
             local.snr_db,
             local.pss_metric,
             (unsigned)local.pbch_ok,
             (unsigned)local.pbch_confirmed,
             (unsigned)local.mib_ok,
             (unsigned)local.sfn, (unsigned)local.mib_payload,
             (unsigned long long)detect_total,
             (unsigned long long)nodetect_total);
      nr_toa_log_cell_ctx(&UE->serving_cell, blk, local.slot);
    }
    prev_locked = (uint8_t)real_lock_now;
    if ((sync_cnt % 50U) == 0U) {
      printf("sync_result: %s ssb_locked=%u real_lock=%u pbch=%u pbch_confirmed=%u mib=%u pci=%u ssb=%u sfn=%u mib_payload=0x%06x freq=%.0fHz gscn=%d target_pci=%d offset=%d shift=%lld cfo=%.2f snr=%.2f metric=%.3f miss_streak=%u det=%llu nodet=%llu dropped_sync=%llu rx_gaps=%llu ring_overrun=%u pool_fallback=%u\n",
             real_lock_now ? "locked" : "not-locked",
             (unsigned)local.locked,
             (unsigned)real_lock_now,
             (unsigned)local.pbch_ok,
             (unsigned)local.pbch_confirmed,
             (unsigned)local.mib_ok, (unsigned)local.pci,
             (unsigned)local.ssb_index, (unsigned)local.sfn,
             (unsigned)local.mib_payload,
             blk ? blk->rx_freq_hz : UE->current_rx_freq_hz,
             blk ? blk->rx_gscn : UE->current_rx_gscn,
             blk ? blk->target_pci : UE->current_target_pci,
             local.coarse_offset_samp,
             (long long)local.cum_tracking_shift_samp,
             local.cfo_hz, local.snr_db, local.pss_metric, (unsigned)miss_streak,
             (unsigned long long)detect_total, (unsigned long long)nodetect_total,
             (unsigned long long)UE->sync_jobs_dropped,
             (unsigned long long)UE->rx_gap_count,
             UE->iq_ring.overrun_cnt,
             UE->iq_ring.alloc_fallback_cnt);
      if (UE->serving_cell.valid) {
        nr_toa_log_cell_ctx(&UE->serving_cell, blk, local.slot);
      }
    }
    sync_cnt++;

    pthread_mutex_lock(&UE->sync_mtx);
    UE->sync = local;
    UE->sync_job_blk = NULL;
    UE->sync_job_done = 1;
    pthread_cond_signal(&UE->sync_cv);
    pthread_mutex_unlock(&UE->sync_mtx);

    nr_iq_block_put(blk);
  }

  return NULL;
}

static void nr_toa_process_solver_meas(PHY_VARS_NR_TOA_UE *UE,
                                       const nr_toa_meas_t *meas,
                                       uint64_t meas_job_id)
{
  if (!UE || !meas) {
    return;
  }

  (void)nr_epoch_mgr_push(&UE->epoch_mgr, meas);
  nr_toa_epoch_t epoch;
  memset(&epoch, 0, sizeof(epoch));
  if (nr_epoch_mgr_pop_ready(&UE->epoch_mgr, &epoch) == 0) {
    nr_solver_input_t in;
    nr_loc_solution_t sol;
    (void)nr_pos_build_equations(&epoch, &in);
    (void)nr_pos_solve_wls(&in, &sol);
    (void)nr_pos_validate_solution(&sol);
    (void)nr_trace_solution(&sol);
    if (sol.valid) {
      printf("sync_actor: solver job=%llu epoch=%llu num_meas=%u sol.valid=%u\n",
             (unsigned long long)meas_job_id,
             (unsigned long long)epoch.epoch_id,
             (unsigned)epoch.num_meas,
             (unsigned)sol.valid);
    }
  }
}

static void *rx_control_thread(void *arg)
{
  PHY_VARS_NR_TOA_UE *UE = (PHY_VARS_NR_TOA_UE *)arg;

  const uint32_t ssb_need = nr_v0_ssb_burst_len_fs(UE->app_cfg.sample_rate_hz);
  const uint32_t ssb_period_samps = (UE->app_cfg.ssb_period_ms > 0)
      ? (uint32_t)(UE->app_cfg.sample_rate_hz * (double)UE->app_cfg.ssb_period_ms / 1000.0)
      : 0U;
  const uint32_t min_capture = (ssb_period_samps > 0)
      ? (ssb_period_samps + ssb_need) : (2U * ssb_need);
  const uint32_t nsamps = (min_capture > 4096U) ? min_capture : 4096U;
  const uint8_t rx_ant = (UE->app_cfg.rx_ant > 0) ? (uint8_t)UE->app_cfg.rx_ant : 1U;
  UE->samples_per_slot = nsamps;
  UE->samples_per_frame = 2 * nsamps;
  UE->abs_samp_wr = 0;
  UE->current_rx_freq_hz = UE->app_cfg.center_freq_hz;
  UE->current_rx_gscn = -1;
  UE->current_target_pci = UE->app_cfg.target_pci;
  UE->current_scan_target_idx = 0U;
  UE->rf_settle_reads = NR_RF_SETTLE_READS;
  UE->rx_read_count = 0;
  UE->rx_gap_count = 0;
  UE->rx_next_abs_samp = 0;

  printf("rx_control: capture_plan fs=%.0fHz ssb_period_ms=%u ssb_period_samps=%u ssb_need=%u nsamps=%u coverage=%s\n",
         UE->app_cfg.sample_rate_hz,
         (unsigned)UE->app_cfg.ssb_period_ms,
         ssb_period_samps,
         ssb_need,
         nsamps,
         (nsamps >= min_capture) ? "ok" : "short");

  if (nr_iq_ring_prealloc(&UE->iq_ring, nsamps, rx_ant) != 0) {
    printf("rx_control: failed to preallocate IQ block pool nsamps=%u rx_ant=%u depth=%d\n",
           nsamps,
           (unsigned)rx_ant,
           UE->iq_ring.depth);
    oai_exit = 1;
    return NULL;
  }
  printf("rx_control: preallocated IQ block pool depth=%d nsamps=%u rx_ant=%u\n",
         UE->iq_ring.depth,
         nsamps,
         (unsigned)rx_ant);

  while (!oai_exit) {
    if (UE->state == TOA_STATE_WAIT_CLOCK) {
      if (nr_toa_clock_ready(UE) == 1) {
        UE->state = TOA_STATE_PRESYNC;
      } else {
        usleep(1000);
      }
      continue;
    }

    pthread_mutex_lock(&UE->sync_mtx);
    const int has_real_lock = (UE->sync.locked &&
                               UE->sync.pbch_ok &&
                               UE->sync.pbch_confirmed &&
                               UE->sync.mib_ok);
    pthread_mutex_unlock(&UE->sync_mtx);

    if (has_real_lock) {
      UE->state = TOA_STATE_LOCKED;
      (void)nr_toa_read_one_slot(UE, &UE->iq_ring);
    } else {
      UE->state = TOA_STATE_PRESYNC;
      (void)nr_toa_read_two_frames(UE, &UE->iq_ring);
    }
  }

  (void)TOA_THREAD_RX_CONTROL;
  return NULL;
}

static void nr_toa_print_usage(const char *prog)
{
  fprintf(stderr,
          "Usage: %s [config.conf]\n"
          "       %s -O config.conf\n"
          "       %s --config config.conf\n",
          prog ? prog : "nr-toa-uesoftmodem",
          prog ? prog : "nr-toa-uesoftmodem",
          prog ? prog : "nr-toa-uesoftmodem");
}

static const char *nr_toa_parse_cfgpath(int argc, char **argv)
{
  const char *default_cfg = "targets/PROJECTS/NR-TOA/CONF/ue.toa.ssb.usrpb210.conf";

  if (argc <= 1) {
    return default_cfg;
  }
  if (strcmp(argv[1], "-h") == 0 || strcmp(argv[1], "--help") == 0) {
    nr_toa_print_usage(argv[0]);
    return NULL;
  }
  if (strcmp(argv[1], "-O") == 0 || strcmp(argv[1], "--config") == 0) {
    if (argc < 3 || argv[2][0] == '\0') {
      fprintf(stderr, "NR-TOA error: missing config path after %s\n", argv[1]);
      nr_toa_print_usage(argv[0]);
      return NULL;
    }
    return argv[2];
  }
  return argv[1];
}

int main(int argc, char **argv)
{
  PHY_VARS_NR_TOA_UE ue;
  memset(&ue, 0, sizeof(ue));
  oai_exit = 0;
  ue.state = TOA_STATE_WAIT_CLOCK;
  ue.provider_ctx = (void *)&ue;

  const char *cfgpath = nr_toa_parse_cfgpath(argc, argv);
  if (!cfgpath) {
    return 1;
  }
  if (nr_toa_load_config(cfgpath, &ue.app_cfg) != 0) {
    fprintf(stderr, "NR-TOA error: failed to load config '%s'\n", cfgpath);
    return 1;
  }
  ue.provider = nr_toa_select_provider(&ue.app_cfg);
  if (!ue.provider) {
    fprintf(stderr, "NR-TOA error: no provider selected\n");
    return 1;
  }
  ue.current_target_pci = ue.app_cfg.target_pci;
  ue.current_scan_target_idx = 0U;
  {
    uint32_t ssb_scs_khz = ue.app_cfg.ssb_scs_khz;
    if (ssb_scs_khz != 15U && ssb_scs_khz != 30U) {
      ssb_scs_khz = nr_v0_default_ssb_scs_khz(ue.app_cfg.center_freq_hz);
    }
    nr_v0_set_ssb_scs_khz(ssb_scs_khz);
    nr_v0_set_ssb_abs_start_symbol(
        nr_get_ssb_start_symbol_abs(ssb_scs_khz, ue.app_cfg.nr_band, 0U));
  }

  if (ue.app_cfg.anchor_db_path[0] != '\0') {
    int n = 0;
    if (nr_toa_load_anchor_db(ue.app_cfg.anchor_db_path, ue.anchors, &n) == 0) {
      ue.n_anchors = n;
    }
  }

  printf("NR-TOA UE cfg: sdr=%s provider=%s clock=%s time=%s band=%d f=%0.f Fs=%0.f rx_gain=%0.1f tx_gain=%0.1f mode=%u meas_mode=%u full_band_sweep=%u strict_center_freq=%u gain_sweep=%u target_pci=%d ssb_scs=%ukHz anchor_db_path=%s n_anchors=%d\n",
         ue.app_cfg.sdr,
         ue.provider->name,
         ue.app_cfg.clock_source,
         ue.app_cfg.time_source,
         ue.app_cfg.nr_band,
         ue.app_cfg.center_freq_hz,
         ue.app_cfg.sample_rate_hz,
         ue.app_cfg.rx_gain_db,
         ue.app_cfg.tx_gain_db,
         (unsigned)ue.app_cfg.mode,
         (unsigned)ue.app_cfg.meas_mode,
         (unsigned)ue.app_cfg.full_band_sweep,
         (unsigned)ue.app_cfg.strict_center_freq,
         (unsigned)ue.app_cfg.gain_sweep_enable,
         ue.app_cfg.target_pci,
         (unsigned)nr_v0_get_ssb_scs_khz(),
         ue.app_cfg.anchor_db_path,
         ue.n_anchors);
  if (ue.app_cfg.scan_target_count > 0U) {
    printf("NR-TOA note: using %u configured scan target(s) instead of wide-band sweep.\n",
           ue.app_cfg.scan_target_count);
    for (uint32_t i = 0U; i < ue.app_cfg.scan_target_count; i++) {
      printf("  scan_target[%u]: freq=%.0fHz target_pci=%d\n",
             i,
             ue.app_cfg.scan_targets[i].center_freq_hz,
             ue.app_cfg.scan_targets[i].target_pci);
    }
  }
  if (ue.app_cfg.mode == NR_TOA_MODE_SSB_TOA) {
    printf("NR-TOA note: SSB mode now attempts real PBCH/MIB decode; treat ssb_locked=1 with mib=0 as a decode failure, not a successful cell accept.\n");
    printf("NR-TOA note: structured MIB context now derives a live FR1 Type0-PDCCH/SIB1 skeleton (k_SSB/CORESET0/SearchSpace0 + monitoring window).\n");
  }
  if (ue.app_cfg.iq_dump_enable) {
    const char *dump_dir = getenv("NR_TOA_IQ_DUMP_DIR");
    printf("NR-TOA note: IQ near-miss dump enabled, dir=%s\n",
           (dump_dir && dump_dir[0] != '\0') ? dump_dir : "/tmp/nr_toa_iq");
  }
  if (nr_rel_delay_logger_init() != 0) {
    printf("NR-TOA note: relative delay CSV logger init failed; continuing without CSV output.\n");
  }
  if (nr_toa_build_rf_cfg(&ue.app_cfg, &ue.rf_cfg) != 0) {
    return 1;
  }
  if (nr_toa_radio_init(&ue.dev, &ue.rf_cfg) != 0) {
    return 1;
  }
  if (nr_toa_radio_start(ue.dev) != 0) {
    return 1;
  }
  if (nr_iq_ring_init(&ue.iq_ring, 64) != 0) {
    return 1;
  }
  if (ue.provider->init(ue.provider_ctx) != 0) {
    return 1;
  }

  signal(SIGINT, nr_toa_sig_handler);
  signal(SIGTERM, nr_toa_sig_handler);

  /* Initialize actor/job synchronization primitives. */
  if (pthread_mutex_init(&ue.sync_mtx, NULL) != 0) {
    return 1;
  }
  pthread_cond_init(&ue.sync_cv, NULL);
  ue.sync_job_pending = 0;
  ue.sync_job_done = 0;
  ue.sync_job_blk = NULL;
  ue.sync_job_id = 0;
  ue.sync_q_head = 0;
  ue.sync_q_tail = 0;
  ue.sync_q_count = 0;
  memset(ue.sync_q, 0, sizeof(ue.sync_q));
  memset(ue.sync_q_job_id, 0, sizeof(ue.sync_q_job_id));
  ue.next_sync_job_id = 1;
  ue.sync_jobs_dropped = 0;

  /* Start actor threads. */
  if (pthread_create(&ue.sync_tid, NULL, sync_actor_thread, &ue) != 0) {
    return 1;
  }

  /* Start the RX/control loop. */
  pthread_t tid;
  if (pthread_create(&tid, NULL, rx_control_thread, &ue) != 0) {
    return 1;
  }
  pthread_join(tid, NULL);

  /* Make sure actors are not stuck in cond waits. */
  pthread_mutex_lock(&ue.sync_mtx);
  ue.sync_job_pending = 0;
  while (ue.sync_q_count > 0U) {
    nr_iq_block_t *old = ue.sync_q[ue.sync_q_head];
    ue.sync_q[ue.sync_q_head] = NULL;
    ue.sync_q_head = (ue.sync_q_head + 1U) % (uint32_t)NR_SYNC_Q_DEPTH;
    ue.sync_q_count--;
    if (old) {
      nr_iq_block_put(old);
    }
  }
  pthread_cond_broadcast(&ue.sync_cv);
  pthread_mutex_unlock(&ue.sync_mtx);

  pthread_join(ue.sync_tid, NULL);
  nr_rel_delay_logger_close();

  nr_iq_ring_free(&ue.iq_ring);
  return 0;
}
