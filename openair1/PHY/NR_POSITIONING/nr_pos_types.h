#pragma once

#include "radio/COMMON/common_lib.h"

#include <pthread.h>
#include <stdint.h>

#define NR_TOA_MAX_RX_ANT 4
#define NR_TOA_MAX_ANCHORS 8
#define NR_TOA_MAX_MEAS_PER_EP 16
#define NR_TOA_MAX_SCAN_TARGETS 16
#define NR_SSB_RE_ROWS 4
#define NR_SSB_RE_COLS 240
#define NR_PBCH_LLR_LEN 864
#define NR_MAX_PCI_HYPS 16
#define NR_MAX_PDCCH_CANDIDATES 8
#define NR_CORESET0_MAX_SYMBOLS 3
#define NR_CORESET0_MAX_RB 96
#define NR_CORESET0_MAX_SC (NR_CORESET0_MAX_RB * 12)
#define NR_PDCCH_DMRS_RE_PER_RB 3
#define NR_PDCCH_DATA_RE_PER_RB 9
#define NR_PDCCH_REG_PER_CCE 6
#define NR_PDCCH_BITS_PER_CCE 108
#define NR_PDCCH_MAX_AGG_LEVEL 16
#define NR_PDCCH_MAX_CAND_REG (NR_PDCCH_MAX_AGG_LEVEL * NR_PDCCH_REG_PER_CCE)
#define NR_PDCCH_MAX_CAND_DATA_RE (NR_PDCCH_MAX_CAND_REG * NR_PDCCH_DATA_RE_PER_RB)
#define NR_PDCCH_MAX_E_BITS (NR_PDCCH_MAX_AGG_LEVEL * NR_PDCCH_BITS_PER_CCE)

typedef struct {
  int16_t r;
  int16_t i;
} c16_t;

typedef struct {
  float r;
  float i;
} cf32_t;

typedef enum {
  NR_PBCH_FAIL_NONE = 0,
  NR_PBCH_FAIL_WINDOW,
  NR_PBCH_FAIL_DEMOD,
  NR_PBCH_FAIL_DMRS_WEAK,
  NR_PBCH_FAIL_DMRS_AMBIG,
  NR_PBCH_FAIL_BCH
} nr_pbch_fail_stage_t;

typedef enum {
  TOA_STATE_INIT = 0,
  TOA_STATE_WAIT_CLOCK,
  TOA_STATE_PRESYNC,
  TOA_STATE_LOCKED,
  TOA_STATE_MEASURING,
  TOA_STATE_RESYNC,
  TOA_STATE_STOPPING
} nr_toa_state_t;

typedef enum {
  NR_TOA_MODE_SSB_TOA = 0,
  NR_TOA_MODE_PRS_TOA
} nr_toa_mode_t;

typedef enum {
  NR_MEAS_MODE_MEAS_ONLY = 0,
  NR_MEAS_MODE_PSEUDORANGE_ONLY,
  NR_MEAS_MODE_POSITION_SOLVE
} nr_meas_mode_t;

typedef struct nr_iq_ring nr_iq_ring_t;

typedef struct nr_iq_block {
  openair0_timestamp_t ts_first;
  uint64_t abs_samp0;
  uint32_t nsamps;
  double fs_hz; /* Sampling rate (Hz), set by radio read path */
  double rx_freq_hz; /* RF center frequency associated with this IQ capture. */
  int32_t rx_gscn;   /* Sweep raster index/GSCN associated with this IQ capture. */
  int32_t target_pci; /* Target PCI snapshot associated with this IQ capture. */
  uint32_t scan_target_idx; /* Configured scan target index for this IQ capture. */
  uint8_t rx_ant;
  uint8_t overrun;
  c16_t *rx[NR_TOA_MAX_RX_ANT];
  int32_t refcnt;
  nr_iq_ring_t *owner;
  uint8_t from_pool;
} nr_iq_block_t;

typedef struct {
  openair0_timestamp_t ts_first;
  uint32_t nsamps;
  uint8_t tx_ant;
  c16_t *tx[NR_TOA_MAX_RX_ANT];
} nr_tx_burst_t;

typedef struct {
  uint8_t anchor_id;
  uint16_t pci;
  uint8_t ssb_index;
  double x_m;
  double y_m;
  double z_m;
  uint8_t absolute_time_valid;
  double hw_cal_delay_ns; /* Frontend/cable chain fixed delay for absolute TOA. */
} nr_anchor_desc_t;

typedef struct {
  uint8_t valid;
  uint8_t subcarrier_spacing_common;
  uint8_t half_frame_bit;
  uint8_t ssb_subcarrier_offset_msb;
  uint8_t ssb_subcarrier_offset;
  uint8_t ssb_subcarrier_offset_full;
  uint8_t dmrs_typeA_position;
  uint8_t control_resource_set_zero;
  uint8_t search_space_zero;
  uint8_t cell_barred;
  uint8_t intra_freq_reselection;
  uint8_t spare;
} nr_mib_info_t;

typedef struct {
  uint8_t valid;
  uint8_t fr1_only;
  uint8_t scs_ssb_khz;
  uint8_t scs_pdcch_khz;
  uint8_t mux_pattern;
  int32_t num_rbs;
  int32_t num_symbols;
  int32_t rb_offset;
  int32_t cset_start_rb;
  int32_t sfn_c;
  uint32_t n_c;
  uint32_t n_0;
  uint32_t first_symbol_index;
  uint32_t search_space_duration;
  uint32_t search_space_frame_period;
  uint32_t slots_per_frame;
  uint32_t ssb_period_frames;
  uint32_t ssb_index;
} nr_type0_css_info_t;

typedef struct {
  uint8_t valid;
  uint16_t si_rnti;
  uint8_t dci_format_1_0;
  uint8_t common_search_space;
  uint8_t ref_point_pointA;
  uint8_t monitoring_active_now;
  uint8_t bwp_start_rb_valid;
  uint8_t coreset0_present;
  uint8_t pdcch_scs_khz;
  uint8_t dlsch_scs_khz;
  uint8_t k_ssb;
  uint8_t k_ssb_msb;
  uint32_t frame;
  uint32_t slot;
  uint32_t monitoring_period_slots;
  uint32_t monitoring_duration_slots;
  uint32_t n0;
  uint32_t nc;
  uint8_t k_ssb_norm;
  int32_t bwp_start_rb;
  int32_t bwp_size_rb;
  int32_t coreset_num_rbs;
  int32_t coreset_num_symbols;
  int32_t coreset_rb_offset;
  int32_t ssb_start_sc_rel_dc;
  int32_t coreset0_start_sc_rel_dc;
  int32_t coreset0_end_sc_rel_dc;
  uint32_t first_symbol_index;
  uint16_t monitoring_symbol_bitmap;
  uint8_t cce_reg_mapping_interleaved;
  uint8_t reg_bundle_size;
  uint8_t interleaver_size;
  uint16_t shift_index;
  uint16_t dmrs_scrambling_id;
  uint8_t candidate_count;
  struct {
    uint8_t aggregation_level;
    uint16_t first_cce;
  } candidates[NR_MAX_PDCCH_CANDIDATES];
} nr_sib1_rx_plan_t;

typedef struct {
  uint8_t valid;
  uint32_t frame;
  uint32_t slot;
  uint32_t start_samp;
  uint32_t len_samp;
  uint32_t first_symbol_index;
  uint32_t num_symbols;
  uint32_t slots_examined;
  int32_t start_sc_rel_dc;
  int32_t end_sc_rel_dc;
  int32_t num_rbs;
} nr_coreset0_monitor_window_t;

typedef struct {
  uint8_t valid;
  uint32_t frame;
  uint32_t slot;
  uint32_t first_symbol_index;
  uint32_t num_symbols;
  uint32_t num_rbs;
  uint32_t num_subcarriers;
  int32_t start_sc_rel_dc;
  int32_t end_sc_rel_dc;
  float avg_re_power;
  float peak_re_power;
  cf32_t re[NR_CORESET0_MAX_SYMBOLS][NR_CORESET0_MAX_SC];
} nr_coreset0_grid_t;

typedef struct {
  uint8_t valid;
  uint32_t frame;
  uint32_t slot;
  uint32_t first_symbol_index;
  uint32_t num_symbols;
  uint32_t num_rbs;
  uint32_t num_dmrs_re;
  uint32_t num_data_re;
  uint16_t dmrs_scrambling_id;
  uint8_t cce_reg_mapping_interleaved;
  uint8_t reg_bundle_size;
  uint8_t interleaver_size;
  uint16_t shift_index;
  float avg_dmrs_power;
  float avg_data_power;
  float avg_h_power;
  float corr_mag;
  float evm_rms;
  cf32_t h_avg[NR_CORESET0_MAX_SYMBOLS];
} nr_coreset0_dmrs_obs_t;

typedef struct {
  uint8_t valid;
  uint32_t frame;
  uint32_t slot;
  uint8_t candidate_index;
  uint8_t aggregation_level;
  uint16_t first_cce;
  uint16_t n_cce_total;
  uint16_t num_regs;
  uint16_t num_data_re;
  uint16_t reg_list[NR_PDCCH_MAX_CAND_REG];
  float avg_data_power;
  float avg_eq_power;
  cf32_t data_re[NR_PDCCH_MAX_CAND_DATA_RE];
  cf32_t eq_re[NR_PDCCH_MAX_CAND_DATA_RE];
} nr_pdcch_candidate_obs_t;

typedef struct {
  uint8_t valid;
  uint32_t frame;
  uint32_t slot;
  uint8_t candidate_index;
  uint8_t aggregation_level;
  uint16_t first_cce;
  uint16_t rnti;
  uint16_t scrambling_id;
  uint16_t encoded_bits;
  uint16_t dci_payload_bits;
  uint8_t freq_assignment_bits;
  uint8_t decoder_ready;
  uint8_t crc_ok;
  uint32_t crc_xor;
  uint64_t payload;
  uint16_t frequency_domain_assignment;
  uint8_t time_domain_assignment;
  uint8_t vrb_to_prb_mapping;
  uint8_t mcs;
  uint8_t redundancy_version;
  uint8_t system_info_indicator;
  uint16_t reserved_bits;
  float mean_abs_llr;
  float max_abs_llr;
  float positive_fraction;
  float llr[NR_PDCCH_MAX_E_BITS];
} nr_pdcch_dci10_si_obs_t;

typedef struct {
  uint8_t valid;
  uint8_t dci_crc_ok;
  uint32_t frame;
  uint32_t slot;
  uint32_t pdsch_frame;
  uint32_t pdsch_slot;
  uint16_t rnti;
  uint8_t candidate_index;
  uint8_t time_domain_assignment;
  uint8_t mapping_type_a;
  uint8_t k0;
  uint8_t start_symbol;
  uint8_t num_symbols;
  uint16_t bwp_start_rb;
  uint16_t bwp_size_rb;
  uint16_t rb_start;
  uint16_t rb_size;
  uint16_t rb_start_abs;
  uint8_t vrb_to_prb_mapping;
  uint8_t mcs;
  uint8_t redundancy_version;
  uint8_t system_info_indicator;
  uint8_t dmrs_typeA_position;
  uint8_t mux_pattern;
  uint8_t dlsch_scs_khz;
} nr_sib1_pdsch_grant_t;

typedef struct {
  double center_freq_hz;
  int32_t target_pci;
} nr_scan_target_t;

typedef struct {
  uint8_t valid;
  double center_freq_hz;
  int32_t gscn;
  int32_t nr_band;
  uint16_t pci;
  uint8_t ssb_index;
  uint32_t sfn;
  uint16_t ssb_start_symbol_abs;
  uint16_t ssb_start_slot;
  uint8_t timing_ref_valid;
  uint64_t frame_start_abs_samp;
  double timing_ref_fs_hz;
  float cfo_hz;
  float snr_db;
  float pss_metric;
  uint32_t mib_payload;
  nr_mib_info_t mib;
  nr_type0_css_info_t type0_css;
  nr_sib1_rx_plan_t sib1_rx;
} nr_cell_ctx_t;

typedef struct {
  uint8_t locked;
  uint8_t nid2;
  uint16_t nid1;
  uint16_t pci;
  uint16_t pci_full;
  uint8_t ssb_index;
  int32_t coarse_offset_samp;
  float frac_offset_samp;
  float cfo_hz;
  uint32_t sfn;
  uint16_t slot;
  float rsrp_db;
  float snr_db;
  float pss_metric;
  uint8_t pbch_ok;
  uint8_t pbch_confirmed; /* PBCH gate confirmation status (for lock consistency). */
  uint8_t mib_ok;         /* True only when a real PBCH/MIB decode has completed. */
  uint32_t mib_payload;   /* Reserved for future decoded MIB payload bits. */
  nr_mib_info_t mib;
  float pbch_metric;
  float pbch_metric_second;
  uint8_t pbch_fail_stage;
  uint8_t pbch_llr_valid;
  int16_t pbch_best_sss_delta_bias;
  int16_t pbch_best_timing_delta;
  float pbch_best_cfo_delta_hz;
  float pbch_best_phase_deg;
  uint8_t pbch_dmrs_n_hf;
  float pbch_noise_var;
  float pbch_cpe_rad;
  float pbch_llr[NR_PBCH_LLR_LEN];
  float lock_confidence;
  int32_t last_gscn;
  uint8_t overflow_seen;
  int64_t cum_tracking_shift_samp;
  uint8_t pci_hyp_count;
  uint16_t pci_hyp[NR_MAX_PCI_HYPS];
  int16_t pci_hyp_delta[NR_MAX_PCI_HYPS];
  float pci_hyp_metric[NR_MAX_PCI_HYPS];
} nr_sync_state_t;

typedef struct {
  uint16_t pci;
  uint8_t ssb_idx;
  uint32_t n_fft;
} nr_ssb_ref_t;

typedef struct {
  cf32_t re[NR_SSB_RE_ROWS][NR_SSB_RE_COLS];
  uint8_t valid;
} nr_ssb_grid_t;

typedef struct {
  int32_t peak_samp;
  float coarse_cfo_hz;
  float metric;
  uint8_t nid2;
} nr_pss_hit_t;

typedef struct {
  uint32_t start_samp;
  uint32_t len_samp;
} nr_ssb_window_t;

typedef struct {
  cf32_t *h_ls;
  uint8_t *valid_re;
  uint32_t n_re;
} nr_chest_t;

typedef struct {
  cf32_t *h_full;
  uint8_t *valid_re;
  uint32_t n_re;
} nr_chest_full_t;

typedef struct {
  cf32_t *cir;
  uint32_t cir_len;
  uint8_t os_factor;
  float peak_metric;
} nr_cir_t;

typedef struct {
  uint64_t epoch_id;
  uint8_t anchor_id;
  uint16_t pci;
  uint8_t ssb_index;
  uint32_t sfn;
  uint16_t slot;

  /* Timestamps kept in sample-domain split (integer + fractional). */
  openair0_timestamp_t tx_ts_int;
  double tx_ts_frac;
  openair0_timestamp_t rx_ts_int;
  double rx_ts_frac;

  /* TOA in sample-domain split; ns/range derived. */
  int64_t toa_samp_int;
  double toa_samp_frac;

  double fs_hz; /* samples per second */
  double toa_ns;
  double range_m;

  float snr_db;
  float peak_metric;
  float quality;
  uint8_t valid;
} nr_toa_meas_t;

typedef struct {
  uint64_t epoch_id;
  uint8_t num_meas;
  nr_toa_meas_t meas[NR_TOA_MAX_MEAS_PER_EP];
} nr_toa_epoch_t;

/* Phase-0 minimal job objects (used by actor threads). */
typedef struct {
  nr_iq_block_t *blk; /* job/actor reference (must be refcount-managed). */
  nr_sync_state_t sync_out; /* filled by provider->acquire */
} nr_sync_job_t;

typedef struct {
  nr_iq_block_t *blk; /* job/actor reference (must be refcount-managed). */
  nr_sync_state_t sync_snapshot;
  nr_toa_meas_t meas_out; /* filled by provider->extract_meas */
} nr_meas_job_t;

typedef struct {
  nr_toa_meas_t meas;
} nr_solver_job_t;

typedef struct {
  uint64_t epoch_id;
  double x_m, y_m, z_m;
  double clk_bias_ns;
  double residual_rms_ns;
  uint8_t used_meas;
  uint8_t valid;
} nr_loc_solution_t;

typedef struct {
  uint8_t locked;
  uint8_t pps_locked;
  uint8_t ref_locked;
  uint8_t gps_locked;
  openair0_timestamp_t epoch0_hw_timestamp;
} nr_clock_status_t;

typedef struct {
  char sdr[32];
  char sdr_addrs[256];
  char clock_source[32];
  char time_source[32];
  char rx_antenna[32];
  char tx_antenna[32];
  double center_freq_hz;
  double sample_rate_hz;
  double rx_gain_db;
  double tx_gain_db;
  nr_toa_mode_t mode;
  nr_meas_mode_t meas_mode;
  int32_t nr_band;
  char anchor_db_path[512];
  uint8_t trace_enable;
  uint8_t iq_dump_enable;
  uint8_t full_band_sweep;
  uint8_t strict_center_freq;
  uint8_t gain_sweep_enable;
  int32_t target_pci;
  uint32_t scan_target_count;
  nr_scan_target_t scan_targets[NR_TOA_MAX_SCAN_TARGETS];
  uint32_t ssb_scs_khz;

  /* SSB scheduler / RF stream shaping */
  uint32_t ssb_period_ms;
  uint32_t rx_ant;
  uint32_t tx_ant;
} nr_toa_app_cfg_t;

typedef struct {
  uint64_t epoch_id;
  uint8_t anchor_id;
  openair0_timestamp_t tx_hw_timestamp;
  uint16_t pci;
  uint8_t ssb_index;
  uint32_t period_ms;
} nr_ssb_tx_plan_t;

typedef struct {
  uint8_t n_obs;
  struct {
    uint8_t anchor_id;
    double x_m, y_m, z_m;
    double range_m;
    double weight;
    uint8_t valid;
  } obs[NR_TOA_MAX_ANCHORS];
} nr_solver_input_t;

struct nr_iq_ring {
  int depth;
  uint64_t abs_samp_base;
  uint32_t overrun_cnt;
  uint32_t underrun_cnt;
  uint32_t alloc_fallback_cnt;

  /* Simple SPSC ring (Phase-0): single producer pushes contiguous blocks,
   * consumer uses get_window() to retrieve by abs_samp0.
   */
  nr_iq_block_t **blocks;
  uint32_t head; /* oldest */
  uint32_t tail; /* next write */
  uint32_t count;

  pthread_mutex_t pool_mtx;
  nr_iq_block_t **pool_blocks;
  nr_iq_block_t **free_stack;
  uint32_t pool_depth;
  uint32_t free_count;
  uint32_t block_nsamps;
  uint8_t block_rx_ant;
  uint8_t pool_ready;
};

typedef struct nr_epoch_mgr {
  uint64_t next_epoch_id;
  uint8_t pending;
  uint32_t meas_count;
  nr_toa_meas_t meas_buf[NR_TOA_MAX_MEAS_PER_EP];
} nr_epoch_mgr_t;
