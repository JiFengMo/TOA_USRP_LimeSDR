#pragma once

#include "radio/COMMON/common_lib.h"

#include <stdint.h>

#define NR_TOA_MAX_RX_ANT 4
#define NR_TOA_MAX_ANCHORS 8
#define NR_TOA_MAX_MEAS_PER_EP 16
#define NR_SSB_RE_ROWS 4
#define NR_SSB_RE_COLS 240

typedef struct {
  int16_t r;
  int16_t i;
} c16_t;

typedef struct {
  float r;
  float i;
} cf32_t;

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

typedef struct {
  openair0_timestamp_t ts_first;
  uint64_t abs_samp0;
  uint32_t nsamps;
  uint8_t rx_ant;
  uint8_t overrun;
  c16_t *rx[NR_TOA_MAX_RX_ANT];
  int32_t refcnt;
} nr_iq_block_t;

typedef struct {
  uint8_t anchor_id;
  uint16_t pci;
  uint8_t ssb_index;
  double x_m;
  double y_m;
  double z_m;
  uint8_t absolute_time_valid;
} nr_anchor_desc_t;

typedef struct {
  uint8_t locked;
  uint16_t pci;
  uint8_t ssb_index;
  int32_t coarse_offset_samp;
  float frac_offset_samp;
  float cfo_hz;
  uint32_t sfn;
  uint16_t slot;
  float rsrp_db;
  float snr_db;
  int64_t cum_tracking_shift_samp;
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
} nr_pss_hit_t;

typedef struct {
  uint32_t start_samp;
  uint32_t len_samp;
} nr_ssb_window_t;

typedef struct {
  cf32_t *h_ls;
  uint32_t n_re;
} nr_chest_t;

typedef struct {
  cf32_t *h_full;
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

  openair0_timestamp_t rx_ts_int;
  double rx_ts_frac;
  double fs_hz;

  double toa_ns;
  double tx_epoch_ns;
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
  double center_freq_hz;
  double sample_rate_hz;
  double rx_gain_db;
  double tx_gain_db;
  nr_toa_mode_t mode;
  nr_meas_mode_t meas_mode;
  char anchor_db_path[512];
  uint8_t trace_enable;
  uint8_t iq_dump_enable;
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
  double ranges_m[NR_TOA_MAX_ANCHORS];
  double weights[NR_TOA_MAX_ANCHORS];
  uint8_t n_anchors;
} nr_solver_input_t;

typedef struct nr_iq_ring {
  int depth;
  uint64_t abs_samp_base;
  uint32_t overrun_cnt;
  uint32_t underrun_cnt;
} nr_iq_ring_t;

typedef struct nr_epoch_mgr {
  uint64_t next_epoch_id;
  uint8_t pending;
} nr_epoch_mgr_t;
