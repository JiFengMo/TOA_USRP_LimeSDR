#pragma once

#include "nr_pos_types.h"
#include "nr_pos_provider_if.h"
#include "radio/COMMON/common_lib.h"

/* pthread types are needed for actor/job Phase-0 runtime. */
#include <pthread.h>

/* Phase-0: positioning UE orchestration context (main thread + actors). */
typedef struct nr_toa_ue {
  nr_toa_state_t state;
  nr_sync_state_t sync;

  nr_iq_ring_t iq_ring;
  nr_epoch_mgr_t epoch_mgr;

  nr_clock_status_t clk;

  openair0_device_t *dev;
  openair0_config_t rf_cfg;
  nr_toa_app_cfg_t app_cfg;

  nr_anchor_desc_t anchors[NR_TOA_MAX_ANCHORS];
  int n_anchors;

  /* Phase-0: sample-domain write bookkeeping for ring windows. */
  uint64_t abs_samp_wr;
  uint32_t samples_per_slot;
  uint32_t samples_per_frame;
  double current_rx_freq_hz;
  int32_t current_rx_gscn;
  int32_t current_target_pci;
  uint32_t current_scan_target_idx;
  uint8_t rf_settle_reads;

  nr_cell_ctx_t serving_cell;
  nr_cell_ctx_t known_cells[NR_TOA_MAX_SCAN_TARGETS];
  uint32_t known_cell_count;

  /* Provider holds its own caches/state via ctx. */
  const nr_pos_provider_if_t *provider;
  void *provider_ctx;

  /* Actor threads + minimal job synchronization (Phase-0 single-fifo). */
  pthread_t sync_tid;

  pthread_mutex_t sync_mtx;
  pthread_cond_t sync_cv;
  int sync_job_pending;
  int sync_job_done;
  nr_iq_block_t *sync_job_blk;
  uint64_t sync_job_id;
#ifndef NR_SYNC_Q_DEPTH
#define NR_SYNC_Q_DEPTH 8
#endif
  nr_iq_block_t *sync_q[NR_SYNC_Q_DEPTH];
  uint64_t sync_q_job_id[NR_SYNC_Q_DEPTH];
  uint32_t sync_q_head;
  uint32_t sync_q_tail;
  uint32_t sync_q_count;

  /* Debug / accounting. */
  uint64_t next_sync_job_id;
  uint64_t sync_jobs_dropped;
  uint64_t rx_read_count;
  uint64_t rx_gap_count;
  uint64_t rx_next_abs_samp;

  /* Current pending job identifier is stored in sync_job_id above. */
} nr_toa_ue_t;
