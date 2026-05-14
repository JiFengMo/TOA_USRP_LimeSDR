#pragma once

#include "openair1/PHY/NR_POSITIONING/nr_pos_types.h"
#include "openair1/PHY/NR_POSITIONING/nr_pos_provider_if.h"
#include "openair1/PHY/NR_POSITIONING/nr_toa_ue.h"
#include "radio/COMMON/common_lib.h"

/* Module: iq_ring */
int nr_iq_ring_init(nr_iq_ring_t *rb, int depth);
int nr_iq_ring_prealloc(nr_iq_ring_t *rb, uint32_t nsamps, uint8_t rx_ant);
nr_iq_block_t *nr_iq_ring_alloc(nr_iq_ring_t *rb, uint32_t nsamps);
nr_iq_block_t *nr_iq_ring_alloc_ex(nr_iq_ring_t *rb, uint32_t nsamps, uint8_t rx_ant);
void nr_iq_ring_push(nr_iq_ring_t *rb, nr_iq_block_t *blk);
void nr_iq_block_get(nr_iq_block_t *blk);
void nr_iq_block_put(nr_iq_block_t *blk);
nr_iq_block_t *nr_iq_ring_get_window(nr_iq_ring_t *rb, uint64_t abs_samp0,
                                     uint32_t len);
void nr_iq_ring_free(nr_iq_ring_t *rb);

/* Module: epoch_mgr */
int nr_epoch_mgr_push(nr_epoch_mgr_t *mgr, const nr_toa_meas_t *meas);
int nr_epoch_mgr_pop_ready(nr_epoch_mgr_t *mgr, nr_toa_epoch_t *epoch);

/* Module: anchor_db */
int nr_toa_load_anchor_db(const char *path, nr_anchor_desc_t *db, int *n);

/* Module: trace */
int nr_trace_sync(const nr_sync_state_t *sync);
int nr_trace_cir(const nr_cir_t *cir);
int nr_trace_meas(const nr_toa_meas_t *meas);
int nr_trace_solution(const nr_loc_solution_t *sol);

/* Module: solver */
int nr_pos_build_equations(const nr_toa_epoch_t *epoch, nr_solver_input_t *in);
int nr_pos_solve_wls(const nr_solver_input_t *in, nr_loc_solution_t *sol);
int nr_pos_validate_solution(const nr_loc_solution_t *sol);

/* cfg_loader */
int nr_toa_load_config(const char *path, nr_toa_app_cfg_t *cfg);
int nr_toa_build_rf_cfg(const nr_toa_app_cfg_t *cfg, openair0_config_t *rf_cfg);
const nr_pos_provider_if_t *nr_toa_select_provider(const nr_toa_app_cfg_t *cfg);

/* radio_if */
int nr_toa_radio_init(openair0_device_t **dev, openair0_config_t *cfg);
int nr_toa_radio_start(openair0_device_t *dev);
int nr_toa_radio_read(openair0_device_t *dev, nr_iq_block_t *blk);
int nr_toa_radio_write(openair0_device_t *dev, const nr_iq_block_t *blk);
int nr_toa_radio_set_rx_freq(openair0_device_t *dev, double rx_freq_hz);
int nr_toa_radio_set_rx_gain(openair0_device_t *dev, double rx_gain_db);

/* clock_mgr */
int nr_toa_wait_clock_lock(openair0_device_t *dev, nr_clock_status_t *st);
int nr_toa_set_epoch_at_next_pps(openair0_device_t *dev, uint64_t epoch_ns);
int nr_toa_get_device_time(openair0_device_t *dev, openair0_timestamp_t *ts);

/* ssb_tx_scheduler */
int nr_ssb_build_tx_plan(const nr_toa_app_cfg_t *cfg, nr_ssb_tx_plan_t *plan);
int nr_ssb_plan_next_epoch(nr_ssb_tx_plan_t *plan, uint64_t epoch_id);

/* ssb_beacon_gen */
int nr_ssb_gen_ref(uint16_t pci, uint8_t ssb_idx, nr_ssb_ref_t *ref);
int nr_ssb_build_grid(const nr_ssb_ref_t *ref, nr_ssb_grid_t *grid);
int nr_ssb_ofdm_mod(const nr_ssb_grid_t *grid, nr_tx_burst_t *burst);

/* ssb_ref (shared PSS/OFDM reference) */
#define NR_V0_SYNTH_PBCH_SSB_IDX 0U
#define NR_V0_SYNTH_PBCH_SFN 341U
#define NR_V0_SYNTH_PBCH_HRF 0U
#define NR_V0_SYNTH_PBCH_MIB_PAYLOAD 0x2b2345U

uint32_t nr_v0_pss_td_len(void);
uint32_t nr_v0_ssb_sym_len(void);
uint32_t nr_v0_ssb_burst_len(void);
void nr_v0_set_ssb_scs_khz(uint32_t scs_khz);
uint32_t nr_v0_get_ssb_scs_khz(void);
uint32_t nr_v0_default_ssb_scs_khz(double center_freq_hz);
double nr_v0_ssb_gscn_step_hz(void);
uint32_t nr_v0_ssb_nfft(double fs_hz);
uint32_t nr_v0_ssb_cp_len(double fs_hz);
uint32_t nr_v0_ssb_symbol_cp_len_fs(double fs_hz, uint32_t sym_idx);
uint32_t nr_v0_ssb_sym_len_fs(double fs_hz);
uint32_t nr_v0_ssb_symbol_len_fs(double fs_hz, uint32_t sym_idx);
uint32_t nr_v0_ssb_burst_len_fs(double fs_hz);
void nr_v0_set_ssb_abs_start_symbol(int32_t abs_symbol);
int32_t nr_v0_get_ssb_abs_start_symbol(void);
void nr_v0_pss_build_fd(int nid2, float *seq, uint32_t len);
void nr_v0_sss_build_fd(int nid1, int nid2, float *seq, uint32_t len);
void nr_v0_pss_build_td_f(int nid2, float *td_i, float *td_q, uint32_t len);
void nr_v0_pss_build_td_f_fs(int nid2, double fs_hz, float *td_i, float *td_q, uint32_t len);
int nr_v0_pss_build_td_iq(int nid2, c16_t *out, uint32_t out_len,
                          float amp);
void nr_v0_sss_build_td_f(int nid1, int nid2, float *td_i, float *td_q, uint32_t len);
void nr_v0_sss_build_td_f_fs(int nid1, int nid2, double fs_hz,
                             float *td_i, float *td_q, uint32_t len);
int nr_v0_ssb_build_burst_iq(int nid1, int nid2, c16_t *out, uint32_t out_len,
                             float amp);
int nr_v0_pbch_dmrs_build(int pci, int ssb_idx, int n_hf,
                          float *seq_i, float *seq_q, uint32_t max_len);
uint32_t nr_pbch_dmrs_re_positions(uint8_t v, uint16_t *rel_idx, uint8_t *sym_idx,
                                   uint32_t max_len);
uint32_t nr_pbch_data_re_positions(uint8_t v, uint16_t *rel_idx, uint8_t *sym_idx,
                                   uint32_t max_len);
int nr_pbch_bch_encode(uint16_t pci, uint8_t ssb_idx, uint16_t sfn, uint8_t hrf,
                       uint32_t mib_payload, uint8_t *coded_bits, uint32_t max_bits);
int nr_pbch_bch_decode(const float *llr, uint32_t llr_len, nr_sync_state_t *sync);
const uint16_t *nr_polar_reliability_sequence_512(void);
const uint8_t *nr_polar_input_interleaver_164(void);
const uint8_t *nr_polar_subblock_interleaver_32(void);

/* type0_css */
int nr_slots_per_frame_from_scs_khz(uint32_t scs_khz);
int nr_slots_per_half_frame_from_scs_khz(uint32_t scs_khz);
int nr_numerology_from_scs_khz(uint32_t scs_khz);
uint32_t nr_scs_khz_from_mib(const nr_mib_info_t *mib);
uint32_t nr_ofdm_symbol_cp_len_fs(double fs_hz, uint32_t scs_khz, uint32_t abs_symbol_index);
uint32_t nr_ofdm_symbol_len_fs(double fs_hz, uint32_t scs_khz, uint32_t abs_symbol_index);
uint32_t nr_slot_len_fs(double fs_hz, uint32_t scs_khz, uint32_t slot_index);
uint64_t nr_slot_start_samp_in_frame_fs(double fs_hz, uint32_t scs_khz, uint32_t slot_index);
uint64_t nr_frame_len_fs(double fs_hz, uint32_t scs_khz);
int nr_get_ssb_start_symbol_abs(uint32_t scs_khz, int32_t nr_band, uint8_t ssb_index);
int nr_cell_build_timing_ref(const nr_iq_block_t *blk,
                             const nr_sync_state_t *sync,
                             nr_cell_ctx_t *cell);
int nr_cell_estimate_frame_slot_from_samp(const nr_cell_ctx_t *cell,
                                          uint64_t abs_samp,
                                          uint32_t *frame,
                                          uint32_t *slot,
                                          uint64_t *slot_start_abs_samp);
int nr_type0_css_from_mib(const nr_mib_info_t *mib,
                          uint8_t ssb_index,
                          uint32_t sfn,
                          uint32_t ssb_period_ms,
                          nr_type0_css_info_t *css);
int nr_type0_css_is_monitoring_occasion(const nr_type0_css_info_t *css,
                                        uint32_t sfn,
                                        uint32_t slot);
int nr_sib1_rx_plan_from_cell(const nr_cell_ctx_t *cell,
                              uint32_t slot,
                              nr_sib1_rx_plan_t *plan);
int nr_sib1_rx_plan_for_frame_slot(const nr_cell_ctx_t *cell,
                                   uint32_t frame,
                                   uint32_t slot,
                                   nr_sib1_rx_plan_t *plan);
int nr_sib1_rx_plan_for_block(const nr_cell_ctx_t *cell,
                              const nr_iq_block_t *blk,
                              nr_sib1_rx_plan_t *plan);
int nr_type0_coreset0_monitor_window(const nr_cell_ctx_t *cell,
                                     const nr_iq_block_t *blk,
                                     nr_coreset0_monitor_window_t *win);
int nr_type0_coreset0_extract_grid(const nr_cell_ctx_t *cell,
                                   const nr_iq_block_t *blk,
                                   nr_coreset0_grid_t *grid);
int nr_type0_coreset0_extract_dmrs(const nr_cell_ctx_t *cell,
                                   const nr_iq_block_t *blk,
                                   const nr_coreset0_grid_t *grid,
                                   nr_coreset0_dmrs_obs_t *obs);
int nr_type0_pdcch_extract_candidate(const nr_cell_ctx_t *cell,
                                     const nr_iq_block_t *blk,
                                     const nr_coreset0_grid_t *grid,
                                     const nr_coreset0_dmrs_obs_t *dmrs,
                                     uint8_t candidate_index,
                                     nr_pdcch_candidate_obs_t *cand);
uint16_t nr_type0_dci10_si_payload_bits(const nr_sib1_rx_plan_t *plan,
                                        uint8_t *freq_assignment_bits);
int nr_type0_pdcch_build_dci10_si_softbits(const nr_cell_ctx_t *cell,
                                           const nr_pdcch_candidate_obs_t *cand,
                                           nr_pdcch_dci10_si_obs_t *dci);
int nr_sib1_pdsch_grant_from_dci(const nr_cell_ctx_t *cell,
                                 const nr_pdcch_dci10_si_obs_t *dci,
                                 nr_sib1_pdsch_grant_t *grant);

/* anchor_write */
int nr_anchor_write_burst(openair0_device_t *dev,
                          const nr_ssb_tx_plan_t *plan,
                          const nr_tx_burst_t *burst);
int nr_anchor_log_tx_event(const nr_ssb_tx_plan_t *plan);
openair0_timestamp_t nr_toa_get_last_tx_timestamp(void);

/* sync_actor */
int nr_ssb_pss_search(const nr_iq_block_t *blk, nr_pss_hit_t *hits, int max_hits);
int nr_ssb_refine_sync(const nr_iq_block_t *blk, const nr_pss_hit_t *hit,
                       nr_sync_state_t *sync);
int nr_ssb_pbch_decode(const nr_iq_block_t *blk, nr_sync_state_t *sync);

/* track_actor */
int nr_ssb_track_cfo(const nr_iq_block_t *blk, nr_sync_state_t *sync);
int nr_ssb_track_timing(const nr_iq_block_t *blk, nr_sync_state_t *sync,
                        int *sample_shift);
int nr_ssb_check_lost_lock(const nr_sync_state_t *sync);

/* ssb_extract */
int nr_ssb_extract_window(const nr_iq_block_t *blk, const nr_sync_state_t *sync,
                          nr_ssb_window_t *win);
int nr_ssb_demod(const nr_iq_block_t *blk, const nr_ssb_window_t *win,
                 float cfo_hz, nr_ssb_grid_t *grid);

/* ssb_chest */
int nr_ssb_ls_estimate(const nr_ssb_grid_t *grid, const nr_sync_state_t *sync, nr_chest_t *h);
int nr_ssb_interp_channel(const nr_chest_t *h, nr_chest_full_t *hf);
int nr_ssb_build_cir(const nr_chest_full_t *hf, nr_cir_t *cir);
float nr_ssb_pbch_dmrs_corr_metric(const nr_ssb_grid_t *grid,
                                   uint16_t pci,
                                   uint8_t ssb_idx,
                                   uint8_t n_hf);
int nr_ssb_pbch_prepare_frontend(const nr_ssb_grid_t *grid,
                                 uint16_t pci,
                                 uint8_t ssb_idx,
                                 uint8_t n_hf,
                                 float *dmrs_metric,
                                 float *noise_var,
                                 float *cpe_rad,
                                 float *llr);

/* ssb_toa_est */
int nr_toa_find_integer_peak(const nr_cir_t *cir, int *peak_idx);
int nr_toa_refine_fractional(const nr_cir_t *cir, int peak_idx, double *frac);
int nr_toa_build_meas(const nr_sync_state_t *sync, const nr_iq_block_t *blk,
                      int peak_idx, double frac, nr_toa_meas_t *meas);

/* meas_assoc */
int nr_meas_assoc_anchor(const nr_toa_meas_t *meas, const nr_anchor_desc_t *db,
                         int n);

/* TOA UE orchestration */
int nr_toa_clock_ready(nr_toa_ue_t *ue);
int nr_toa_read_two_frames(nr_toa_ue_t *ue, nr_iq_ring_t *ring);
int nr_toa_read_one_slot(nr_toa_ue_t *ue, nr_iq_ring_t *ring);
int nr_toa_enqueue_sync_job(nr_toa_ue_t *ue, nr_iq_block_t *blk);
void nr_toa_reset_tracking(nr_toa_ue_t *ue);
int nr_slot_contains_ssb(const nr_sync_state_t *sync);
