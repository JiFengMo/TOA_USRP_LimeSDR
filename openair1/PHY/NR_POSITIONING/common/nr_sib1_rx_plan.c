#include "openair1/PHY/NR_POSITIONING/nr_pos_api.h"

#include <string.h>

#define NR_SI_RNTI 0xFFFFU

/* TS 38.214 Table 5.1.2.1.1-2/-4/-5, mirrored from OAI get_dl_tda_info(). */
static const uint8_t g_pdsch_tda_default_a_pos2[16][4] = {
    {1,0,2,12}, {1,0,2,10}, {1,0,2,9},  {1,0,2,7},
    {1,0,2,5},  {0,0,9,4},  {0,0,4,4},  {0,0,5,7},
    {0,0,5,2},  {0,0,9,2},  {0,0,12,2}, {1,0,1,13},
    {1,0,1,6},  {1,0,2,4},  {0,0,4,7},  {0,0,8,4}};
static const uint8_t g_pdsch_tda_default_a_pos3[16][4] = {
    {1,0,3,11}, {1,0,3,9},  {1,0,3,8},  {1,0,3,6},
    {1,0,3,4},  {0,0,10,4}, {0,0,6,4},  {0,0,5,7},
    {0,0,5,2},  {0,0,9,2},  {0,0,12,2}, {1,0,1,13},
    {1,0,1,6},  {1,0,2,4},  {0,0,4,7},  {0,0,8,4}};
static const uint8_t g_pdsch_tda_default_b_pos2[16][4] = {
    {0,0,2,2},  {0,0,4,2},  {0,0,6,2},  {0,0,8,2},
    {0,0,10,2}, {0,1,2,2},  {0,1,4,2},  {0,0,2,4},
    {0,0,4,4},  {0,0,6,4},  {0,0,8,4},  {0,0,10,4},
    {0,0,2,7},  {1,0,2,12}, {0,1,2,4},  {0,0,0,0}};
static const uint8_t g_pdsch_tda_default_b_pos3[16][4] = {
    {0,0,2,2},  {0,0,4,2},  {0,0,6,2},  {0,0,8,2},
    {0,0,10,2}, {0,1,2,2},  {0,1,4,2},  {0,0,2,4},
    {0,0,4,4},  {0,0,6,4},  {0,0,8,4},  {0,0,10,4},
    {0,0,2,7},  {1,0,3,11}, {0,1,2,4},  {0,0,0,0}};
static const uint8_t g_pdsch_tda_default_c_pos2[16][4] = {
    {0,0,2,2},  {0,0,4,2},  {0,0,6,2},  {0,0,8,2},
    {0,0,10,2}, {0,0,0,0},  {0,0,0,0},  {0,0,2,4},
    {0,0,4,4},  {0,0,6,4},  {0,0,8,4},  {0,0,10,4},
    {0,0,2,7},  {1,0,2,12}, {1,0,0,6},  {1,0,2,6}};
static const uint8_t g_pdsch_tda_default_c_pos3[16][4] = {
    {0,0,2,2},  {0,0,4,2},  {0,0,6,2},  {0,0,8,2},
    {0,0,10,2}, {0,0,0,0},  {0,0,0,0},  {0,0,2,4},
    {0,0,4,4},  {0,0,6,4},  {0,0,8,4},  {0,0,10,4},
    {0,0,2,7},  {1,0,3,11}, {1,0,0,6},  {1,0,2,6}};

static uint8_t nr_k_ssb_norm_fr1(const nr_cell_ctx_t *cell)
{
  const uint32_t scs_common_khz =
      (cell && (cell->mib.subcarrier_spacing_common & 1U)) ? 30U : 15U;

  if (!cell) {
    return 0U;
  }

  if (cell->mib.ssb_subcarrier_offset_full < 24U) {
    return (scs_common_khz == 30U)
             ? (uint8_t)(cell->mib.ssb_subcarrier_offset_full >> 1U)
             : cell->mib.ssb_subcarrier_offset_full;
  }

  return cell->mib.ssb_subcarrier_offset_full;
}

static uint16_t nr_sl_to_bitmap(uint32_t first_symbol, uint32_t num_symbols)
{
  uint16_t bitmap = 0U;
  for (uint32_t i = 0U; i < num_symbols && (first_symbol + i) < 14U; i++) {
    bitmap |= (uint16_t)(1U << (13U - (first_symbol + i)));
  }
  return bitmap;
}

static uint8_t nr_sib1_common_css_candidates(uint8_t aggregation_level)
{
  switch (aggregation_level) {
    case 1U:
    case 2U:
      return 0U;
    case 4U:
      return 4U;
    case 8U:
      return 2U;
    case 16U:
      return 1U;
    default:
      return 0U;
  }
}

static void nr_sib1_fill_candidates(nr_sib1_rx_plan_t *plan)
{
  uint8_t count = 0U;
  int n_cce_sym = 0;
  int n_cces = 0;

  if (!plan) {
    return;
  }

  n_cce_sym = (plan->coreset_num_rbs > 0) ? (plan->coreset_num_rbs / 6) : 0;
  n_cces = n_cce_sym * plan->coreset_num_symbols;
  if (n_cces <= 0) {
    return;
  }

  for (int maxL = 16; maxL > 0; maxL >>= 1) {
    const uint8_t aggregation = (uint8_t)maxL;
    const uint8_t max_candidates = nr_sib1_common_css_candidates(aggregation);
    if (max_candidates == 0U || n_cces < aggregation) {
      continue;
    }

    for (uint8_t j = 0U; j < max_candidates; j++) {
      const uint16_t first_cce =
          (uint16_t)(aggregation *
                     (((uint32_t)j * (uint32_t)n_cces) /
                      ((uint32_t)aggregation * (uint32_t)max_candidates) %
                      ((uint32_t)n_cces / (uint32_t)aggregation)));
      uint8_t duplicated = 0U;

      for (uint8_t k = 0U; k < count; k++) {
        if (plan->candidates[k].aggregation_level == aggregation &&
            plan->candidates[k].first_cce == first_cce) {
          duplicated = 1U;
          break;
        }
      }
      if (!duplicated && count < NR_MAX_PDCCH_CANDIDATES) {
        plan->candidates[count].aggregation_level = aggregation;
        plan->candidates[count].first_cce = first_cce;
        count++;
      }
    }
  }

  plan->candidate_count = count;
}

static int nr_sib1_rx_plan_build(const nr_cell_ctx_t *cell,
                                 uint32_t frame,
                                 uint32_t slot,
                                 nr_sib1_rx_plan_t *plan)
{
  if (!cell || !plan || !cell->valid || !cell->mib.valid || !cell->type0_css.valid) {
    if (plan) {
      memset(plan, 0, sizeof(*plan));
    }
    return -1;
  }

  memset(plan, 0, sizeof(*plan));
  plan->valid = 1U;
  plan->si_rnti = NR_SI_RNTI;
  plan->dci_format_1_0 = 1U;
  plan->common_search_space = 1U;
  plan->ref_point_pointA = 1U;
  plan->pdcch_scs_khz = cell->type0_css.scs_pdcch_khz;
  plan->dlsch_scs_khz = cell->mib.subcarrier_spacing_common ? 30U : 15U;
  plan->k_ssb = cell->mib.ssb_subcarrier_offset_full;
  plan->k_ssb_msb = cell->mib.ssb_subcarrier_offset_msb;
  plan->k_ssb_norm = nr_k_ssb_norm_fr1(cell);
  plan->frame = frame;
  plan->slot = slot;
  plan->monitoring_period_slots = cell->type0_css.search_space_frame_period;
  plan->monitoring_duration_slots = cell->type0_css.search_space_duration;
  plan->n0 = cell->type0_css.n_0;
  plan->nc = cell->type0_css.n_c;
  plan->bwp_size_rb = cell->type0_css.num_rbs;
  plan->bwp_start_rb = cell->type0_css.cset_start_rb;
  plan->bwp_start_rb_valid = (cell->type0_css.cset_start_rb >= 0) ? 1U : 0U;
  plan->coreset0_present = (cell->mib.ssb_subcarrier_offset_full < 24U) ? 1U : 0U;
  plan->coreset_num_rbs = cell->type0_css.num_rbs;
  plan->coreset_num_symbols = cell->type0_css.num_symbols;
  plan->coreset_rb_offset = cell->type0_css.rb_offset;
  plan->ssb_start_sc_rel_dc = -120;
  plan->coreset0_start_sc_rel_dc =
      plan->ssb_start_sc_rel_dc - (12 * plan->coreset_rb_offset) - (int32_t)plan->k_ssb_norm;
  plan->coreset0_end_sc_rel_dc =
      plan->coreset0_start_sc_rel_dc + (12 * plan->coreset_num_rbs) - 1;
  plan->first_symbol_index = cell->type0_css.first_symbol_index;
  plan->monitoring_symbol_bitmap =
      nr_sl_to_bitmap(cell->type0_css.first_symbol_index,
                      (uint32_t)(cell->type0_css.num_symbols > 0 ? cell->type0_css.num_symbols : 0));
  /* Keep CORESET0 defaults aligned with OAI fill_coresetZero() for Type0 CSS. */
  plan->cce_reg_mapping_interleaved = 1U;
  plan->reg_bundle_size = 6U;
  plan->interleaver_size = 2U;
  plan->shift_index = cell->pci;
  plan->dmrs_scrambling_id = cell->pci;
  plan->monitoring_active_now =
      nr_type0_css_is_monitoring_occasion(&cell->type0_css, frame, slot) ? 1U : 0U;
  nr_sib1_fill_candidates(plan);
  return 0;
}

int nr_sib1_rx_plan_from_cell(const nr_cell_ctx_t *cell,
                              uint32_t slot,
                              nr_sib1_rx_plan_t *plan)
{
  uint32_t scs_khz = 15U;
  int slots_per_half_frame = -1;
  uint32_t abs_slot = slot;

  if (!cell || !plan || !cell->valid || !cell->mib.valid || !cell->type0_css.valid) {
    if (plan) {
      memset(plan, 0, sizeof(*plan));
    }
    return -1;
  }

  scs_khz = nr_scs_khz_from_mib(&cell->mib);
  slots_per_half_frame = nr_slots_per_half_frame_from_scs_khz(scs_khz);
  if (slots_per_half_frame > 0) {
    abs_slot = (uint32_t)(cell->mib.half_frame_bit ? slots_per_half_frame : 0) + cell->ssb_start_slot;
  }

  return nr_sib1_rx_plan_build(cell, cell->sfn, abs_slot, plan);
}

int nr_sib1_rx_plan_for_frame_slot(const nr_cell_ctx_t *cell,
                                   uint32_t frame,
                                   uint32_t slot,
                                   nr_sib1_rx_plan_t *plan)
{
  if (!cell || !plan || !cell->valid || !cell->mib.valid || !cell->type0_css.valid) {
    if (plan) {
      memset(plan, 0, sizeof(*plan));
    }
    return -1;
  }

  return nr_sib1_rx_plan_build(cell, frame, slot, plan);
}

int nr_sib1_rx_plan_for_block(const nr_cell_ctx_t *cell,
                              const nr_iq_block_t *blk,
                              nr_sib1_rx_plan_t *plan)
{
  uint32_t frame = 0U;
  uint32_t slot = 0U;

  if (!cell || !blk || !plan) {
    if (plan) {
      memset(plan, 0, sizeof(*plan));
    }
    return -1;
  }

  if (nr_cell_estimate_frame_slot_from_samp(cell, blk->abs_samp0, &frame, &slot, NULL) != 0) {
    return nr_sib1_rx_plan_from_cell(cell, cell->ssb_start_slot, plan);
  }

  return nr_sib1_rx_plan_build(cell, frame, slot, plan);
}

int nr_type0_coreset0_monitor_window(const nr_cell_ctx_t *cell,
                                     const nr_iq_block_t *blk,
                                     nr_coreset0_monitor_window_t *win)
{
  const uint32_t scs_khz =
      (cell && cell->type0_css.valid && cell->type0_css.scs_pdcch_khz > 0U)
          ? cell->type0_css.scs_pdcch_khz
          : nr_scs_khz_from_mib(cell ? &cell->mib : NULL);
  uint32_t frame = 0U;
  uint32_t slot = 0U;
  uint64_t slot_start_abs = 0U;
  uint64_t cursor_abs = 0U;
  uint32_t examined = 0U;

  if (!win) {
    return -1;
  }
  memset(win, 0, sizeof(*win));

  if (!cell || !blk || !cell->valid || !cell->type0_css.valid || !cell->mib.valid) {
    return -1;
  }

  if (nr_cell_estimate_frame_slot_from_samp(cell, blk->abs_samp0, &frame, &slot, &slot_start_abs) != 0) {
    return -1;
  }

  cursor_abs = slot_start_abs;
  while (cursor_abs < (blk->abs_samp0 + blk->nsamps)) {
    nr_sib1_rx_plan_t plan;
    uint64_t sym_offset = 0U;
    uint64_t sym_len = 0U;
    memset(&plan, 0, sizeof(plan));
    if (nr_sib1_rx_plan_build(cell, frame, slot, &plan) == 0 && plan.monitoring_active_now) {
      for (uint32_t sym = 0U; sym < plan.first_symbol_index; sym++) {
        sym_offset += nr_ofdm_symbol_len_fs(blk->fs_hz, scs_khz, slot * 14U + sym);
      }
      for (uint32_t sym = 0U; sym < (uint32_t)plan.coreset_num_symbols; sym++) {
        sym_len += nr_ofdm_symbol_len_fs(blk->fs_hz, scs_khz, slot * 14U + plan.first_symbol_index + sym);
      }
      if ((cursor_abs + sym_offset) >= blk->abs_samp0 &&
          (cursor_abs + sym_offset + sym_len) <= (blk->abs_samp0 + blk->nsamps)) {
        win->valid = 1U;
        win->frame = frame;
        win->slot = slot;
        win->start_samp = (uint32_t)((cursor_abs + sym_offset) - blk->abs_samp0);
        win->len_samp = (uint32_t)sym_len;
        win->first_symbol_index = plan.first_symbol_index;
        win->num_symbols = (uint32_t)plan.coreset_num_symbols;
        win->start_sc_rel_dc = plan.coreset0_start_sc_rel_dc;
        win->end_sc_rel_dc = plan.coreset0_end_sc_rel_dc;
        win->num_rbs = plan.coreset_num_rbs;
        win->slots_examined = examined + 1U;
        return 0;
      }
    }

    cursor_abs += nr_slot_len_fs(blk->fs_hz, scs_khz, slot);
    slot++;
    if (slot >= (uint32_t)nr_slots_per_frame_from_scs_khz(scs_khz)) {
      slot = 0U;
      frame = (frame + 1U) % 1024U;
    }
    examined++;
  }

  return -1;
}

static int nr_sib1_default_tda(const nr_cell_ctx_t *cell,
                               uint8_t tda_index,
                               nr_sib1_pdsch_grant_t *grant)
{
  const uint8_t (*table)[4] = NULL;

  if (!cell || !grant || tda_index >= 16U) {
    return -1;
  }

  switch (cell->type0_css.mux_pattern) {
    case 1U:
      table = cell->mib.dmrs_typeA_position ? g_pdsch_tda_default_a_pos3
                                            : g_pdsch_tda_default_a_pos2;
      break;
    case 2U:
      table = cell->mib.dmrs_typeA_position ? g_pdsch_tda_default_b_pos3
                                            : g_pdsch_tda_default_b_pos2;
      break;
    case 3U:
      table = cell->mib.dmrs_typeA_position ? g_pdsch_tda_default_c_pos3
                                            : g_pdsch_tda_default_c_pos2;
      break;
    default:
      return -1;
  }

  if (table[tda_index][3] == 0U) {
    return -1;
  }

  grant->time_domain_assignment = tda_index;
  grant->mapping_type_a = table[tda_index][0];
  grant->k0 = table[tda_index][1];
  grant->start_symbol = table[tda_index][2];
  grant->num_symbols = table[tda_index][3];
  grant->dmrs_typeA_position = cell->mib.dmrs_typeA_position;
  grant->mux_pattern = cell->type0_css.mux_pattern;
  grant->dlsch_scs_khz = cell->mib.subcarrier_spacing_common ? 30U : 15U;
  return 0;
}

static int nr_sib1_riv_to_rb(uint16_t riv,
                             uint16_t n_rb,
                             uint16_t *rb_start,
                             uint16_t *rb_size)
{
  const uint16_t tmp = n_rb ? (uint16_t)(riv / n_rb) : 0U;
  const uint16_t tmp2 = n_rb ? (uint16_t)(riv % n_rb) : 0U;

  if (!n_rb || !rb_start || !rb_size) {
    return -1;
  }

  if (tmp <= (uint16_t)((n_rb >> 1U) + 1U) && (uint16_t)(tmp + tmp2) < n_rb) {
    *rb_size = (uint16_t)(tmp + 1U);
    *rb_start = tmp2;
  } else {
    *rb_size = (uint16_t)(n_rb + 1U - tmp);
    *rb_start = (uint16_t)(n_rb - 1U - tmp2);
  }

  if (*rb_size == 0U || *rb_start >= n_rb || *rb_size > (uint16_t)(n_rb - *rb_start)) {
    return -1;
  }
  return 0;
}

int nr_sib1_pdsch_grant_from_dci(const nr_cell_ctx_t *cell,
                                 const nr_pdcch_dci10_si_obs_t *dci,
                                 nr_sib1_pdsch_grant_t *grant)
{
  const uint16_t n_rb = (uint16_t)((cell && cell->type0_css.num_rbs > 0)
                                      ? cell->type0_css.num_rbs
                                      : 0);
  const int slots_per_frame =
      cell ? nr_slots_per_frame_from_scs_khz(cell->mib.subcarrier_spacing_common ? 30U : 15U) : -1;
  uint32_t pdsch_abs_slot = 0U;

  if (!grant) {
    return -1;
  }
  memset(grant, 0, sizeof(*grant));

  if (!cell || !cell->valid || !cell->mib.valid || !cell->type0_css.valid ||
      !dci || !dci->valid || !dci->decoder_ready || !dci->crc_ok ||
      dci->rnti != NR_SI_RNTI || slots_per_frame <= 0) {
    return -1;
  }

  grant->dci_crc_ok = dci->crc_ok;
  grant->frame = dci->frame;
  grant->slot = dci->slot;
  grant->rnti = dci->rnti;
  grant->candidate_index = dci->candidate_index;
  grant->bwp_size_rb = n_rb;
  grant->bwp_start_rb = (uint16_t)((cell->type0_css.cset_start_rb >= 0)
                                      ? cell->type0_css.cset_start_rb
                                      : 0);
  grant->vrb_to_prb_mapping = dci->vrb_to_prb_mapping;
  grant->mcs = dci->mcs;
  grant->redundancy_version = dci->redundancy_version;
  grant->system_info_indicator = dci->system_info_indicator;

  if (nr_sib1_default_tda(cell, dci->time_domain_assignment, grant) != 0 ||
      nr_sib1_riv_to_rb(dci->frequency_domain_assignment, n_rb,
                        &grant->rb_start, &grant->rb_size) != 0) {
    memset(grant, 0, sizeof(*grant));
    return -1;
  }

  grant->rb_start_abs = (uint16_t)(grant->bwp_start_rb + grant->rb_start);
  pdsch_abs_slot = dci->frame * (uint32_t)slots_per_frame + dci->slot + grant->k0;
  grant->pdsch_frame = (pdsch_abs_slot / (uint32_t)slots_per_frame) % 1024U;
  grant->pdsch_slot = pdsch_abs_slot % (uint32_t)slots_per_frame;
  grant->valid = 1U;
  return 0;
}
