#include "openair1/PHY/NR_POSITIONING/nr_pos_api.h"

#include <limits.h>
#include <stdint.h>
#include <string.h>

#define NR_RESERVED (-1)

/* 3GPP TS 38.213 Table 13-1 .. 13-4 / 13-11.
 * This skeleton intentionally focuses on the FR1 cases we need next for
 * Type0-PDCCH/SIB1 progression after PBCH/MIB lock.
 */
static const int32_t table_38213_13_1_c2[16] = {24, 24, 24, 24, 24, 24, 48, 48, 48, 48, 48, 48, 96, 96, 96, NR_RESERVED};
static const int32_t table_38213_13_1_c3[16] = { 2,  2,  2,  3,  3,  3,  1,  1,  2,  2,  3,  3,  1,  2,  3, NR_RESERVED};
static const int32_t table_38213_13_1_c4[16] = { 0,  2,  4,  0,  2,  4, 12, 16, 12, 16, 12, 16, 38, 38, 38, NR_RESERVED};

static const int32_t table_38213_13_2_c2[16] = {24, 24, 24, 24, 24, 24, 24, 24, 48, 48, 48, 48, 48, 48, NR_RESERVED, NR_RESERVED};
static const int32_t table_38213_13_2_c3[16] = { 2,  2,  2,  2,  3,  3,  3,  3,  1,  1,  2,  2,  3,  3, NR_RESERVED, NR_RESERVED};
static const int32_t table_38213_13_2_c4[16] = { 5,  6,  7,  8,  5,  6,  7,  8, 18, 20, 18, 20, 18, 20, NR_RESERVED, NR_RESERVED};

static const int32_t table_38213_13_3_c2[16] = {48, 48, 48, 48, 48, 48, 96, 96, 96, NR_RESERVED, NR_RESERVED, NR_RESERVED, NR_RESERVED, NR_RESERVED, NR_RESERVED, NR_RESERVED};
static const int32_t table_38213_13_3_c3[16] = { 1,  1,  2,  2,  3,  3,  1,  2,  3, NR_RESERVED, NR_RESERVED, NR_RESERVED, NR_RESERVED, NR_RESERVED, NR_RESERVED, NR_RESERVED};
static const int32_t table_38213_13_3_c4[16] = { 2,  6,  2,  6,  2,  6, 28, 28, 28, NR_RESERVED, NR_RESERVED, NR_RESERVED, NR_RESERVED, NR_RESERVED, NR_RESERVED, NR_RESERVED};

static const int32_t table_38213_13_4_c2[16] = {24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 48, 48, 48, 48, 48, 48};
static const int32_t table_38213_13_4_c3[16] = { 2,  2,  2,  2,  2,  3,  3,  3,  3,  3,  1,  1,  1,  2,  2,  2};
static const int32_t table_38213_13_4_c4[16] = { 0,  1,  2,  3,  4,  0,  1,  2,  3,  4, 12, 14, 16, 12, 14, 16};

static const float table_38213_13_11_c1[16] = {0, 0, 2, 2, 5, 5, 7, 7, 0, 5, 0, 0, 2, 2, 5, 5};
static const float table_38213_13_11_c3[16] = {1, 0.5f, 1, 0.5f, 1, 0.5f, 1, 0.5f, 2, 2, 1, 1, 1, 1, 1, 1};
static const int32_t table_38213_13_11_c4[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 1, 2, 1, 2};

int nr_slots_per_frame_from_scs_khz(uint32_t scs_khz)
{
  switch (scs_khz) {
    case 15U:
      return 10;
    case 30U:
      return 20;
    case 60U:
      return 40;
    case 120U:
      return 80;
    case 240U:
      return 160;
    default:
      return -1;
  }
}

static int nr_type0_css_fill_coreset_fr1(uint32_t scs_ssb_khz,
                                         uint32_t scs_pdcch_khz,
                                         uint8_t index_4msb,
                                         nr_type0_css_info_t *css)
{
  if (!css) {
    return -1;
  }

  css->mux_pattern = 1U;
  css->num_rbs = NR_RESERVED;
  css->num_symbols = NR_RESERVED;
  css->rb_offset = NR_RESERVED;

  if (scs_ssb_khz == 15U && scs_pdcch_khz == 15U) {
    if (index_4msb >= 15U) {
      return -1;
    }
    css->num_rbs = table_38213_13_1_c2[index_4msb];
    css->num_symbols = table_38213_13_1_c3[index_4msb];
    css->rb_offset = table_38213_13_1_c4[index_4msb];
    return 0;
  }

  if (scs_ssb_khz == 15U && scs_pdcch_khz == 30U) {
    if (index_4msb >= 14U) {
      return -1;
    }
    css->num_rbs = table_38213_13_2_c2[index_4msb];
    css->num_symbols = table_38213_13_2_c3[index_4msb];
    css->rb_offset = table_38213_13_2_c4[index_4msb];
    return 0;
  }

  if (scs_ssb_khz == 30U && scs_pdcch_khz == 15U) {
    if (index_4msb >= 9U) {
      return -1;
    }
    css->num_rbs = table_38213_13_3_c2[index_4msb];
    css->num_symbols = table_38213_13_3_c3[index_4msb];
    css->rb_offset = table_38213_13_3_c4[index_4msb];
    return 0;
  }

  if (scs_ssb_khz == 30U && scs_pdcch_khz == 30U) {
    css->num_rbs = table_38213_13_4_c2[index_4msb];
    css->num_symbols = table_38213_13_4_c3[index_4msb];
    css->rb_offset = table_38213_13_4_c4[index_4msb];
    return 0;
  }

  return -1;
}

int nr_type0_css_from_mib(const nr_mib_info_t *mib,
                          uint8_t ssb_index,
                          uint32_t sfn,
                          uint32_t ssb_period_ms,
                          nr_type0_css_info_t *css)
{
  const uint32_t scs_ssb_khz = (mib && (mib->subcarrier_spacing_common & 1U)) ? 30U : 15U;
  const uint32_t scs_pdcch_khz = scs_ssb_khz;
  const int slots_per_frame = nr_slots_per_frame_from_scs_khz(scs_ssb_khz);
  const uint8_t index_4msb = mib ? mib->control_resource_set_zero : 0U;
  const uint8_t index_4lsb = mib ? mib->search_space_zero : 0U;
  float big_o = 0.0f;
  float big_m = 0.0f;
  uint32_t temp = 0U;

  if (!mib || !css || !mib->valid) {
    return -1;
  }

  memset(css, 0, sizeof(*css));
  css->fr1_only = 1U;
  css->scs_ssb_khz = (uint8_t)scs_ssb_khz;
  css->scs_pdcch_khz = (uint8_t)scs_pdcch_khz;
  css->ssb_index = ssb_index;
  css->slots_per_frame = (uint32_t)slots_per_frame;
  css->ssb_period_frames = (ssb_period_ms ? ssb_period_ms : 20U) / 10U;
  if (css->ssb_period_frames == 0U) {
    css->ssb_period_frames = 2U;
  }
  css->cset_start_rb = NR_RESERVED;

  if (slots_per_frame <= 0) {
    return -1;
  }
  if (nr_type0_css_fill_coreset_fr1(scs_ssb_khz, scs_pdcch_khz, index_4msb, css) != 0) {
    return -1;
  }

  big_o = table_38213_13_11_c1[index_4lsb];
  big_m = table_38213_13_11_c3[index_4lsb];
  temp = (uint32_t)(big_o * (float)(1U << (scs_pdcch_khz == 30U ? 1U : 0U))) +
         (uint32_t)((float)ssb_index * big_m);

  css->n_c = temp / (uint32_t)slots_per_frame;
  css->sfn_c = (int32_t)(css->n_c % 2U);
  css->n_0 = temp % (uint32_t)slots_per_frame;
  if ((index_4lsb == 1U || index_4lsb == 3U || index_4lsb == 5U || index_4lsb == 7U) &&
      (ssb_index & 1U)) {
    css->first_symbol_index = (uint32_t)css->num_symbols;
  } else {
    css->first_symbol_index = (uint32_t)table_38213_13_11_c4[index_4lsb];
  }
  css->search_space_duration = 2U;
  css->search_space_frame_period = ((uint32_t)slots_per_frame) << 1U;
  css->valid = 1U;

  (void)sfn;
  return 0;
}

int nr_type0_css_is_monitoring_occasion(const nr_type0_css_info_t *css,
                                        uint32_t sfn,
                                        uint32_t slot)
{
  uint32_t absolute_slot = 0U;
  uint32_t offset = 0U;

  if (!css || !css->valid || css->search_space_frame_period == 0U) {
    return 0;
  }

  absolute_slot = sfn * css->slots_per_frame + slot;
  offset = css->n_0 + css->slots_per_frame * (uint32_t)(css->sfn_c < 0 ? 0 : css->sfn_c);
  return ((absolute_slot + css->search_space_frame_period - offset) %
          css->search_space_frame_period) < css->search_space_duration;
}
