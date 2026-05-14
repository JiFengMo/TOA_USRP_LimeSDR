#include "openair1/PHY/NR_POSITIONING/nr_pos_api.h"

#include <math.h>
#include <stdint.h>

uint32_t nr_scs_khz_from_mib(const nr_mib_info_t *mib)
{
  if (!mib) {
    return 15U;
  }
  return (mib->subcarrier_spacing_common & 1U) ? 30U : 15U;
}

int nr_numerology_from_scs_khz(uint32_t scs_khz)
{
  switch (scs_khz) {
    case 15U:
      return 0;
    case 30U:
      return 1;
    case 60U:
      return 2;
    case 120U:
      return 3;
    case 240U:
      return 4;
    default:
      return -1;
  }
}

int nr_slots_per_half_frame_from_scs_khz(uint32_t scs_khz)
{
  const int slots_per_frame = nr_slots_per_frame_from_scs_khz(scs_khz);
  return (slots_per_frame > 0) ? (slots_per_frame / 2) : -1;
}

uint32_t nr_ofdm_symbol_cp_len_fs(double fs_hz, uint32_t scs_khz, uint32_t abs_symbol_index)
{
  const uint32_t nfft = nr_v0_ssb_nfft(fs_hz);
  const int mu = nr_numerology_from_scs_khz(scs_khz);
  const double short_cp = (144.0 * (double)nfft) / 2048.0;
  const double long_cp = ((144.0 + 16.0 * (double)(1U << mu)) * (double)nfft) / 2048.0;

  if (mu < 0) {
    return 0U;
  }

  return (abs_symbol_index % (uint32_t)(7U << mu)) == 0U
           ? (uint32_t)llround(long_cp)
           : (uint32_t)llround(short_cp);
}

uint32_t nr_ofdm_symbol_len_fs(double fs_hz, uint32_t scs_khz, uint32_t abs_symbol_index)
{
  return nr_v0_ssb_nfft(fs_hz) + nr_ofdm_symbol_cp_len_fs(fs_hz, scs_khz, abs_symbol_index);
}

uint32_t nr_slot_len_fs(double fs_hz, uint32_t scs_khz, uint32_t slot_index)
{
  uint32_t total = 0U;

  for (uint32_t sym = 0U; sym < 14U; sym++) {
    total += nr_ofdm_symbol_len_fs(fs_hz, scs_khz, slot_index * 14U + sym);
  }

  return total;
}

uint64_t nr_slot_start_samp_in_frame_fs(double fs_hz, uint32_t scs_khz, uint32_t slot_index)
{
  uint64_t total = 0U;

  for (uint32_t slot = 0U; slot < slot_index; slot++) {
    total += nr_slot_len_fs(fs_hz, scs_khz, slot);
  }

  return total;
}

uint64_t nr_frame_len_fs(double fs_hz, uint32_t scs_khz)
{
  const int slots_per_frame = nr_slots_per_frame_from_scs_khz(scs_khz);
  uint64_t total = 0U;

  if (slots_per_frame <= 0) {
    return 0U;
  }

  for (int slot = 0; slot < slots_per_frame; slot++) {
    total += nr_slot_len_fs(fs_hz, scs_khz, (uint32_t)slot);
  }

  return total;
}

static int64_t nr_floor_div_i64(int64_t num, int64_t den)
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

int nr_cell_build_timing_ref(const nr_iq_block_t *blk,
                             const nr_sync_state_t *sync,
                             nr_cell_ctx_t *cell)
{
  const uint32_t scs_khz =
      (cell && cell->type0_css.valid && cell->type0_css.scs_pdcch_khz > 0U)
          ? cell->type0_css.scs_pdcch_khz
          : nr_scs_khz_from_mib(cell ? &cell->mib : NULL);
  const int slots_per_frame = nr_slots_per_frame_from_scs_khz(scs_khz);
  const uint32_t ssb_symbol_in_slot = cell ? (uint32_t)(cell->ssb_start_symbol_abs % 14U) : 0U;
  uint64_t ssb_symbol_offset_in_slot = 0U;
  uint64_t slot_start_abs_samp = 0U;
  uint64_t slot_start_in_frame = 0U;
  int64_t ssb_start_rel_samp = 0;

  if (!blk || !sync || !cell || !cell->valid || !cell->mib.valid || !cell->type0_css.valid) {
    return -1;
  }

  if (slots_per_frame <= 0 || cell->ssb_start_slot >= (uint16_t)slots_per_frame) {
    return -1;
  }

  for (uint32_t sym = 0U; sym < ssb_symbol_in_slot; sym++) {
    ssb_symbol_offset_in_slot +=
        nr_ofdm_symbol_len_fs(blk->fs_hz, scs_khz, (uint32_t)cell->ssb_start_slot * 14U + sym);
  }

  ssb_start_rel_samp = (int64_t)sync->coarse_offset_samp + sync->cum_tracking_shift_samp;
  if (ssb_start_rel_samp < 0 || (uint64_t)ssb_start_rel_samp < ssb_symbol_offset_in_slot) {
    return -1;
  }

  slot_start_abs_samp =
      blk->abs_samp0 + (uint64_t)ssb_start_rel_samp - ssb_symbol_offset_in_slot;
  slot_start_in_frame = nr_slot_start_samp_in_frame_fs(blk->fs_hz, scs_khz, cell->ssb_start_slot);
  if (slot_start_abs_samp < slot_start_in_frame) {
    return -1;
  }

  cell->frame_start_abs_samp = slot_start_abs_samp - slot_start_in_frame;
  cell->timing_ref_fs_hz = blk->fs_hz;
  cell->timing_ref_valid = 1U;
  return 0;
}

int nr_cell_estimate_frame_slot_from_samp(const nr_cell_ctx_t *cell,
                                          uint64_t abs_samp,
                                          uint32_t *frame,
                                          uint32_t *slot,
                                          uint64_t *slot_start_abs_samp)
{
  const uint32_t scs_khz =
      (cell && cell->type0_css.valid && cell->type0_css.scs_pdcch_khz > 0U)
          ? cell->type0_css.scs_pdcch_khz
          : nr_scs_khz_from_mib(cell ? &cell->mib : NULL);
  const int slots_per_frame = nr_slots_per_frame_from_scs_khz(scs_khz);
  const uint64_t frame_len = nr_frame_len_fs(cell ? cell->timing_ref_fs_hz : 0.0, scs_khz);
  int64_t delta = 0;
  int64_t frame_adv = 0;
  uint64_t in_frame = 0U;
  uint32_t cur_slot = 0U;
  uint64_t cur_slot_start = 0U;

  if (!cell || !cell->valid || !cell->timing_ref_valid || !frame || !slot) {
    return -1;
  }

  if (slots_per_frame <= 0 || frame_len == 0U) {
    return -1;
  }

  delta = (int64_t)abs_samp - (int64_t)cell->frame_start_abs_samp;
  frame_adv = nr_floor_div_i64(delta, (int64_t)frame_len);
  in_frame = (uint64_t)(delta - frame_adv * (int64_t)frame_len);

  for (int i = 0; i < slots_per_frame; i++) {
    const uint64_t start = nr_slot_start_samp_in_frame_fs(cell->timing_ref_fs_hz, scs_khz, (uint32_t)i);
    const uint64_t len = nr_slot_len_fs(cell->timing_ref_fs_hz, scs_khz, (uint32_t)i);
    if (in_frame >= start && in_frame < (start + len)) {
      cur_slot = (uint32_t)i;
      cur_slot_start = start;
      break;
    }
  }

  *frame = (uint32_t)(((int64_t)cell->sfn + frame_adv) % 1024LL);
  if ((int32_t)(*frame) < 0) {
    *frame += 1024U;
  }
  *slot = cur_slot;
  if (slot_start_abs_samp) {
    *slot_start_abs_samp = cell->frame_start_abs_samp + (uint64_t)frame_adv * frame_len + cur_slot_start;
  }
  return 0;
}

int nr_get_ssb_start_symbol_abs(uint32_t scs_khz, int32_t nr_band, uint8_t ssb_index)
{
  static const int case_ac[2] = {2, 8};
  static const int case_bd[4] = {4, 8, 16, 20};
  int n = 0;

  switch (scs_khz) {
    case 15U:
      n = ssb_index >> 1;
      return case_ac[ssb_index & 1U] + 14 * n;
    case 30U:
      if (nr_band == 5 || nr_band == 66) {
        n = ssb_index >> 2;
        return case_bd[ssb_index & 3U] + 28 * n;
      }
      if (nr_band >= 0) {
        n = ssb_index >> 1;
        return case_ac[ssb_index & 1U] + 14 * n;
      }
      return -1;
    default:
      return -1;
  }
}
