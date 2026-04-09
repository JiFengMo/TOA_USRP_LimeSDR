#include "openair1/PHY/NR_POSITIONING/nr_pos_api.h"

#include <stdint.h>
#include <string.h>

int main(void)
{
  {
    const double fs_hz = 30720000.0;
    if (nr_v0_ssb_symbol_cp_len_fs(fs_hz, 0U) != 88U) {
      return 24;
    }
    if (nr_v0_ssb_symbol_cp_len_fs(fs_hz, 1U) != 72U) {
      return 25;
    }
    if (nr_v0_ssb_burst_len_fs(fs_hz) != 4400U) {
      return 26;
    }
  }

  {
    uint16_t rel[432];
    uint8_t sym[432];
    if (nr_pbch_data_re_positions(1U, rel, sym, 432U) != 432U) {
      return 20;
    }
    for (uint32_t i = 0U; i < 180U; i++) {
      if (sym[i] != 1U) {
        return 21;
      }
    }
    for (uint32_t i = 180U; i < 252U; i++) {
      if (sym[i] != 2U) {
        return 22;
      }
    }
    for (uint32_t i = 252U; i < 432U; i++) {
      if (sym[i] != 3U) {
        return 23;
      }
    }
  }

  uint8_t coded[864];
  float llr[864];
  nr_pss_hit_t hits[4];
  nr_iq_block_t blk;
  nr_sync_state_t sync;
  nr_sync_state_t pbch_only;
  memset(&blk, 0, sizeof(blk));
  memset(&pbch_only, 0, sizeof(pbch_only));

  pbch_only.pci = 1U;
  pbch_only.ssb_index = NR_V0_SYNTH_PBCH_SSB_IDX;
  if (nr_pbch_bch_encode(pbch_only.pci, NR_V0_SYNTH_PBCH_SSB_IDX,
                         NR_V0_SYNTH_PBCH_SFN, NR_V0_SYNTH_PBCH_HRF,
                         NR_V0_SYNTH_PBCH_MIB_PAYLOAD, coded, 864U) != 864) {
    return 17;
  }
  for (uint32_t i = 0; i < 864U; i++) {
    llr[i] = coded[i] ? -20.0f : 20.0f;
  }
  if (nr_pbch_bch_decode(llr, 864U, &pbch_only) != 0) {
    return 18;
  }
  if (!pbch_only.mib_ok || pbch_only.sfn != NR_V0_SYNTH_PBCH_SFN ||
      pbch_only.mib_payload != NR_V0_SYNTH_PBCH_MIB_PAYLOAD) {
    return 19;
  }

  const uint32_t burst_len = nr_v0_ssb_burst_len();
  blk.nsamps = 8192;
  blk.fs_hz = 30720000.0;
  c16_t buf[8192];
  memset(buf, 0, sizeof(buf));
  const int ins = 100;
  (void)nr_v0_ssb_build_burst_iq(0, 1, &buf[ins], burst_len, 3500.0f);
  blk.rx[0] = buf;

  /* Positive case */
  if (nr_ssb_pss_search(&blk, hits, 4) != 0) {
    return 1;
  }
  if (hits[0].nid2 != 1U) {
    return 2;
  }
  if (hits[0].metric < 0.12f) {
    return 3;
  }
  if (hits[0].peak_samp < (ins - 2) || hits[0].peak_samp > (ins + 2)) {
    return 4;
  }
  if (nr_ssb_refine_sync(&blk, &hits[0], &sync) != 0) {
    return 14;
  }
  if (sync.locked) {
    return 5;
  }
  if (sync.pci > 1007U) {
    return 9;
  }
  if (sync.pbch_ok || sync.pbch_confirmed || sync.mib_ok) {
    return 10;
  }
  if (nr_ssb_pbch_decode(&blk, &sync) != 0) {
    return 11;
  }
  if (!sync.pbch_ok || sync.pbch_confirmed || !sync.mib_ok) {
    return 12;
  }
  if (sync.ssb_index != NR_V0_SYNTH_PBCH_SSB_IDX) {
    return 13;
  }
  if (sync.sfn != NR_V0_SYNTH_PBCH_SFN) {
    return 15;
  }
  if (sync.mib_payload != NR_V0_SYNTH_PBCH_MIB_PAYLOAD) {
    return 16;
  }

  /* Negative case: pure zero/noise must fail */
  memset(buf, 0, sizeof(buf));
  if (nr_ssb_pss_search(&blk, hits, 4) == 0) {
    return 6;
  }

  /* Negative case: wrong nid2 waveform should not claim nid2=1 as best */
  memset(buf, 0, sizeof(buf));
  (void)nr_v0_ssb_build_burst_iq(0, 2, &buf[ins], burst_len, 3500.0f);
  if (nr_ssb_pss_search(&blk, hits, 4) != 0) {
    return 7;
  }
  if (hits[0].nid2 == 1U) {
    return 8;
  }
  return 0;
}
