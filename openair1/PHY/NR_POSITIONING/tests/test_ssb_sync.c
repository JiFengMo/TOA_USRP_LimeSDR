#include "openair1/PHY/NR_POSITIONING/nr_pos_api.h"

#include <stdint.h>
#include <string.h>

int main(void)
{
  nr_pss_hit_t hits[4];
  nr_iq_block_t blk;
  nr_sync_state_t sync;
  memset(&blk, 0, sizeof(blk));

  const uint32_t burst_len = nr_v0_ssb_burst_len();
  blk.nsamps = 2048;
  c16_t buf[2048];
  memset(buf, 0, sizeof(buf));
  const int ins = 100;
  (void)nr_v0_ssb_build_burst_iq(0, 1, &buf[ins], burst_len, 12000.0f);
  blk.rx[0] = buf;

  /* Positive case */
  if (nr_ssb_pss_search(&blk, hits, 4) != 0) {
    return 1;
  }
  if (hits[0].nid2 != 1U) {
    return 2;
  }
  if (hits[0].metric < 0.80f) {
    return 3;
  }
  if (hits[0].peak_samp < (ins - 2) || hits[0].peak_samp > (ins + 2)) {
    return 4;
  }
  if (nr_ssb_refine_sync(&blk, &hits[0], &sync) != 0) {
    return 1;
  }
  if (!sync.locked || sync.pci != 1U) {
    return 5;
  }

  /* Negative case: pure zero/noise must fail */
  memset(buf, 0, sizeof(buf));
  if (nr_ssb_pss_search(&blk, hits, 4) == 0) {
    return 6;
  }

  /* Negative case: wrong nid2 waveform should not claim nid2=1 as best */
  memset(buf, 0, sizeof(buf));
  (void)nr_v0_ssb_build_burst_iq(0, 2, &buf[ins], burst_len, 12000.0f);
  if (nr_ssb_pss_search(&blk, hits, 4) != 0) {
    return 7;
  }
  if (hits[0].nid2 == 1U) {
    return 8;
  }
  return 0;
}
