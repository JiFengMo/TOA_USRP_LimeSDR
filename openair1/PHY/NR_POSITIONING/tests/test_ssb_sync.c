#include "openair1/PHY/NR_POSITIONING/nr_pos_api.h"

#include <stdlib.h>
#include <string.h>

int main(void)
{
  nr_pss_hit_t hits[4];
  nr_iq_block_t blk;
  nr_sync_state_t sync;
  memset(&blk, 0, sizeof(blk));
  blk.nsamps = 256;
  c16_t buf[256];
  memset(buf, 0, sizeof(buf));
  for (int i = 64; i < 96; i++) {
    buf[i].r = 12000;
    buf[i].i = 0;
  }
  blk.rx[0] = buf;
  if (nr_ssb_pss_search(&blk, hits, 4) != 0) {
    return 1;
  }
  if (nr_ssb_refine_sync(&blk, &hits[0], &sync) != 0) {
    return 1;
  }
  return 0;
}
