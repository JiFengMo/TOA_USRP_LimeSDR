#include "openair1/PHY/NR_POSITIONING/nr_pos_types.h"

#include <string.h>

int nr_ssb_pss_search(const nr_iq_block_t *blk, nr_pss_hit_t *hits, int max_hits)
{
  if (!blk || !hits || max_hits <= 0) {
    return -1;
  }
  memset(hits, 0, sizeof(nr_pss_hit_t) * (size_t)max_hits);
  return 0;
}

int nr_ssb_refine_sync(const nr_iq_block_t *blk, const nr_pss_hit_t *hit,
                       nr_sync_state_t *sync)
{
  if (!blk || !hit || !sync) {
    return -1;
  }
  (void)memset(sync, 0, sizeof(*sync));
  return 0;
}

int nr_ssb_pbch_decode(const nr_iq_block_t *blk, nr_sync_state_t *sync)
{
  if (!blk || !sync) {
    return -1;
  }
  return 0;
}

int nr_ssb_track_cfo(const nr_iq_block_t *blk, nr_sync_state_t *sync)
{
  if (!blk || !sync) {
    return -1;
  }
  return 0;
}

int nr_ssb_track_timing(const nr_iq_block_t *blk, nr_sync_state_t *sync,
                        int *sample_shift)
{
  if (!blk || !sync || !sample_shift) {
    return -1;
  }
  *sample_shift = 0;
  return 0;
}

int nr_ssb_check_lost_lock(const nr_sync_state_t *sync)
{
  if (!sync) {
    return 1;
  }
  return sync->locked ? 0 : 1;
}
