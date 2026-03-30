#include "openair1/PHY/NR_POSITIONING/nr_pos_types.h"

#include <stdlib.h>
#include <string.h>

int nr_iq_ring_init(nr_iq_ring_t *rb, int depth)
{
  if (!rb || depth <= 0) {
    return -1;
  }
  memset(rb, 0, sizeof(*rb));
  rb->depth = depth;
  return 0;
}

nr_iq_block_t *nr_iq_ring_alloc(nr_iq_ring_t *rb, uint32_t nsamps)
{
  (void)rb;
  nr_iq_block_t *blk = (nr_iq_block_t *)calloc(1, sizeof(nr_iq_block_t));
  if (!blk) {
    return NULL;
  }
  blk->nsamps = nsamps;
  blk->refcnt = 1;
  return blk;
}

void nr_iq_ring_push(nr_iq_ring_t *rb, nr_iq_block_t *blk)
{
  (void)rb;
  (void)blk;
}

nr_iq_block_t *nr_iq_ring_get_window(nr_iq_ring_t *rb, uint64_t abs_samp0,
                                     uint32_t len)
{
  (void)rb;
  (void)abs_samp0;
  (void)len;
  return NULL;
}
