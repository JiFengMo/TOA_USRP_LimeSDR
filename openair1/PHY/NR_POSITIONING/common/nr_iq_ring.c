#include "openair1/PHY/NR_POSITIONING/nr_pos_types.h"

#include <stdlib.h>
#include <string.h>

static void nr_iq_block_destroy(nr_iq_block_t *blk)
{
  if (!blk) {
    return;
  }
  for (int a = 0; a < NR_TOA_MAX_RX_ANT; a++) {
    free(blk->rx[a]);
    blk->rx[a] = NULL;
  }
  free(blk);
}

int nr_iq_ring_init(nr_iq_ring_t *rb, int depth)
{
  if (!rb || depth <= 0) {
    return -1;
  }
  memset(rb, 0, sizeof(*rb));
  rb->depth = depth;
  rb->blocks = (nr_iq_block_t **)calloc((size_t)depth, sizeof(nr_iq_block_t *));
  if (!rb->blocks) {
    return -1;
  }
  rb->head = 0;
  rb->tail = 0;
  rb->count = 0;
  return 0;
}

nr_iq_block_t *nr_iq_ring_alloc_ex(nr_iq_ring_t *rb, uint32_t nsamps, uint8_t rx_ant)
{
  if (!rb || nsamps == 0) {
    return NULL;
  }
  if (rx_ant == 0) {
    rx_ant = 1;
  }
  if (rx_ant > NR_TOA_MAX_RX_ANT) {
    rx_ant = NR_TOA_MAX_RX_ANT;
  }

  nr_iq_block_t *blk = (nr_iq_block_t *)calloc(1, sizeof(nr_iq_block_t));
  if (!blk) {
    return NULL;
  }
  blk->nsamps = nsamps;
  blk->refcnt = 1;
  blk->rx_ant = rx_ant;
  blk->abs_samp0 = 0;

  /* Allocate per-antenna complex16 sample buffers. */
  for (int a = 0; a < rx_ant; a++) {
    blk->rx[a] = (c16_t *)calloc(nsamps, sizeof(c16_t));
    if (!blk->rx[a]) {
      /* Free partially allocated buffers */
      for (int k = 0; k < a; k++) {
        free(blk->rx[k]);
        blk->rx[k] = NULL;
      }
      free(blk);
      return NULL;
    }
  }
  return blk;
}

nr_iq_block_t *nr_iq_ring_alloc(nr_iq_ring_t *rb, uint32_t nsamps)
{
  return nr_iq_ring_alloc_ex(rb, nsamps, NR_TOA_MAX_RX_ANT);
}

void nr_iq_block_get(nr_iq_block_t *blk)
{
  if (!blk) {
    return;
  }
  blk->refcnt++;
}

void nr_iq_block_put(nr_iq_block_t *blk)
{
  if (!blk) {
    return;
  }
  if (blk->refcnt > 0) {
    blk->refcnt--;
  }
  if (blk->refcnt == 0) {
    nr_iq_block_destroy(blk);
  }
}

void nr_iq_ring_push(nr_iq_ring_t *rb, nr_iq_block_t *blk)
{
  if (!rb || !blk || rb->depth <= 0) {
    return;
  }

  /* Replace oldest when full. */
  if (rb->count == (uint32_t)rb->depth) {
    /* Release the ring's reference on eviction. */
    nr_iq_block_put(rb->blocks[rb->head]);
    rb->blocks[rb->head] = blk;
    rb->head = (rb->head + 1U) % (uint32_t)rb->depth;
    rb->tail = rb->head;
    /* count stays == depth */
    rb->overrun_cnt++;
    return;
  }

  rb->blocks[rb->tail] = blk;
  rb->tail = (rb->tail + 1U) % (uint32_t)rb->depth;
  rb->count++;
}

nr_iq_block_t *nr_iq_ring_get_window(nr_iq_ring_t *rb, uint64_t abs_samp0,
                                     uint32_t len)
{
  if (!rb || rb->count == 0 || !rb->blocks) {
    return NULL;
  }
  if (len == 0) {
    return NULL;
  }

  uint32_t idx = rb->head;
  for (uint32_t c = 0; c < rb->count; c++) {
    nr_iq_block_t *blk = rb->blocks[idx];
    if (blk) {
      uint64_t blk_start = blk->abs_samp0;
      uint64_t blk_end = blk_start + (uint64_t)blk->nsamps;
      /* Hit if requested window fits inside this block. */
      if (abs_samp0 >= blk_start && (uint64_t)abs_samp0 + (uint64_t)len <= blk_end) {
        /* Transfer one reference to caller. */
        nr_iq_block_get(blk);
        return blk;
      }
    }
    idx = (idx + 1U) % (uint32_t)rb->depth;
  }

  return NULL;
}

void nr_iq_ring_free(nr_iq_ring_t *rb)
{
  if (!rb || rb->depth <= 0) {
    return;
  }
  if (rb->blocks) {
    for (int i = 0; i < rb->depth; i++) {
      nr_iq_block_put(rb->blocks[i]);
      rb->blocks[i] = NULL;
    }
    free(rb->blocks);
    rb->blocks = NULL;
  }
  memset(rb, 0, sizeof(*rb));
}
