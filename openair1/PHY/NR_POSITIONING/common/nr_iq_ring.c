#include "openair1/PHY/NR_POSITIONING/nr_pos_types.h"

#include <stdlib.h>
#include <string.h>

#define NR_IQ_RING_POOL_SPARE_BLOCKS 8U

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

static nr_iq_block_t *nr_iq_block_create(uint32_t nsamps, uint8_t rx_ant)
{
  if (nsamps == 0) {
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
  blk->rx_ant = rx_ant;
  for (int a = 0; a < rx_ant; a++) {
    blk->rx[a] = (c16_t *)calloc(nsamps, sizeof(c16_t));
    if (!blk->rx[a]) {
      nr_iq_block_destroy(blk);
      return NULL;
    }
  }
  return blk;
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
  pthread_mutex_init(&rb->pool_mtx, NULL);
  return 0;
}

int nr_iq_ring_prealloc(nr_iq_ring_t *rb, uint32_t nsamps, uint8_t rx_ant)
{
  if (!rb || rb->depth <= 0 || nsamps == 0) {
    return -1;
  }
  if (rx_ant == 0) {
    rx_ant = 1;
  }
  if (rx_ant > NR_TOA_MAX_RX_ANT) {
    rx_ant = NR_TOA_MAX_RX_ANT;
  }
  if (rb->pool_ready) {
    return (rb->block_nsamps == nsamps && rb->block_rx_ant == rx_ant) ? 0 : -1;
  }

  rb->pool_depth = (uint32_t)rb->depth + NR_IQ_RING_POOL_SPARE_BLOCKS;
  rb->pool_blocks = (nr_iq_block_t **)calloc(rb->pool_depth, sizeof(*rb->pool_blocks));
  rb->free_stack = (nr_iq_block_t **)calloc(rb->pool_depth, sizeof(*rb->free_stack));
  if (!rb->pool_blocks || !rb->free_stack) {
    free(rb->pool_blocks);
    free(rb->free_stack);
    rb->pool_blocks = NULL;
    rb->free_stack = NULL;
    rb->pool_depth = 0;
    return -1;
  }

  rb->block_nsamps = nsamps;
  rb->block_rx_ant = rx_ant;
  for (uint32_t i = 0; i < rb->pool_depth; i++) {
    nr_iq_block_t *blk = nr_iq_block_create(nsamps, rx_ant);
    if (!blk) {
      for (uint32_t k = 0; k < i; k++) {
        nr_iq_block_destroy(rb->pool_blocks[k]);
      }
      free(rb->pool_blocks);
      free(rb->free_stack);
      rb->pool_blocks = NULL;
      rb->free_stack = NULL;
      rb->pool_depth = 0;
      rb->free_count = 0;
      rb->block_nsamps = 0;
      rb->block_rx_ant = 0;
      return -1;
    }
    blk->owner = rb;
    blk->from_pool = 1U;
    blk->refcnt = 0;
    rb->pool_blocks[i] = blk;
    rb->free_stack[i] = blk;
  }
  rb->free_count = rb->pool_depth;
  rb->pool_ready = 1U;
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

  if (rb->pool_ready && rb->block_nsamps == nsamps && rb->block_rx_ant == rx_ant) {
    pthread_mutex_lock(&rb->pool_mtx);
    if (rb->free_count > 0U) {
      nr_iq_block_t *blk = rb->free_stack[--rb->free_count];
      rb->free_stack[rb->free_count] = NULL;
      blk->ts_first = 0;
      blk->abs_samp0 = 0;
      blk->fs_hz = 0.0;
      blk->rx_freq_hz = 0.0;
      blk->rx_gscn = -1;
      blk->target_pci = -1;
      blk->scan_target_idx = 0;
      blk->overrun = 0;
      blk->refcnt = 1;
      pthread_mutex_unlock(&rb->pool_mtx);
      return blk;
    }
    rb->alloc_fallback_cnt++;
    pthread_mutex_unlock(&rb->pool_mtx);
  }

  nr_iq_block_t *blk = nr_iq_block_create(nsamps, rx_ant);
  if (!blk) {
    return NULL;
  }
  blk->owner = NULL;
  blk->from_pool = 0U;
  blk->refcnt = 1;
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
  if (blk->from_pool && blk->owner) {
    pthread_mutex_lock(&blk->owner->pool_mtx);
    blk->refcnt++;
    pthread_mutex_unlock(&blk->owner->pool_mtx);
  } else {
    blk->refcnt++;
  }
}

void nr_iq_block_put(nr_iq_block_t *blk)
{
  if (!blk) {
    return;
  }
  if (blk->from_pool && blk->owner) {
    nr_iq_ring_t *rb = blk->owner;
    pthread_mutex_lock(&rb->pool_mtx);
    if (blk->refcnt > 0) {
      blk->refcnt--;
    }
    if (blk->refcnt == 0 && rb->free_count < rb->pool_depth) {
      rb->free_stack[rb->free_count++] = blk;
    }
    pthread_mutex_unlock(&rb->pool_mtx);
  } else {
    if (blk->refcnt > 0) {
      blk->refcnt--;
    }
    if (blk->refcnt == 0) {
      nr_iq_block_destroy(blk);
    }
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
  if (rb->pool_blocks) {
    for (uint32_t i = 0; i < rb->pool_depth; i++) {
      nr_iq_block_destroy(rb->pool_blocks[i]);
      rb->pool_blocks[i] = NULL;
    }
    free(rb->pool_blocks);
    rb->pool_blocks = NULL;
  }
  free(rb->free_stack);
  rb->free_stack = NULL;
  pthread_mutex_destroy(&rb->pool_mtx);
  memset(rb, 0, sizeof(*rb));
}
