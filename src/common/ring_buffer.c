#include "common/ring_buffer.h"

int ring_buffer_init(ring_buffer_t *rb, uint8_t *mem, size_t cap)
{
  if (!rb || !mem || cap == 0)
    return -1;

  rb->buf = mem;
  rb->cap = cap;
  rb->head = 0;
  rb->tail = 0;
  rb->size = 0;
  return 0;
}

size_t ring_buffer_write(ring_buffer_t *rb, const uint8_t *src, size_t len)
{
  size_t i;

  if (!rb || !src)
    return 0;

  for (i = 0; i < len && rb->size < rb->cap; ++i) {
    rb->buf[rb->head] = src[i];
    rb->head = (rb->head + 1) % rb->cap;
    rb->size++;
  }
  return i;
}

size_t ring_buffer_read(ring_buffer_t *rb, uint8_t *dst, size_t len)
{
  size_t i;

  if (!rb || !dst)
    return 0;

  for (i = 0; i < len && rb->size > 0; ++i) {
    dst[i] = rb->buf[rb->tail];
    rb->tail = (rb->tail + 1) % rb->cap;
    rb->size--;
  }
  return i;
}
