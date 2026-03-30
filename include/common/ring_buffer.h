#ifndef COMMON_RING_BUFFER_H
#define COMMON_RING_BUFFER_H

#include <stddef.h>
#include <stdint.h>

typedef struct {
  uint8_t *buf;
  size_t cap;
  size_t head;
  size_t tail;
  size_t size;
} ring_buffer_t;

int ring_buffer_init(ring_buffer_t *rb, uint8_t *mem, size_t cap);
size_t ring_buffer_write(ring_buffer_t *rb, const uint8_t *src, size_t len);
size_t ring_buffer_read(ring_buffer_t *rb, uint8_t *dst, size_t len);

#endif
