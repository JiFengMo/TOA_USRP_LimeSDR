#ifndef IO_IQ_META_H
#define IO_IQ_META_H

#include <stdint.h>

#include "common/error.h"

#define IQ_META_DEVICE_ARGS_MAX 128

/*
 * Capture / replay metadata (JSON on disk, see iq_meta_save_json / iq_meta_load_json).
 */
typedef struct {
  uint32_t format_version;
  double sample_rate_hz;
  double center_freq_hz;
  double bandwidth_hz;
  double gain_db;
  uint32_t block_size;
  int64_t capture_start_time_ns;
  char device_args[IQ_META_DEVICE_ARGS_MAX];
  /* Filled when closing a session (optional in JSON). */
  uint64_t total_samples;
} iq_meta_t;

void iq_meta_init_default(iq_meta_t *m);

/* Return 1 if sample_rate_hz looks configured. */
int iq_meta_has_sample_rate(const iq_meta_t *m);

toa_error_t iq_meta_load_json(const char *path, iq_meta_t *m);

toa_error_t iq_meta_save_json(const char *path, const iq_meta_t *m);

#endif
