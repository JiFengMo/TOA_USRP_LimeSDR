#ifndef IO_IQ_READER_H
#define IO_IQ_READER_H

#include <stdio.h>
#include <stdint.h>

#include "common/cf32.h"
#include "common/error.h"
#include "io/iq_meta.h"

/*
 * One contiguous IQ block as seen by offline pipelines.
 */
typedef struct {
  uint64_t block_index;
  uint64_t sample_index0;
  int64_t hw_time_ns0;
  uint32_t nsamps;
  uint32_t flags;
} iq_block_info_t;

typedef struct {
  FILE *fp;
  iq_meta_t meta;
  int meta_loaded;
  double time_fs_hz;
  uint64_t block_index;
  uint64_t sample_cursor;
  int64_t synthetic_time_ns;
  void *blog_priv; /* internal blocklog table */
  uint32_t blog_cap;
} iq_reader_t;

/*
 * Open IQ file.
 * meta_path: explicit JSON, or NULL to try <stem>.meta.json.
 * blocklog_path: explicit CSV, NULL to try <stem>.blocklog.csv (ignore if missing),
 *                or "" (empty string) to disable blocklog entirely.
 */
toa_error_t iq_reader_open(iq_reader_t *r,
                           const char *iq_path,
                           const char *meta_path,
                           const char *blocklog_path);

void iq_reader_close(iq_reader_t *r);

/*
 * Set sample rate used for synthetic hw_time_ns0 when blocklog is not used (Hz > 0).
 */
void iq_reader_set_time_fs_hz(iq_reader_t *r, double fs_hz);

/*
 * Read up to max_samples. Returns TOA_OK with *nread_out==0 at EOF.
 */
toa_error_t iq_reader_next_block(iq_reader_t *r,
                                 cf32_t *buf,
                                 uint32_t max_samples,
                                 uint32_t *nread_out,
                                 iq_block_info_t *info_out);

/*
 * When fs_from_env==0, try JSON meta (explicit path or <stem>.meta.json) and set *fs_hz_io if present.
 */
toa_error_t iq_replay_fill_fs_from_meta(const char *iq_path,
                                        const char *meta_path_opt,
                                        int fs_from_env,
                                        double *fs_hz_io);

#endif
