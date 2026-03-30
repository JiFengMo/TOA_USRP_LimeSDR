#ifndef IO_CAPTURE_SESSION_H
#define IO_CAPTURE_SESSION_H

#include <stdio.h>
#include <stdint.h>

#include "common/cf32.h"
#include "common/error.h"
#include "io/iq_meta.h"
#include "recorder/iq_recorder.h"

/*
 * Session: <base>.cf32 + <base>.blocklog.csv + <base>.meta.json (written on close).
 * base_path_no_ext example: "rx_iq" -> rx_iq.cf32
 */
typedef struct {
  iq_recorder_t iq;
  FILE *fp_blocklog;
  char base_path[512];
  iq_meta_t meta;
  uint64_t block_index;
  uint64_t total_samples_written;
  int opened;
} capture_session_t;

toa_error_t capture_session_open(capture_session_t *s, const char *base_path_no_ext);

void capture_session_set_meta(capture_session_t *s, const iq_meta_t *m);

/*
 * hw_time_ns: device timestamp for block start (e.g. radio_rx_result_t.timestamp_ns).
 */
toa_error_t capture_session_write_block(capture_session_t *s,
                                        const cf32_t *buf,
                                        uint32_t nsamps,
                                        int64_t hw_time_ns,
                                        uint32_t flags);

void capture_session_close(capture_session_t *s);

#endif
