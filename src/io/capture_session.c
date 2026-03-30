#include "io/capture_session.h"

#include <string.h>

toa_error_t capture_session_open(capture_session_t *s, const char *base_path_no_ext)
{
  char path[640];

  if (!s || !base_path_no_ext || !base_path_no_ext[0])
    return TOA_ERR_INVALID_ARG;

  memset(s, 0, sizeof(*s));
  snprintf(s->base_path, sizeof(s->base_path), "%s", base_path_no_ext);
  iq_meta_init_default(&s->meta);

  snprintf(path, sizeof(path), "%s.cf32", base_path_no_ext);
  if (iq_recorder_open(&s->iq, path) != 0)
    return TOA_ERR_IO;

  snprintf(path, sizeof(path), "%s.blocklog.csv", base_path_no_ext);
  s->fp_blocklog = fopen(path, "w");
  if (!s->fp_blocklog) {
    iq_recorder_close(&s->iq);
    memset(s, 0, sizeof(*s));
    return TOA_ERR_IO;
  }

  fprintf(s->fp_blocklog, "block_idx,sample_offset,hw_time_ns,nsamps,flags\n");
  fflush(s->fp_blocklog);

  s->opened = 1;
  return TOA_OK;
}

void capture_session_set_meta(capture_session_t *s, const iq_meta_t *m)
{
  if (!s || !m)
    return;
  s->meta = *m;
}

toa_error_t capture_session_write_block(capture_session_t *s,
                                        const cf32_t *buf,
                                        uint32_t nsamps,
                                        int64_t hw_time_ns,
                                        uint32_t flags)
{
  uint64_t sample_off;

  if (!s || !s->opened || !buf || nsamps == 0)
    return TOA_ERR_INVALID_ARG;

  sample_off = s->iq.total_samples;
  if (iq_recorder_write_cf32(&s->iq, buf, nsamps) != 0)
    return TOA_ERR_IO;

  fprintf(s->fp_blocklog,
          "%llu,%llu,%lld,%u,%u\n",
          (unsigned long long)s->block_index,
          (unsigned long long)sample_off,
          (long long)hw_time_ns,
          nsamps,
          flags);
  fflush(s->fp_blocklog);

  s->block_index++;
  s->total_samples_written = s->iq.total_samples;
  return TOA_OK;
}

void capture_session_close(capture_session_t *s)
{
  char path[640];

  if (!s || !s->opened) {
    if (s)
      memset(s, 0, sizeof(*s));
    return;
  }

  if (s->fp_blocklog) {
    fclose(s->fp_blocklog);
    s->fp_blocklog = NULL;
  }

  s->meta.total_samples = s->iq.total_samples;
  snprintf(path, sizeof(path), "%s.meta.json", s->base_path);
  (void)iq_meta_save_json(path, &s->meta);

  iq_recorder_close(&s->iq);
  memset(s, 0, sizeof(*s));
}
