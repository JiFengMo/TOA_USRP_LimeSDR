#include "pipeline/offline_runner.h"

#include <stdlib.h>

toa_error_t offline_run_cf32_file(const char *iq_path,
                                  const char *meta_path,
                                  const char *blocklog_path,
                                  uint32_t chunk_samples,
                                  double time_fs_hz,
                                  void *userdata,
                                  offline_iq_block_fn cb)
{
  iq_reader_t rd;
  cf32_t *buf = NULL;
  toa_error_t err;
  toa_error_t cberr = TOA_OK;
  uint32_t nread;
  double fs;

  if (!iq_path || chunk_samples == 0 || !cb)
    return TOA_ERR_INVALID_ARG;

  err = iq_reader_open(&rd, iq_path, meta_path, blocklog_path);
  if (err != TOA_OK)
    return err;

  fs = time_fs_hz;
  if (fs <= 0.0 && iq_meta_has_sample_rate(&rd.meta))
    fs = rd.meta.sample_rate_hz;
  if (fs <= 0.0) {
    iq_reader_close(&rd);
    return TOA_ERR_INVALID_ARG;
  }
  iq_reader_set_time_fs_hz(&rd, fs);

  buf = (cf32_t *)malloc(sizeof(cf32_t) * chunk_samples);
  if (!buf) {
    iq_reader_close(&rd);
    return TOA_ERR_NO_MEMORY;
  }

  for (;;) {
    iq_block_info_t info;

    err = iq_reader_next_block(&rd, buf, chunk_samples, &nread, &info);
    if (err != TOA_OK) {
      cberr = err;
      break;
    }
    if (nread == 0)
      break;

    cberr = cb(userdata, buf, nread, &info);
    if (cberr != TOA_OK)
      break;
  }

  free(buf);
  iq_reader_close(&rd);
  return cberr;
}
