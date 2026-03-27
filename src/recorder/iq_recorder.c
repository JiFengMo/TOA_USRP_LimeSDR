#include "recorder/iq_recorder.h"

#include <stdio.h>
#include <string.h>

/* 打开录制文件 */
int iq_recorder_open(iq_recorder_t *rec, const char *filename)
{
  if (!rec || !filename)
    return -1;

  memset(rec, 0, sizeof(*rec));

  snprintf(rec->filename, sizeof(rec->filename), "%s", filename);

  /* 用二进制写方式打开 */
  rec->fp = fopen(filename, "wb");
  if (!rec->fp)
    return -2;

  rec->total_samples = 0;
  return 0;
}

/* 写入 cf32 数据 */
int iq_recorder_write_cf32(iq_recorder_t *rec, const cf32_t *buff, uint32_t nsamps)
{
  size_t nwrite;

  if (!rec || !rec->fp || !buff)
    return -1;

  nwrite = fwrite(buff, sizeof(cf32_t), nsamps, rec->fp);
  if (nwrite != nsamps)
    return -2;

  rec->total_samples += nsamps;
  return 0;
}

/* 关闭文件 */
void iq_recorder_close(iq_recorder_t *rec)
{
  if (!rec)
    return;

  if (rec->fp) {
    fclose(rec->fp);
    rec->fp = NULL;
  }
}