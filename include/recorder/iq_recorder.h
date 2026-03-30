#ifndef IQ_RECORDER_H
#define IQ_RECORDER_H

#include <stdio.h>
#include <stdint.h>

#include "common/cf32.h"

/* IQ 录制�? */
typedef struct {
  FILE *fp;                  /* 文件指针 */
  char filename[256];        /* 文件�? */
  uint64_t total_samples;    /* 已保存样点数 */
} iq_recorder_t;

/* 打开录制文件 */
int iq_recorder_open(iq_recorder_t *rec, const char *filename);

/* 写入一�? cf32 IQ 数据 */
int iq_recorder_write_cf32(iq_recorder_t *rec, const cf32_t *buff, uint32_t nsamps);

/* 关闭录制文件 */
void iq_recorder_close(iq_recorder_t *rec);

#endif