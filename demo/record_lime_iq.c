#include "radio/radio_factory.h"
#include "recorder/iq_recorder.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int main(void)
{
  //定义一些结构体
  radio_device_t device;//设备相关的结构体
  radio_channel_config_t rx_cfg;//信道相关的参数
  radio_stream_config_t rx_stream_cfg;//接受流控制相关参数
  radio_rx_result_t rx_result;//接受结果标志（并非存储）
  iq_recorder_t recorder;//结果记录结构体

  cf32_t *rx_buffer = NULL;//存储缓存
  void *buffs[1];

  int ret;
  int i;
  int num_blocks = 2000; /* 录多少个块，可自行调 */

  /* 初始化设备对象 */
  ret = radio_device_init(&device, RADIO_DEVICE_LIME);
  if (ret != 0) {
    printf("radio_device_init failed, ret=%d\n", ret);
    return -1;
  }

  /* 打开设备 */
  ret = device.dev_open(&device, "driver=lime");
  if (ret != 0) {
    printf("device open failed, ret=%d\n", ret);
    return -1;
  }

  printf("device name: %s\n", device.name);

  /* 配置 RX */
  memset(&rx_cfg, 0, sizeof(rx_cfg));
  rx_cfg.channel_id = 0;
  rx_cfg.sample_rate = 5e6;
  rx_cfg.center_freq = 2.45e9;
  rx_cfg.bandwidth = 5e6;
  rx_cfg.gain = 40.0;
  snprintf(rx_cfg.antenna, sizeof(rx_cfg.antenna), "LNAH");

  ret = device.trx_configure_rx(&device, &rx_cfg);
  if (ret != 0) {
    printf("trx_configure_rx failed, ret=%d\n", ret);
    device.dev_close(&device);
    return -1;
  }

  /* 配置 RX 流 */
  memset(&rx_stream_cfg, 0, sizeof(rx_stream_cfg));
  rx_stream_cfg.channel_id = 0;
  rx_stream_cfg.buffer_size = 4096;

  ret = device.trx_setup_rx_stream(&device, &rx_stream_cfg);
  if (ret != 0) {
    printf("trx_setup_rx_stream failed, ret=%d\n", ret);
    device.dev_close(&device);
    return -1;
  }

  /* 启动 RX */
  ret = device.trx_start_rx_stream(&device);
  if (ret != 0) {
    printf("trx_start_rx_stream failed, ret=%d\n", ret);
    device.dev_close(&device);
    return -1;
  }

  /* 分配缓冲区 */
  rx_buffer = (cf32_t *)malloc(sizeof(cf32_t) * rx_stream_cfg.buffer_size);
  if (!rx_buffer) {
    printf("malloc rx_buffer failed\n");
    device.trx_stop_rx_stream(&device);
    device.dev_close(&device);
    return -1;
  }

  buffs[0] = rx_buffer;

  /* 打开保存文件 */
  ret = iq_recorder_open(&recorder, "rx_iq.cf32");
  if (ret != 0) {
    printf("iq_recorder_open failed, ret=%d\n", ret);
    free(rx_buffer);
    device.trx_stop_rx_stream(&device);
    device.dev_close(&device);
    return -1;
  }

  printf("start recording...\n");

  /* 循环录制 */
  for (i = 0; i < num_blocks; i++) {
    memset(&rx_result, 0, sizeof(rx_result));

    ret = device.trx_read_func(&device,
                               buffs,
                               rx_stream_cfg.buffer_size,
                               &rx_result,
                               100000);

    if (ret != 0) {
      printf("trx_read_func failed at block=%d, ret=%d\n", i, ret);
      break;
    }

    /* 只把真正读到的样点写入文件 */
    if (rx_result.samples_read > 0) {
      ret = iq_recorder_write_cf32(&recorder,
                                   rx_buffer,
                                   (uint32_t)rx_result.samples_read);
      if (ret != 0) {
        printf("iq_recorder_write_cf32 failed at block=%d, ret=%d\n", i, ret);
        break;
      }
    }

    if ((i % 100) == 0) {
      printf("block=%d total_samples=%llu\n",
             i,
             (unsigned long long)recorder.total_samples);
    }
  }

  printf("record finished, total_samples=%llu\n",
         (unsigned long long)recorder.total_samples);

  iq_recorder_close(&recorder);
  free(rx_buffer);
  device.trx_stop_rx_stream(&device);
  device.dev_close(&device);

  return 0;
}