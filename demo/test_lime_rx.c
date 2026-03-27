#include "radio_factory.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int main(void)
{
  radio_device_t device;
  radio_channel_config_t rx_cfg;
  radio_stream_config_t rx_stream_cfg;
  radio_rx_result_t rx_result;

  cf32_t *rx_buffer = NULL;
  void *buffs[1];

  int ret;
  int i;

  /* 初始化统一设备对象 */
  ret = radio_device_init(&device, RADIO_DEVICE_LIME);
  if (ret != 0) {
    printf("radio_device_init failed, ret=%d\n", ret);
    return -1;
  }

  /* 打开 LimeSDR */
  ret = device.dev_open(&device, "driver=lime");
  if (ret != 0) {
    printf("device open failed, ret=%d\n", ret);
    return -1;
  }

  printf("device name: %s\n", device.name);

  /* 配置 RX 参数 */
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

  /* 启动 RX 流 */
  ret = device.trx_start_rx_stream(&device);
  if (ret != 0) {
    printf("trx_start_rx_stream failed, ret=%d\n", ret);
    device.dev_close(&device);
    return -1;
  }

  /* 分配 RX 缓冲区 */
  rx_buffer = (cf32_t *)malloc(sizeof(cf32_t) * rx_stream_cfg.buffer_size);
  if (!rx_buffer) {
    printf("malloc rx_buffer failed\n");
    device.trx_stop_rx_stream(&device);
    device.dev_close(&device);
    return -1;
  }

  buffs[0] = rx_buffer;

  /* 连续读取几次 IQ */
  for (i = 0; i < 10; i++) {
    memset(&rx_result, 0, sizeof(rx_result));

    ret = device.trx_read_func(&device,
                               buffs,
                               rx_stream_cfg.buffer_size,
                               &rx_result,
                               100000);

    printf("[RX] iter=%d ret=%d samples=%d flags=%d ts=%lld\n",
           i,
           ret,
           rx_result.samples_read,
           rx_result.flags,
           (long long)rx_result.timestamp_ns);
  }

  /* 清理资源 */
  free(rx_buffer);
  device.trx_stop_rx_stream(&device);
  device.dev_close(&device);

  return 0;
}