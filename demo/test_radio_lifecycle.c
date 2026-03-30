#include "common/error.h"
#include "common/log.h"
#include "radio/radio_factory.h"

#include <stdio.h>
#include <string.h>

int main(void)
{
  radio_device_t dev;
  radio_device_caps_t caps;
  radio_stream_config_t rx_stream_cfg;
  radio_stream_config_t tx_stream_cfg;
  int ret;

  log_set_level(LOG_LEVEL_INFO);

  ret = radio_device_init(&dev, RADIO_DEVICE_LIME);
  if (ret != TOA_OK) {
    log_msg(LOG_LEVEL_ERROR, "SELFTEST", "radio_device_init failed, ret=%d", ret);
    return -1;
  }

  ret = dev.dev_open(&dev, "driver=lime");
  if (ret != TOA_OK) {
    log_msg(LOG_LEVEL_ERROR, "SELFTEST", "dev_open failed, ret=%d", ret);
    return -1;
  }

  ret = dev.dev_get_caps(&dev, &caps);
  if (ret == TOA_OK) {
    printf("caps: fs=[%.0f, %.0f] gain=[%.1f, %.1f] rx_ant=%s\n",
           caps.sample_rate_min,
           caps.sample_rate_max,
           caps.gain_min,
           caps.gain_max,
           caps.rx_antennas);
  }

  memset(&rx_stream_cfg, 0, sizeof(rx_stream_cfg));
  rx_stream_cfg.channel_id = 0;
  rx_stream_cfg.buffer_size = 1024;
  ret = dev.trx_setup_rx_stream(&dev, &rx_stream_cfg);
  if (ret != TOA_OK) {
    log_msg(LOG_LEVEL_WARN, "SELFTEST", "trx_setup_rx_stream failed, ret=%d", ret);
  }

  memset(&tx_stream_cfg, 0, sizeof(tx_stream_cfg));
  tx_stream_cfg.channel_id = 0;
  tx_stream_cfg.buffer_size = 1024;
  ret = dev.trx_setup_tx_stream(&dev, &tx_stream_cfg);
  if (ret != TOA_OK) {
    log_msg(LOG_LEVEL_WARN, "SELFTEST", "trx_setup_tx_stream failed, ret=%d", ret);
  } else {
    log_msg(LOG_LEVEL_INFO, "SELFTEST", "tx stream setup success");
  }

  dev.dev_close(&dev);
  log_msg(LOG_LEVEL_INFO, "SELFTEST", "radio lifecycle check done");
  return 0;
}
