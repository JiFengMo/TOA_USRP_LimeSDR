#include "radio/radio_factory.h"
#include "common/cli.h"
#include "common/demo_rx_defaults.h"
#include "common/error.h"
#include "common/log.h"
#include "io/capture_session.h"
#include "io/iq_meta.h"
#include "nr/gscn.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int main(void)
{
  radio_device_t device;
  radio_device_caps_t caps;
  radio_channel_config_t rx_cfg;
  radio_stream_config_t rx_stream_cfg;
  radio_rx_result_t rx_result;
  capture_session_t session;
  iq_meta_t meta;
  cf32_t *rx_buffer = NULL;
  void *buffs[1];
  const char *session_base = "rx_iq";
  int ret, rc = -1;
  int i;
  int num_blocks = 2000;
  int session_opened = 0;
  uint32_t record_gscn = 0u;
  int have_record_gscn = 0;

  memset(&device, 0, sizeof(device));
  memset(&session, 0, sizeof(session));
  log_set_level(LOG_LEVEL_INFO);

  if (getenv("RECORD_SESSION_BASE"))
    session_base = getenv("RECORD_SESSION_BASE");
  {
    uint32_t nb = 0;
    if (cli_env_u32("RECORD_NUM_BLOCKS", &nb) == 0 && nb > 0u && nb < 1000000u)
      num_blocks = (int)nb;
  }
  if (cli_env_u32("RECORD_GSCN", &record_gscn) == 0)
    have_record_gscn = 1;

  ret = radio_device_init(&device, RADIO_DEVICE_LIME);
  if (ret != TOA_OK) {
    log_msg(LOG_LEVEL_ERROR, "RECORD", "radio_device_init failed, ret=%d", ret);
    goto out;
  }

  ret = device.dev_open(&device, "driver=lime");
  if (ret != TOA_OK) {
    log_msg(LOG_LEVEL_ERROR, "RECORD", "dev_open failed, ret=%d", ret);
    goto out;
  }

  log_msg(LOG_LEVEL_INFO, "RECORD", "device name: %s", device.name);
  {
    int caps_ok = (device.dev_get_caps && device.dev_get_caps(&device, &caps) == TOA_OK);
    if (caps_ok) {
      log_msg(LOG_LEVEL_INFO,
              "RECORD",
              "caps fs=[%.0f, %.0f] fc=[%.0f, %.0f] gain=[%.1f, %.1f]",
              caps.sample_rate_min,
              caps.sample_rate_max,
              caps.center_freq_min,
              caps.center_freq_max,
              caps.gain_min,
              caps.gain_max);
    }

    memset(&rx_cfg, 0, sizeof(rx_cfg));
    rx_cfg.channel_id = 0;
    rx_cfg.sample_rate = DEMO_RX_SAMPLE_RATE_HZ;
    rx_cfg.center_freq = DEMO_RX_CENTER_FREQ_HZ;
    rx_cfg.bandwidth = DEMO_RX_BANDWIDTH_HZ;
    rx_cfg.gain = DEMO_RX_GAIN_DB;
    (void)cli_env_f64("RECORD_FS", &rx_cfg.sample_rate);
    (void)cli_env_f64("RECORD_FC", &rx_cfg.center_freq);
    (void)cli_env_f64("RECORD_BW", &rx_cfg.bandwidth);
    (void)cli_env_f64("RECORD_GAIN_DB", &rx_cfg.gain);
    if (caps_ok) {
      if (rx_cfg.sample_rate < caps.sample_rate_min)
        rx_cfg.sample_rate = caps.sample_rate_min;
      if (rx_cfg.sample_rate > caps.sample_rate_max)
        rx_cfg.sample_rate = caps.sample_rate_max;
      if (rx_cfg.bandwidth < caps.bandwidth_min)
        rx_cfg.bandwidth = caps.bandwidth_min;
      if (rx_cfg.bandwidth > caps.bandwidth_max)
        rx_cfg.bandwidth = caps.bandwidth_max;
      if (rx_cfg.center_freq < caps.center_freq_min)
        rx_cfg.center_freq = caps.center_freq_min;
      if (rx_cfg.center_freq > caps.center_freq_max)
        rx_cfg.center_freq = caps.center_freq_max;
      if (rx_cfg.gain < caps.gain_min)
        rx_cfg.gain = caps.gain_min;
      if (rx_cfg.gain > caps.gain_max)
        rx_cfg.gain = caps.gain_max;
    }

    if (have_record_gscn) {
      uint32_t gg = record_gscn;
      uint32_t do_snap = 0u;
      int snapped = 0;

      if (cli_env_u32("RECORD_SNAP_N41", &do_snap) == 0 && do_snap != 0u) {
        if (nr_gscn_snap_to_n41_scs30k_raster(record_gscn, &gg) == TOA_OK && gg != record_gscn)
          snapped = 1;
      }
      if (nr_gscn_fr1_lt_3ghz_to_ss_ref_hz(gg, &rx_cfg.center_freq) != TOA_OK) {
        log_msg(LOG_LEVEL_ERROR, "RECORD", "RECORD_GSCN invalid or out of FR1 raster");
        goto out;
      }
      if (caps_ok) {
        if (rx_cfg.center_freq < caps.center_freq_min)
          rx_cfg.center_freq = caps.center_freq_min;
        if (rx_cfg.center_freq > caps.center_freq_max)
          rx_cfg.center_freq = caps.center_freq_max;
      }
      log_msg(LOG_LEVEL_INFO,
              "RECORD",
              "RECORD_GSCN=%u%s -> center_freq=%.3f MHz (SS_REF)",
              record_gscn,
              snapped ? " snapped to n41 raster" : "",
              rx_cfg.center_freq * 1e-6);
    }
  }
  snprintf(rx_cfg.antenna, sizeof(rx_cfg.antenna), "LNAH");
  if (getenv("RECORD_ANTENNA"))
    snprintf(rx_cfg.antenna, sizeof(rx_cfg.antenna), "%s", getenv("RECORD_ANTENNA"));

  log_msg(LOG_LEVEL_INFO,
          "RECORD",
          "RX cfg (after clamp): fs=%.3f MHz fc=%.3f MHz bw=%.3f MHz gain=%.1f dB ant=%s",
          rx_cfg.sample_rate * 1e-6,
          rx_cfg.center_freq * 1e-6,
          rx_cfg.bandwidth * 1e-6,
          rx_cfg.gain,
          rx_cfg.antenna);

  ret = device.trx_configure_rx(&device, &rx_cfg);
  if (ret != TOA_OK) {
    log_msg(LOG_LEVEL_ERROR, "RECORD", "trx_configure_rx failed, ret=%d", ret);
    goto out;
  }

  memset(&rx_stream_cfg, 0, sizeof(rx_stream_cfg));
  rx_stream_cfg.channel_id = 0;
  rx_stream_cfg.buffer_size = DEMO_RX_BUFFER_SAMPLES;

  ret = device.trx_setup_rx_stream(&device, &rx_stream_cfg);
  if (ret != TOA_OK) {
    log_msg(LOG_LEVEL_ERROR, "RECORD", "trx_setup_rx_stream failed, ret=%d", ret);
    goto out;
  }

  ret = device.trx_start_rx_stream(&device);
  if (ret != TOA_OK) {
    log_msg(LOG_LEVEL_ERROR, "RECORD", "trx_start_rx_stream failed, ret=%d", ret);
    goto out;
  }

  rx_buffer = (cf32_t *)malloc(sizeof(cf32_t) * rx_stream_cfg.buffer_size);
  if (!rx_buffer) {
    log_msg(LOG_LEVEL_ERROR, "RECORD", "malloc rx_buffer failed");
    ret = TOA_ERR_NO_MEMORY;
    goto out;
  }

  buffs[0] = rx_buffer;

  if (capture_session_open(&session, session_base) != TOA_OK) {
    log_msg(LOG_LEVEL_ERROR, "RECORD", "capture_session_open failed: %s", session_base);
    goto out;
  }
  session_opened = 1;

  iq_meta_init_default(&meta);
  meta.sample_rate_hz = rx_cfg.sample_rate;
  meta.center_freq_hz = rx_cfg.center_freq;
  meta.bandwidth_hz = rx_cfg.bandwidth;
  meta.gain_db = rx_cfg.gain;
  meta.block_size = rx_stream_cfg.buffer_size;
  if (device.get_hw_time_ns)
    meta.capture_start_time_ns = device.get_hw_time_ns(&device);
  snprintf(meta.device_args, sizeof(meta.device_args), "%s", "driver=lime");
  capture_session_set_meta(&session, &meta);

  log_msg(LOG_LEVEL_INFO,
          "RECORD",
          "start recording session=%s (.cf32 + .blocklog.csv + .meta.json)",
          session_base);

  for (i = 0; i < num_blocks; i++) {
    memset(&rx_result, 0, sizeof(rx_result));

    ret = device.trx_read_func(&device,
                               buffs,
                               rx_stream_cfg.buffer_size,
                               &rx_result,
                               100000);

    if (ret != 0) {
      log_msg(LOG_LEVEL_WARN, "RECORD", "read failed block=%d ret=%d", i, ret);
      break;
    }

    if (rx_result.samples_read > 0) {
      if (capture_session_write_block(&session,
                                      rx_buffer,
                                      (uint32_t)rx_result.samples_read,
                                      rx_result.timestamp_ns,
                                      0) != TOA_OK) {
        log_msg(LOG_LEVEL_WARN, "RECORD", "write failed block=%d", i);
        break;
      }
    }

    if ((i % 100) == 0) {
      log_msg(LOG_LEVEL_INFO,
              "RECORD",
              "block=%d total_samples=%llu",
              i,
              (unsigned long long)session.iq.total_samples);
    }
  }

  log_msg(LOG_LEVEL_INFO,
          "RECORD",
          "record finished, total_samples=%llu",
          (unsigned long long)session.iq.total_samples);
  rc = 0;

out:
  if (session_opened)
    capture_session_close(&session);
  free(rx_buffer);
  if (device.rx_running)
    (void)device.trx_stop_rx_stream(&device);
  if (device.is_open)
    device.dev_close(&device);
  return rc;
}
