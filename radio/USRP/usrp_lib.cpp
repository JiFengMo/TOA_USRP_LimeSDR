/**
 * USRP B2xx backend using real UHD RX/TX streams.
 */
#include "../COMMON/common_lib.h"

#include <cmath>
#include <chrono>
#include <complex>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>
#include <vector>

#include <uhd/stream.hpp>
#include <uhd/types/metadata.hpp>
#include <uhd/types/stream_cmd.hpp>
#include <uhd/types/tune_request.hpp>
#include <uhd/usrp/multi_usrp.hpp>

struct usrp_state_t {
  openair0_config_t cfg;
  uhd::usrp::multi_usrp::sptr usrp;
  uhd::rx_streamer::sptr rx_stream;
  uhd::tx_streamer::sptr tx_stream;
  double sample_rate;
  bool started;
  bool rx_started;
  uint64_t rx_overflow_cnt;
  uint64_t rx_timeout_cnt;
};

static bool usrp_sensor_bool(uhd::usrp::multi_usrp::sptr usrp,
                             const std::string &name,
                             size_t chan,
                             bool default_value)
{
  if (!usrp) {
    return default_value;
  }

  const auto board_sensors = usrp->get_mboard_sensor_names(0);
  for (const auto &n : board_sensors) {
    if (n == name) {
      return usrp->get_mboard_sensor(name, 0).to_bool();
    }
  }

  const auto rx_sensors = usrp->get_rx_sensor_names(chan);
  for (const auto &n : rx_sensors) {
    if (n == name) {
      return usrp->get_rx_sensor(name, chan).to_bool();
    }
  }

  return default_value;
}

extern "C" {

static int usrp_trx_config(openair0_device_t *device, openair0_config_t *cfg)
{
  if (!device || !cfg || !device->priv) {
    return -1;
  }
  usrp_state_t *st = (usrp_state_t *)device->priv;
  st->cfg = *cfg;
  st->sample_rate = (cfg->sample_rate > 0.0) ? cfg->sample_rate : 30.72e6;
  st->started = false;
  st->rx_started = false;
  st->rx_overflow_cnt = 0;
  st->rx_timeout_cnt = 0;

  try {
    const std::string args = (cfg->sdr_addrs && cfg->sdr_addrs[0] != '\0')
                                 ? std::string(cfg->sdr_addrs)
                                 : std::string("");
    st->usrp = uhd::usrp::multi_usrp::make(args);
    if (!st->usrp) {
      std::printf("USRP: failed to create multi_usrp\n");
      return -1;
    }
    if (cfg->clock_source && cfg->clock_source[0] != '\0') {
      st->usrp->set_clock_source(cfg->clock_source);
    }
    if (cfg->time_source && cfg->time_source[0] != '\0') {
      st->usrp->set_time_source(cfg->time_source);
    }

    st->usrp->set_rx_rate(st->sample_rate, 0);
    st->usrp->set_tx_rate(st->sample_rate, 0);
    if (cfg->rx_antenna && cfg->rx_antenna[0] != '\0') {
      st->usrp->set_rx_antenna(cfg->rx_antenna, 0);
    }
    if (cfg->tx_antenna && cfg->tx_antenna[0] != '\0') {
      st->usrp->set_tx_antenna(cfg->tx_antenna, 0);
    }
    st->usrp->set_rx_freq(uhd::tune_request_t(cfg->rx_freq_hz), 0);
    st->usrp->set_tx_freq(uhd::tune_request_t(cfg->tx_freq_hz), 0);
    st->usrp->set_rx_gain(cfg->rx_gain_db, 0);
    st->usrp->set_tx_gain(cfg->tx_gain_db, 0);
    /* Let LO settle before streaming and report lock status if available. */
    std::this_thread::sleep_for(std::chrono::milliseconds(30));

    uhd::stream_args_t rx_args("sc16", "sc16");
    rx_args.channels = {0};
    st->rx_stream = st->usrp->get_rx_stream(rx_args);

    uhd::stream_args_t tx_args("sc16", "sc16");
    tx_args.channels = {0};
    st->tx_stream = st->usrp->get_tx_stream(tx_args);

    const double rate_act = st->usrp->get_rx_rate(0);
    st->sample_rate = rate_act;
    st->cfg.sample_rate = rate_act;
    cfg->sample_rate = rate_act;
    const double rx_freq_act = st->usrp->get_rx_freq(0);
    const double tx_freq_act = st->usrp->get_tx_freq(0);
    const double rx_gain_act = st->usrp->get_rx_gain(0);
    const double tx_gain_act = st->usrp->get_tx_gain(0);
    const double rx_bw_act = st->usrp->get_rx_bandwidth(0);
    const std::string rx_ant_act = st->usrp->get_rx_antenna(0);
    const std::string tx_ant_act = st->usrp->get_tx_antenna(0);
    bool lo_locked = true;
    const auto sns = st->usrp->get_rx_sensor_names(0);
    for (const auto &n : sns) {
      if (n == "lo_locked") {
        lo_locked = st->usrp->get_rx_sensor("lo_locked", 0).to_bool();
        break;
      }
    }
    std::printf(
        "USRP: configured req(rate=%.0f rx=%.0f tx=%.0f gain=%.1f) "
        "act(rate=%.0f rx=%.0f tx=%.0f rx_gain=%.1f tx_gain=%.1f rx_bw=%.0f rx_ant=%s tx_ant=%s) lo_locked=%d\n",
        st->sample_rate, cfg->rx_freq_hz, cfg->tx_freq_hz, cfg->rx_gain_db,
        rate_act, rx_freq_act, tx_freq_act, rx_gain_act, tx_gain_act, rx_bw_act,
        rx_ant_act.c_str(), tx_ant_act.c_str(),
        lo_locked ? 1 : 0);
  } catch (const std::exception &e) {
    std::printf("USRP: trx_config exception: %s\n", e.what());
    return -1;
  }

  return 0;
}

static int usrp_trx_start(openair0_device_t *device)
{
  if (!device || !device->priv) {
    return -1;
  }
  usrp_state_t *st = (usrp_state_t *)device->priv;
  if (!st->usrp || !st->rx_stream || !st->tx_stream) {
    return -1;
  }
  try {
    const uhd::time_spec_t t0 = st->usrp->get_time_now() + uhd::time_spec_t(0.05);
    uhd::stream_cmd_t cmd(uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);
    cmd.stream_now = false;
    cmd.time_spec = t0;
    st->rx_stream->issue_stream_cmd(cmd);
    st->rx_started = true;
    st->started = true;
    std::printf("USRP: RX streaming started\n");
  } catch (const std::exception &e) {
    std::printf("USRP: trx_start exception: %s\n", e.what());
    return -1;
  }
  return 0;
}

static int usrp_trx_stop(openair0_device_t *device)
{
  if (!device || !device->priv) {
    return -1;
  }
  usrp_state_t *st = (usrp_state_t *)device->priv;
  if (!st->rx_started || !st->rx_stream) {
    return 0;
  }
  try {
    uhd::stream_cmd_t cmd(uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS);
    st->rx_stream->issue_stream_cmd(cmd);
  } catch (...) {
  }
  st->rx_started = false;
  st->started = false;
  return 0;
}

static int usrp_trx_end(openair0_device_t *device, openair0_device_t **device2)
{
  (void)device2;
  if (!device) {
    return -1;
  }
  if (device->priv) {
    usrp_state_t *st = (usrp_state_t *)device->priv;
    delete st;
    device->priv = NULL;
  }
  return 0;
}

static int usrp_trx_read(openair0_device_t *device,
                         openair0_timestamp_t *ptimestamp,
                         void **buff,
                         uint32_t nsamps,
                         int antenna)
{
  (void)antenna;
  if (!device || !device->priv || !ptimestamp || !buff || !buff[0]) {
    return -1;
  }
  usrp_state_t *st = (usrp_state_t *)device->priv;
  if (!st->started || !st->rx_stream) {
    return -1;
  }

  try {
    auto *dst = reinterpret_cast<std::complex<int16_t> *>(buff[0]);
    uint32_t total = 0;
    bool ts_set = false;
    uint64_t timeout_ms = (uint64_t)(1000.0 * ((double)nsamps / st->sample_rate + 0.5));
    if (timeout_ms < 200U) {
      timeout_ms = 200U;
    }
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds((long long)timeout_ms);
    while (total < nsamps) {
      const auto now = std::chrono::steady_clock::now();
      if (now >= deadline) {
        std::printf("USRP: RX block timeout partial=%u/%u timeout_ms=%llu\n",
                    total,
                    nsamps,
                    (unsigned long long)timeout_ms);
        return -1;
      }
      const double remain = std::chrono::duration<double>(deadline - now).count();
      const double recv_timeout = (remain < 0.05) ? remain : 0.05;
      std::vector<void *> buffs(1);
      buffs[0] = (void *)(dst + total);
      uhd::rx_metadata_t md;
      const size_t got = st->rx_stream->recv(buffs, nsamps - total, md, recv_timeout, false);

      if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_TIMEOUT) {
        st->rx_timeout_cnt++;
        continue;
      }
      if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_OVERFLOW) {
        st->rx_overflow_cnt++;
        std::printf("USRP: RX overflow count=%llu timeout=%llu partial=%u/%u; dropping block\n",
                    (unsigned long long)st->rx_overflow_cnt,
                    (unsigned long long)st->rx_timeout_cnt,
                    total,
                    nsamps);
        return -2;
      }
      if (md.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE) {
        std::printf("USRP RX metadata error: %s\n", md.strerror().c_str());
        return -1;
      }
      if (!ts_set) {
        if (!md.has_time_spec) {
          std::printf("USRP RX metadata missing first-sample timestamp\n");
          return -1;
        }
        const double tsec = md.time_spec.get_real_secs();
        *ptimestamp = (openair0_timestamp_t)llround(tsec * st->sample_rate);
        ts_set = true;
      }
      total += (uint32_t)got;
    }
    if (total != nsamps) {
      return -1;
    }
    if (!ts_set) {
      *ptimestamp = 0;
    }
    return (int)total;
  } catch (const std::exception &e) {
    std::printf("USRP: trx_read exception: %s\n", e.what());
    return -1;
  }
}

static int usrp_trx_write(openair0_device_t *device,
                          openair0_timestamp_t *ptimestamp,
                          void **buff,
                          uint32_t nsamps,
                          int antenna,
                          int flags)
{
  (void)antenna;
  (void)flags;
  if (!device || !device->priv || !buff || !buff[0]) {
    return -1;
  }
  usrp_state_t *st = (usrp_state_t *)device->priv;
  if (!st->started || !st->tx_stream) {
    return -1;
  }

  try {
    uhd::tx_metadata_t md;
    md.start_of_burst = false;
    md.end_of_burst = false;
    md.has_time_spec = false;
    if (ptimestamp && st->sample_rate > 0.0) {
      md.has_time_spec = true;
      md.time_spec = uhd::time_spec_t((double)(*ptimestamp) / st->sample_rate);
    }
    std::vector<const void *> buffs(1);
    buffs[0] = buff[0];
    const size_t sent = st->tx_stream->send(buffs, nsamps, md, 0.2);
    return (int)sent;
  } catch (const std::exception &e) {
    std::printf("USRP: trx_write exception: %s\n", e.what());
    return -1;
  }
}

static int usrp_set_rx_freq(openair0_device_t *device, double rx_freq_hz)
{
  if (!device || !device->priv || !(rx_freq_hz > 0.0)) {
    return -1;
  }
  usrp_state_t *st = (usrp_state_t *)device->priv;
  if (!st->usrp) {
    return -1;
  }
  try {
    st->usrp->set_rx_freq(uhd::tune_request_t(rx_freq_hz), 0);
    st->cfg.rx_freq_hz = rx_freq_hz;
    std::this_thread::sleep_for(std::chrono::milliseconds(15));
    bool lo_locked = true;
    const auto sns = st->usrp->get_rx_sensor_names(0);
    for (const auto &n : sns) {
      if (n == "lo_locked") {
        lo_locked = st->usrp->get_rx_sensor("lo_locked", 0).to_bool();
        break;
      }
    }
    std::printf("USRP: RX retuned to %.0f Hz (act=%.0f lo_locked=%d)\n",
                rx_freq_hz, st->usrp->get_rx_freq(0), lo_locked ? 1 : 0);
    return 0;
  } catch (const std::exception &e) {
    std::printf("USRP: set_rx_freq exception: %s\n", e.what());
    return -1;
  }
}

static int usrp_set_rx_gain(openair0_device_t *device, double rx_gain_db)
{
  if (!device || !device->priv || !(rx_gain_db >= 0.0)) {
    return -1;
  }
  usrp_state_t *st = (usrp_state_t *)device->priv;
  if (!st->usrp) {
    return -1;
  }
  try {
    st->usrp->set_rx_gain(rx_gain_db, 0);
    st->cfg.rx_gain_db = rx_gain_db;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    std::printf("USRP: RX gain set to %.1f dB (act=%.1f)\n",
                rx_gain_db, st->usrp->get_rx_gain(0));
    return 0;
  } catch (const std::exception &e) {
    std::printf("USRP: set_rx_gain exception: %s\n", e.what());
    return -1;
  }
}

static int usrp_get_device_time(openair0_device_t *device, openair0_timestamp_t *ts)
{
  if (!device || !device->priv || !ts) {
    return -1;
  }
  usrp_state_t *st = (usrp_state_t *)device->priv;
  if (!st->usrp || !(st->sample_rate > 0.0)) {
    return -1;
  }
  try {
    const double tsec = st->usrp->get_time_now().get_real_secs();
    *ts = (openair0_timestamp_t)llround(tsec * st->sample_rate);
    return 0;
  } catch (const std::exception &e) {
    std::printf("USRP: get_device_time exception: %s\n", e.what());
    return -1;
  }
}

static int usrp_set_time_next_pps(openair0_device_t *device, uint64_t epoch_ns)
{
  if (!device || !device->priv) {
    return -1;
  }
  usrp_state_t *st = (usrp_state_t *)device->priv;
  if (!st->usrp) {
    return -1;
  }
  try {
    const double epoch_sec = (double)epoch_ns / 1.0e9;
    st->usrp->set_time_next_pps(uhd::time_spec_t(epoch_sec));
    std::this_thread::sleep_for(std::chrono::milliseconds(1100));
    std::printf("USRP: time set at next PPS to %.9f s\n", epoch_sec);
    return 0;
  } catch (const std::exception &e) {
    std::printf("USRP: set_time_next_pps exception: %s\n", e.what());
    return -1;
  }
}

static int usrp_get_clock_status(openair0_device_t *device,
                                 uint8_t *ref_locked,
                                 uint8_t *pps_locked,
                                 uint8_t *gps_locked)
{
  if (!device || !device->priv || !ref_locked || !pps_locked || !gps_locked) {
    return -1;
  }
  usrp_state_t *st = (usrp_state_t *)device->priv;
  if (!st->usrp) {
    return -1;
  }
  try {
    const std::string clock_src = st->cfg.clock_source ? std::string(st->cfg.clock_source) : std::string("");
    const std::string time_src = st->cfg.time_source ? std::string(st->cfg.time_source) : std::string("");
    bool ref = true;
    if (clock_src == "external" || clock_src == "gpsdo" || clock_src == "gps") {
      ref = usrp_sensor_bool(st->usrp, "ref_locked", 0, false);
    }
    const bool gps = usrp_sensor_bool(st->usrp, "gps_locked", 0, false);
    bool pps = true;
    if (time_src == "gpsdo" || time_src == "gps") {
      pps = gps;
    } else if (time_src == "external") {
      pps = ref;
    }
    *ref_locked = ref ? 1U : 0U;
    *pps_locked = pps ? 1U : 0U;
    *gps_locked = gps ? 1U : 0U;
    return 0;
  } catch (const std::exception &e) {
    std::printf("USRP: get_clock_status exception: %s\n", e.what());
    return -1;
  }
}

openair0_device_t *openair0_device_get_usrp(openair0_config_t *cfg)
{
  openair0_device_t *dev = (openair0_device_t *)calloc(1, sizeof(openair0_device_t));
  if (!dev) {
    return NULL;
  }
  dev->trx_config_func = usrp_trx_config;
  dev->trx_start_func = usrp_trx_start;
  dev->trx_stop_func = usrp_trx_stop;
  dev->trx_end_func = usrp_trx_end;
  dev->trx_read_func = usrp_trx_read;
  dev->trx_write_func = usrp_trx_write;
  dev->trx_set_rx_freq_func = usrp_set_rx_freq;
  dev->trx_set_rx_gain_func = usrp_set_rx_gain;
  dev->trx_get_time_func = usrp_get_device_time;
  dev->trx_set_time_next_pps_func = usrp_set_time_next_pps;
  dev->trx_get_clock_status_func = usrp_get_clock_status;
  dev->openair0_cfg = cfg;
  dev->priv = (void *)(new usrp_state_t());
  if (!dev->priv) {
    free(dev);
    return NULL;
  }
  return dev;
}

} /* extern "C" */
