/**
 * USRP B2xx backend using real UHD RX/TX streams.
 */
#include "../COMMON/common_lib.h"

#include <cmath>
#include <complex>
#include <cstdio>
#include <cstdlib>
#include <string>
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
};

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
    st->usrp->set_rx_freq(uhd::tune_request_t(cfg->rx_freq_hz), 0);
    st->usrp->set_tx_freq(uhd::tune_request_t(cfg->tx_freq_hz), 0);
    st->usrp->set_rx_gain(cfg->rx_gain_db, 0);
    st->usrp->set_tx_gain(cfg->tx_gain_db, 0);

    uhd::stream_args_t rx_args("sc16", "sc16");
    rx_args.channels = {0};
    st->rx_stream = st->usrp->get_rx_stream(rx_args);

    uhd::stream_args_t tx_args("sc16", "sc16");
    tx_args.channels = {0};
    st->tx_stream = st->usrp->get_tx_stream(tx_args);

    std::printf("USRP: configured (rate=%.0f, rx=%.0f, tx=%.0f)\n",
                st->sample_rate, cfg->rx_freq_hz, cfg->tx_freq_hz);
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
    int guard = 0;
    while (total < nsamps && guard < 16) {
      guard++;
      std::vector<void *> buffs(1);
      buffs[0] = (void *)(dst + total);
      uhd::rx_metadata_t md;
      const size_t got = st->rx_stream->recv(buffs, nsamps - total, md, 0.05, false);

      if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_TIMEOUT) {
        continue;
      }
      if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_OVERFLOW) {
        continue;
      }
      if (md.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE) {
        std::printf("USRP RX metadata error: %s\n", md.strerror().c_str());
        return -1;
      }
      if (!ts_set) {
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
    std::printf("USRP: RX retuned to %.0f Hz\n", rx_freq_hz);
    return 0;
  } catch (const std::exception &e) {
    std::printf("USRP: set_rx_freq exception: %s\n", e.what());
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
  dev->openair0_cfg = cfg;
  dev->priv = (void *)(new usrp_state_t());
  if (!dev->priv) {
    free(dev);
    return NULL;
  }
  return dev;
}

} /* extern "C" */
