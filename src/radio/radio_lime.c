#include "radio/radio_lime.h"
#include "common/error.h"
#include "common/log.h"

#include <SoapySDR/Device.h>
#include <SoapySDR/Formats.h>
#include <SoapySDR/Errors.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* LimeSDR 底层私有数据 */
typedef struct {
  SoapySDRDevice *sdr;
  SoapySDRStream *rx_stream;
  SoapySDRStream *tx_stream;
} radio_lime_priv_t;

/* 打开设�?? */
static int radio_lime_open(radio_device_t *device, const char *args)
{
  radio_lime_priv_t *priv = NULL;
  const char *dev_args = (args && args[0] != '\0') ? args : "driver=lime";

  if (!device)
    return TOA_ERR_INVALID_ARG;

  if (device->is_open)
    return 0;

  priv = (radio_lime_priv_t *)calloc(1, sizeof(*priv));
  if (!priv)
    return TOA_ERR_NO_MEMORY;

  /* 通过字�?�串参数创建设�?? */
  priv->sdr = SoapySDRDevice_makeStrArgs(dev_args);
  if (!priv->sdr) {
    log_msg(LOG_LEVEL_ERROR, "RADIO", "SoapySDR make failed: args=%s", dev_args);
    free(priv);
    return TOA_ERR_IO;
  }

  device->priv = priv;
  device->is_open = 1;
  return 0;
}

/* 关闭设�?? */
static void radio_lime_close(radio_device_t *device)
{
  radio_lime_priv_t *priv = NULL;

  if (!device || !device->priv)
    return;

  priv = (radio_lime_priv_t *)device->priv;

  /* �? RX 流仍存在，先关流 */
  if (priv->rx_stream) {
    SoapySDRDevice_closeStream(priv->sdr, priv->rx_stream);
    priv->rx_stream = NULL;
  }

  /* �? TX 流仍存在，先关流 */
  if (priv->tx_stream) {
    SoapySDRDevice_closeStream(priv->sdr, priv->tx_stream);
    priv->tx_stream = NULL;
  }

  /* 销毁�?��?? */
  if (priv->sdr) {
    SoapySDRDevice_unmake(priv->sdr);
    priv->sdr = NULL;
  }

  free(priv);
  device->priv = NULL;
  device->is_open = 0;
  device->rx_stream_open = 0;
  device->tx_stream_open = 0;
  device->rx_running = 0;
  device->tx_running = 0;
}

/* 查�?��?��?�能力（先提供单通道固定基线，后�?�?改成动态�?�取�? */
static int radio_lime_get_caps(radio_device_t *device, radio_device_caps_t *caps)
{
  if (!device || !device->priv || !caps)
    return TOA_ERR_INVALID_ARG;

  memset(caps, 0, sizeof(*caps));
  /* LimeSDR-Mini 量级（LimeSDR USB �?更高；�?��?�与 demo 默�?�一致） */
  caps->sample_rate_min = 1.0e6;
  caps->sample_rate_max = 30.72e6;
  caps->center_freq_min = 1.0e7;
  caps->center_freq_max = 3.5e9;
  caps->bandwidth_min = 1.5e6;
  caps->bandwidth_max = 30.72e6;
  caps->gain_min = 0.0;
  caps->gain_max = 70.0;
  snprintf(caps->rx_antennas, sizeof(caps->rx_antennas), "LNAH,LNAL,LNAW");
  snprintf(caps->tx_antennas, sizeof(caps->tx_antennas), "BAND1,BAND2");
  return TOA_OK;
}

/* 设置 RX 增益 */
static int radio_lime_set_rx_gain(radio_device_t *device, uint32_t channel, double gain)
{
  radio_lime_priv_t *priv = NULL;

  if (!device || !device->priv)
    return -1;

  priv = (radio_lime_priv_t *)device->priv;

  if (SoapySDRDevice_setGain(priv->sdr, SOAPY_SDR_RX, channel, gain) != 0)
    return -2;

  return 0;
}

/* 设置 TX 增益 */
static int radio_lime_set_tx_gain(radio_device_t *device, uint32_t channel, double gain)
{
  radio_lime_priv_t *priv = NULL;

  if (!device || !device->priv)
    return -1;

  priv = (radio_lime_priv_t *)device->priv;

  if (SoapySDRDevice_setGain(priv->sdr, SOAPY_SDR_TX, channel, gain) != 0)
    return -2;

  return 0;
}

/* 配置 RX 参数 */
static int radio_lime_configure_rx(radio_device_t *device, const radio_channel_config_t *cfg)
{
  radio_lime_priv_t *priv = NULL;

  if (!device || !device->priv || !cfg)
    return -1;

  priv = (radio_lime_priv_t *)device->priv;

  if (SoapySDRDevice_setSampleRate(priv->sdr, SOAPY_SDR_RX, cfg->channel_id, cfg->sample_rate) != 0)
    return -2;

  if (SoapySDRDevice_setFrequency(priv->sdr, SOAPY_SDR_RX, cfg->channel_id, cfg->center_freq, NULL) != 0)
    return -3;

  if (SoapySDRDevice_setBandwidth(priv->sdr, SOAPY_SDR_RX, cfg->channel_id, cfg->bandwidth) != 0)
    return -4;

  if (SoapySDRDevice_setGain(priv->sdr, SOAPY_SDR_RX, cfg->channel_id, cfg->gain) != 0)
    return -5;

  if (cfg->antenna[0] != '\0') {
    if (SoapySDRDevice_setAntenna(priv->sdr, SOAPY_SDR_RX, cfg->channel_id, cfg->antenna) != 0)
      return -6;
  }

  device->rx_cfg = *cfg;
  return 0;
}

/* 配置 TX 参数 */
static int radio_lime_configure_tx(radio_device_t *device, const radio_channel_config_t *cfg)
{
  radio_lime_priv_t *priv = NULL;

  if (!device || !device->priv || !cfg)
    return -1;

  priv = (radio_lime_priv_t *)device->priv;

  if (SoapySDRDevice_setSampleRate(priv->sdr, SOAPY_SDR_TX, cfg->channel_id, cfg->sample_rate) != 0)
    return -2;

  if (SoapySDRDevice_setFrequency(priv->sdr, SOAPY_SDR_TX, cfg->channel_id, cfg->center_freq, NULL) != 0)
    return -3;

  if (SoapySDRDevice_setBandwidth(priv->sdr, SOAPY_SDR_TX, cfg->channel_id, cfg->bandwidth) != 0)
    return -4;

  if (SoapySDRDevice_setGain(priv->sdr, SOAPY_SDR_TX, cfg->channel_id, cfg->gain) != 0)
    return -5;

  if (cfg->antenna[0] != '\0') {
    if (SoapySDRDevice_setAntenna(priv->sdr, SOAPY_SDR_TX, cfg->channel_id, cfg->antenna) != 0)
      return -6;
  }

  device->tx_cfg = *cfg;
  return 0;
}

/* 创建 RX �? */
static int radio_lime_setup_rx_stream(radio_device_t *device, const radio_stream_config_t *cfg)
{
  radio_lime_priv_t *priv = NULL;
  size_t channels[1];

  if (!device || !device->priv || !cfg)
    return TOA_ERR_INVALID_ARG;

  priv = (radio_lime_priv_t *)device->priv;
  channels[0] = cfg->channel_id;

  /* 已有流则先关�? */
  if (priv->rx_stream) {
    SoapySDRDevice_closeStream(priv->sdr, priv->rx_stream);
    priv->rx_stream = NULL;
  }

  priv->rx_stream = SoapySDRDevice_setupStream(
    priv->sdr,
    SOAPY_SDR_RX,
    SOAPY_SDR_CF32,
    channels,
    1,
    NULL);

  if (!priv->rx_stream)
    return TOA_ERR_IO;

  device->rx_stream_cfg = *cfg;
  device->rx_stream_open = 1;
  return 0;
}

/* 创建 TX �? */
static int radio_lime_setup_tx_stream(radio_device_t *device, const radio_stream_config_t *cfg)
{
  radio_lime_priv_t *priv = NULL;
  size_t channels[1];

  if (!device || !device->priv || !cfg)
    return TOA_ERR_INVALID_ARG;

  priv = (radio_lime_priv_t *)device->priv;
  channels[0] = cfg->channel_id;

  /* 已有流则先关�? */
  if (priv->tx_stream) {
    SoapySDRDevice_closeStream(priv->sdr, priv->tx_stream);
    priv->tx_stream = NULL;
  }

  priv->tx_stream = SoapySDRDevice_setupStream(
    priv->sdr,
    SOAPY_SDR_TX,
    SOAPY_SDR_CF32,
    channels,
    1,
    NULL);

  if (!priv->tx_stream)
    return TOA_ERR_IO;

  device->tx_stream_cfg = *cfg;
  device->tx_stream_open = 1;
  return 0;
}

/* �?�? RX �? */
static int radio_lime_start_rx_stream(radio_device_t *device)
{
  radio_lime_priv_t *priv = NULL;

  if (!device || !device->priv)
    return -1;

  priv = (radio_lime_priv_t *)device->priv;

  if (!priv->rx_stream)
    return -2;

  if (SoapySDRDevice_activateStream(priv->sdr, priv->rx_stream, 0, 0, 0) != 0)
    return -3;

  device->rx_running = 1;
  return 0;
}

/* 停�?? RX �? */
static int radio_lime_stop_rx_stream(radio_device_t *device)
{
  radio_lime_priv_t *priv = NULL;

  if (!device || !device->priv)
    return -1;

  priv = (radio_lime_priv_t *)device->priv;

  if (!priv->rx_stream)
    return -2;

  if (SoapySDRDevice_deactivateStream(priv->sdr, priv->rx_stream, 0, 0) != 0)
    return -3;

  device->rx_running = 0;
  return 0;
}

/* �?�? TX �? */
static int radio_lime_start_tx_stream(radio_device_t *device)
{
  radio_lime_priv_t *priv = NULL;

  if (!device || !device->priv)
    return -1;

  priv = (radio_lime_priv_t *)device->priv;

  if (!priv->tx_stream)
    return -2;

  if (SoapySDRDevice_activateStream(priv->sdr, priv->tx_stream, 0, 0, 0) != 0)
    return -3;

  device->tx_running = 1;
  return 0;
}

/* 停�?? TX �? */
static int radio_lime_stop_tx_stream(radio_device_t *device)
{
  radio_lime_priv_t *priv = NULL;

  if (!device || !device->priv)
    return -1;

  priv = (radio_lime_priv_t *)device->priv;

  if (!priv->tx_stream)
    return -2;

  if (SoapySDRDevice_deactivateStream(priv->sdr, priv->tx_stream, 0, 0) != 0)
    return -3;

  device->tx_running = 0;
  return 0;
}

/* �? RX 流�?�取 IQ */
static int radio_lime_read(radio_device_t *device,
                           void **buffs,
                           uint32_t nsamps,
                           radio_rx_result_t *result,
                           int timeout_us)
{
  radio_lime_priv_t *priv = NULL;
  int flags = 0;
  long long time_ns = 0;
  int ret;

  if (!device || !device->priv || !buffs || !result)
    return -1;

  priv = (radio_lime_priv_t *)device->priv;

  if (!priv->rx_stream)
    return -2;

  ret = SoapySDRDevice_readStream(priv->sdr,
                                  priv->rx_stream,
                                  buffs,
                                  nsamps,
                                  &flags,
                                  &time_ns,
                                  timeout_us);

  result->samples_read = ret;
  result->flags = flags;
  result->timestamp_ns = time_ns;

  /* ret < 0 表示底层读失�? */
  if (ret < 0)
    return ret;

  return 0;
}

/* �? TX 流写 IQ */
static int radio_lime_write(radio_device_t *device,
                            const void **buffs,
                            uint32_t nsamps,
                            int64_t timestamp_ns,
                            int flags,
                            int timeout_us)
{
  radio_lime_priv_t *priv = NULL;
  int ret;

  if (!device || !device->priv || !buffs)
    return -1;

  priv = (radio_lime_priv_t *)device->priv;

  if (!priv->tx_stream)
    return -2;

  ret = SoapySDRDevice_writeStream(priv->sdr,
                                   priv->tx_stream,
                                   buffs,
                                   nsamps,
                                   &flags,
                                   timestamp_ns,
                                   timeout_us);

  if (ret < 0)
    return ret;

  return 0;
}

/* 获取�?件时�? */
static int64_t radio_lime_get_hw_time_ns(radio_device_t *device)
{
  radio_lime_priv_t *priv = NULL;

  if (!device || !device->priv)
    return -1;

  priv = (radio_lime_priv_t *)device->priv;
  return SoapySDRDevice_getHardwareTime(priv->sdr, NULL);
}

/* 设置�?件时�? */
static int radio_lime_set_hw_time_ns(radio_device_t *device, int64_t time_ns)
{
  radio_lime_priv_t *priv = NULL;

  if (!device || !device->priv)
    return -1;

  priv = (radio_lime_priv_t *)device->priv;
  return SoapySDRDevice_setHardwareTime(priv->sdr, time_ns, NULL);
}

/* 初�?�化 LimeSDR 设�?��?�象，�?�果之后改成USRP则重点改这里 */
int radio_lime_device_init(radio_device_t *device)
{
  if (!device)
    return TOA_ERR_INVALID_ARG;

  memset(device, 0, sizeof(*device));

  device->type = RADIO_DEVICE_LIME;
  snprintf(device->name, sizeof(device->name), "LimeSDR");

  device->dev_open = radio_lime_open;
  device->dev_close = radio_lime_close;
  device->dev_get_caps = radio_lime_get_caps;

  device->trx_set_rx_gain = radio_lime_set_rx_gain;
  device->trx_set_tx_gain = radio_lime_set_tx_gain;

  device->trx_configure_rx = radio_lime_configure_rx;
  device->trx_configure_tx = radio_lime_configure_tx;

  device->trx_setup_rx_stream = radio_lime_setup_rx_stream;
  device->trx_setup_tx_stream = radio_lime_setup_tx_stream;

  device->trx_start_rx_stream = radio_lime_start_rx_stream;
  device->trx_stop_rx_stream = radio_lime_stop_rx_stream;

  device->trx_start_tx_stream = radio_lime_start_tx_stream;
  device->trx_stop_tx_stream = radio_lime_stop_tx_stream;

  device->trx_read_func = radio_lime_read;
  device->trx_write_func = radio_lime_write;

  device->get_hw_time_ns = radio_lime_get_hw_time_ns;
  device->set_hw_time_ns = radio_lime_set_hw_time_ns;

  return 0;
}