/**
 * Lime LMS / SoapyLMS7 backend ?ù OAI-style openair0_device_t.
 * Keep Lime logic isolated; PHY/NR_POSITIONING stays backend-agnostic.
 */
#include "../COMMON/common_lib.h"
#include <cstdlib>
#include <stdio.h>

extern "C" {

static int lime_trx_config(openair0_device_t *device, openair0_config_t *cfg)
{
  (void)device;
  (void)cfg;
  return 0;
}

static int lime_trx_start(openair0_device_t *device)
{
  (void)device;
  return 0;
}

static int lime_trx_stop(openair0_device_t *device)
{
  (void)device;
  return 0;
}

static int lime_trx_end(openair0_device_t *device, openair0_device_t **device2)
{
  (void)device;
  (void)device2;
  return 0;
}

static int lime_trx_read(openair0_device_t *device,
                         openair0_timestamp_t *ptimestamp,
                         void **buff,
                         uint32_t nsamps,
                         int antenna)
{
  (void)device;
  (void)ptimestamp;
  (void)buff;
  (void)nsamps;
  (void)antenna;
  static int warned = 0;
  if (!warned) {
    warned = 1;
    printf("Lime backend placeholder: lime_trx_read() not implemented (no IQ path)\n");
  }
  return -1;
}

static int lime_trx_write(openair0_device_t *device,
                          openair0_timestamp_t *ptimestamp,
                          void **buff,
                          uint32_t nsamps,
                          int antenna,
                          int flags)
{
  (void)device;
  (void)ptimestamp;
  (void)buff;
  (void)nsamps;
  (void)antenna;
  (void)flags;
  return 0;
}

static int lime_set_rx_freq(openair0_device_t *device, double rx_freq_hz)
{
  (void)device;
  (void)rx_freq_hz;
  return -1;
}

openair0_device_t *openair0_device_get_lime(openair0_config_t *cfg)
{
  openair0_device_t *dev =
      (openair0_device_t *)calloc(1, sizeof(openair0_device_t));
  if (!dev) {
    return NULL;
  }
  dev->trx_config_func = lime_trx_config;
  dev->trx_start_func = lime_trx_start;
  dev->trx_stop_func = lime_trx_stop;
  dev->trx_end_func = lime_trx_end;
  dev->trx_read_func = lime_trx_read;
  dev->trx_write_func = lime_trx_write;
  dev->trx_set_rx_freq_func = lime_set_rx_freq;
  dev->openair0_cfg = cfg;
  return dev;
}

} /* extern "C" */
