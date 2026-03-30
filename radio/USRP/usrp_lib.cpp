/**
 * USRP backend (UHD) 闁炽儻鎷� placeholder wiring to openair0_device_t.
 * Full implementation mirrors OAI usrp_lib.cpp.
 */
#include "../COMMON/common_lib.h"
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>

extern "C" {

typedef struct {
  openair0_config_t cfg;
  openair0_timestamp_t ts;
  int started;
  uint64_t read_count;
} usrp_sim_state_t;

static void usrp_v0_build_pss_seq(int nid2, float *seq, int len)
{
  uint8_t m[127] = {0};
  if (len > 127) {
    len = 127;
  }

  m[0] = 1;
  m[1] = 0;
  m[2] = 0;
  m[3] = 0;
  m[4] = 0;
  m[5] = 0;
  m[6] = 0;
  for (int n = 0; n < 120; n++) {
    m[n + 7] = (uint8_t)((m[n + 4] + m[n]) & 1U);
  }

  int shift = (43 * (nid2 % 3)) % 127;
  for (int n = 0; n < len; n++) {
    int idx = (n + shift) % 127;
    seq[n] = m[idx] ? -1.0f : 1.0f;
  }
}

static int usrp_trx_config(openair0_device_t *device, openair0_config_t *cfg)
{
  if (!device || !cfg) {
    return -1;
  }
  usrp_sim_state_t *st = (usrp_sim_state_t *)device->priv;
  if (!st) {
    return -1;
  }
  st->cfg = *cfg;
  st->ts = 0;
  st->read_count = 0;
  st->started = 0;
  return 0;
}

static int usrp_trx_start(openair0_device_t *device)
{
  if (!device || !device->priv) {
    return -1;
  }
  usrp_sim_state_t *st = (usrp_sim_state_t *)device->priv;
  st->started = 1;
  return 0;
}

static int usrp_trx_stop(openair0_device_t *device)
{
  (void)device;
  return 0;
}

static int usrp_trx_end(openair0_device_t *device, openair0_device_t **device2)
{
  (void)device;
  (void)device2;
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
  usrp_sim_state_t *st = (usrp_sim_state_t *)device->priv;
  if (!st->started) {
    return -1;
  }

  int16_t *iq = (int16_t *)buff[0];
  const uint32_t burst_pos = nsamps / 4U;
  const uint32_t burst_len = 127U;
  const int burst_on = 1;
  float pss[127];
  usrp_v0_build_pss_seq((int)(st->read_count % 3U), pss, 127);

  for (uint32_t n = 0; n < nsamps; n++) {
    int16_t i = 0;
    int16_t q = 0;

    if (burst_on && n >= burst_pos && n < burst_pos + burst_len) {
      uint32_t k = n - burst_pos;
      i = (int16_t)(12000.0f * pss[k]);
      q = 0;
    } else {
      i = (int16_t)(((int)(st->read_count + n) % 7) - 3) * 30;
      q = (int16_t)(((int)(st->read_count + 2U * n) % 7) - 3) * 30;
    }

    iq[2U * n] = i;
    iq[2U * n + 1U] = q;
  }

  *ptimestamp = st->ts;
  st->ts += nsamps;
  st->read_count++;
  return (int)nsamps;
}

static int usrp_trx_write(openair0_device_t *device,
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

openair0_device_t *openair0_device_get_usrp(openair0_config_t *cfg)
{
  openair0_device_t *dev =
      (openair0_device_t *)calloc(1, sizeof(openair0_device_t));
  if (!dev) {
    return NULL;
  }
  dev->trx_config_func = usrp_trx_config;
  dev->trx_start_func = usrp_trx_start;
  dev->trx_stop_func = usrp_trx_stop;
  dev->trx_end_func = usrp_trx_end;
  dev->trx_read_func = usrp_trx_read;
  dev->trx_write_func = usrp_trx_write;
  dev->openair0_cfg = cfg;
  dev->priv = calloc(1, sizeof(usrp_sim_state_t));
  if (!dev->priv) {
    free(dev);
    return NULL;
  }
  return dev;
}

} /* extern "C" */
