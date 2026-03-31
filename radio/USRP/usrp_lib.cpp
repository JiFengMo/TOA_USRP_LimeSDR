/**
 * USRP backend (UHD) 闁炽儻鎷�? placeholder wiring to openair0_device_t.
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

#define USRP_V0_PSS_LEN 127
#define USRP_V0_NFFT 256
#define USRP_V0_CP 20
#define USRP_V0_PSS_TD_LEN (USRP_V0_NFFT + USRP_V0_CP)

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

static void usrp_v0_build_pss_td(int nid2, float *td_i, float *td_q)
{
  float pss[USRP_V0_PSS_LEN];
  float Xr[USRP_V0_NFFT];
  float Xi[USRP_V0_NFFT];
  usrp_v0_build_pss_seq(nid2, pss, USRP_V0_PSS_LEN);
  memset(Xr, 0, sizeof(Xr));
  memset(Xi, 0, sizeof(Xi));

  const int k0 = -(USRP_V0_PSS_LEN / 2);
  for (int m = 0; m < USRP_V0_PSS_LEN; m++) {
    int k = k0 + m;
    int bin = (k >= 0) ? k : (USRP_V0_NFFT + k);
    if (bin >= 0 && bin < USRP_V0_NFFT) {
      Xr[bin] = pss[m];
    }
  }

  float xr[USRP_V0_NFFT];
  float xq[USRP_V0_NFFT];
  for (int n = 0; n < USRP_V0_NFFT; n++) {
    double sr = 0.0;
    double si = 0.0;
    for (int k = 0; k < USRP_V0_NFFT; k++) {
      double ph = 2.0 * M_PI * (double)(k * n) / (double)USRP_V0_NFFT;
      sr += Xr[k] * cos(ph) - Xi[k] * sin(ph);
      si += Xr[k] * sin(ph) + Xi[k] * cos(ph);
    }
    xr[n] = (float)(sr / (double)USRP_V0_NFFT);
    xq[n] = (float)(si / (double)USRP_V0_NFFT);
  }

  for (int n = 0; n < USRP_V0_CP; n++) {
    td_i[n] = xr[USRP_V0_NFFT - USRP_V0_CP + n];
    td_q[n] = xq[USRP_V0_NFFT - USRP_V0_CP + n];
  }
  for (int n = 0; n < USRP_V0_NFFT; n++) {
    td_i[USRP_V0_CP + n] = xr[n];
    td_q[USRP_V0_CP + n] = xq[n];
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
  const uint32_t burst_len = USRP_V0_PSS_TD_LEN;
  const int burst_on = 1;
  float pss_i[USRP_V0_PSS_TD_LEN];
  float pss_q[USRP_V0_PSS_TD_LEN];
  usrp_v0_build_pss_td((int)(st->read_count % 3U), pss_i, pss_q);

  for (uint32_t n = 0; n < nsamps; n++) {
    int16_t i = 0;
    int16_t q = 0;

    if (burst_on && n >= burst_pos && n < burst_pos + burst_len) {
      uint32_t k = n - burst_pos;
      i = (int16_t)(12000.0f * pss_i[k]);
      q = (int16_t)(12000.0f * pss_q[k]);
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
