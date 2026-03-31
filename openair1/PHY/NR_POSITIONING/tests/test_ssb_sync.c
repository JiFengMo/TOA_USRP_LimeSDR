#include "openair1/PHY/NR_POSITIONING/nr_pos_api.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

#define T_PSS_LEN 127
#define T_NFFT 256
#define T_CP 20
#define T_PSS_TD (T_NFFT + T_CP)

static void build_pss_seq(int nid2, float *seq)
{
  uint8_t x[7];
  x[0] = 0;
  x[1] = 1;
  x[2] = 1;
  x[3] = 0;
  x[4] = 1;
  x[5] = 1;
  x[6] = 1;
  uint8_t mseq[T_PSS_LEN];
  for (int n = 0; n < T_PSS_LEN; n++) {
    uint8_t newbit = (uint8_t)((x[3] ^ x[0]) & 0x1);
    uint8_t out = x[6];
    for (int i = 6; i > 0; i--) {
      x[i] = x[i - 1];
    }
    x[0] = newbit;
    mseq[n] = out;
  }
  const int shift = (43 * nid2) % T_PSS_LEN;
  for (int n = 0; n < T_PSS_LEN; n++) {
    int idx = (n + shift) % T_PSS_LEN;
    seq[n] = (mseq[idx] == 0) ? 1.0f : -1.0f;
  }
}

static void build_pss_td(int nid2, float *td_i, float *td_q)
{
  float pss[T_PSS_LEN];
  float xr[T_NFFT];
  float xq[T_NFFT];
  float Xr[T_NFFT];
  float Xq[T_NFFT];
  build_pss_seq(nid2, pss);
  memset(Xr, 0, sizeof(Xr));
  memset(Xq, 0, sizeof(Xq));
  const int k0 = -(T_PSS_LEN / 2);
  for (int m = 0; m < T_PSS_LEN; m++) {
    int k = k0 + m;
    int bin = (k >= 0) ? k : (T_NFFT + k);
    Xr[bin] = pss[m];
  }
  for (int n = 0; n < T_NFFT; n++) {
    double sr = 0.0;
    double si = 0.0;
    for (int k = 0; k < T_NFFT; k++) {
      double ph = 2.0 * M_PI * (double)(k * n) / (double)T_NFFT;
      sr += Xr[k] * cos(ph) - Xq[k] * sin(ph);
      si += Xr[k] * sin(ph) + Xq[k] * cos(ph);
    }
    xr[n] = (float)(sr / (double)T_NFFT);
    xq[n] = (float)(si / (double)T_NFFT);
  }
  for (int n = 0; n < T_CP; n++) {
    td_i[n] = xr[T_NFFT - T_CP + n];
    td_q[n] = xq[T_NFFT - T_CP + n];
  }
  for (int n = 0; n < T_NFFT; n++) {
    td_i[T_CP + n] = xr[n];
    td_q[T_CP + n] = xq[n];
  }
}

int main(void)
{
  nr_pss_hit_t hits[4];
  nr_iq_block_t blk;
  nr_sync_state_t sync;
  memset(&blk, 0, sizeof(blk));
  blk.nsamps = 512;
  c16_t buf[512];
  float td_i[T_PSS_TD];
  float td_q[T_PSS_TD];
  memset(buf, 0, sizeof(buf));
  build_pss_td(1, td_i, td_q);
  for (int i = 100; i < 100 + T_PSS_TD; i++) {
    int k = i - 100;
    buf[i].r = (int16_t)(12000.0f * td_i[k]);
    buf[i].i = (int16_t)(12000.0f * td_q[k]);
  }
  blk.rx[0] = buf;
  if (nr_ssb_pss_search(&blk, hits, 4) != 0) {
    return 1;
  }
  if (nr_ssb_refine_sync(&blk, &hits[0], &sync) != 0) {
    return 1;
  }
  return 0;
}
