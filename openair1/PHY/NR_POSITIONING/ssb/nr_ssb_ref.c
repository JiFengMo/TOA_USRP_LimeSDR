#include "openair1/PHY/NR_POSITIONING/nr_pos_types.h"
#include "openair1/PHY/NR_POSITIONING/nr_pos_api.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

#define NR_V0_PSS_LEN 127U
#define NR_V0_NFFT 256U
#define NR_V0_CP 20U
#define NR_V0_SSB_SYMS 4U

uint32_t nr_v0_pss_td_len(void)
{
  return (uint32_t)(NR_V0_NFFT + NR_V0_CP);
}

uint32_t nr_v0_ssb_sym_len(void)
{
  return nr_v0_pss_td_len();
}

uint32_t nr_v0_ssb_burst_len(void)
{
  return NR_V0_SSB_SYMS * nr_v0_ssb_sym_len();
}

static void nr_v0_build_pss_seq(int nid2, float *seq)
{
  uint8_t x[NR_V0_PSS_LEN];
  memset(x, 0, sizeof(x));

  x[0] = 1;
  x[1] = 0;
  x[2] = 0;
  x[3] = 0;
  x[4] = 0;
  x[5] = 0;
  x[6] = 0;

  for (uint32_t n = 0; n < NR_V0_PSS_LEN - 7U; n++) {
    x[n + 7U] = (uint8_t)((x[n + 4U] + x[n]) & 1U);
  }

  int shift = (43 * (nid2 % 3)) % (int)NR_V0_PSS_LEN;
  for (uint32_t n = 0; n < NR_V0_PSS_LEN; n++) {
    int idx = ((int)n + shift) % (int)NR_V0_PSS_LEN;
    seq[n] = x[(uint32_t)idx] ? -1.0f : 1.0f;
  }
}

void nr_v0_pss_build_fd(int nid2, float *seq, uint32_t len)
{
  if (!seq || len < NR_V0_PSS_LEN) {
    return;
  }
  nr_v0_build_pss_seq(nid2, seq);
}

void nr_v0_pss_build_td_f(int nid2, float *td_i, float *td_q, uint32_t len)
{
  const uint32_t need = nr_v0_pss_td_len();
  if (!td_i || !td_q || len < need) {
    return;
  }

  float pss[NR_V0_PSS_LEN];
  float Xr[NR_V0_NFFT];
  float Xi[NR_V0_NFFT];
  nr_v0_build_pss_seq(nid2, pss);
  memset(Xr, 0, sizeof(Xr));
  memset(Xi, 0, sizeof(Xi));

  const int k0 = -((int)NR_V0_PSS_LEN / 2);
  for (uint32_t m = 0; m < NR_V0_PSS_LEN; m++) {
    int k = k0 + (int)m;
    int bin = (k >= 0) ? k : ((int)NR_V0_NFFT + k);
    if (bin >= 0 && bin < (int)NR_V0_NFFT) {
      Xr[(uint32_t)bin] = pss[m];
    }
  }

  float xr[NR_V0_NFFT];
  float xq[NR_V0_NFFT];
  for (uint32_t n = 0; n < NR_V0_NFFT; n++) {
    double sr = 0.0;
    double si = 0.0;
    for (uint32_t k = 0; k < NR_V0_NFFT; k++) {
      double ph = 2.0 * M_PI * (double)(k * n) / (double)NR_V0_NFFT;
      sr += (double)Xr[k] * cos(ph) - (double)Xi[k] * sin(ph);
      si += (double)Xr[k] * sin(ph) + (double)Xi[k] * cos(ph);
    }
    xr[n] = (float)(sr / (double)NR_V0_NFFT);
    xq[n] = (float)(si / (double)NR_V0_NFFT);
  }

  for (uint32_t n = 0; n < NR_V0_CP; n++) {
    td_i[n] = xr[NR_V0_NFFT - NR_V0_CP + n];
    td_q[n] = xq[NR_V0_NFFT - NR_V0_CP + n];
  }
  for (uint32_t n = 0; n < NR_V0_NFFT; n++) {
    td_i[NR_V0_CP + n] = xr[n];
    td_q[NR_V0_CP + n] = xq[n];
  }
}

int nr_v0_pss_build_td_iq(int nid2, c16_t *out, uint32_t out_len, float amp)
{
  const uint32_t need = nr_v0_pss_td_len();
  if (!out || out_len < need) {
    return -1;
  }
  float ti[NR_V0_NFFT + NR_V0_CP];
  float tq[NR_V0_NFFT + NR_V0_CP];
  nr_v0_pss_build_td_f(nid2, ti, tq, need);
  if (!(amp > 0.0f)) {
    amp = 12000.0f;
  }
  for (uint32_t n = 0; n < need; n++) {
    float ir = amp * ti[n];
    float iq = amp * tq[n];
    if (ir > 32767.0f) ir = 32767.0f;
    if (ir < -32768.0f) ir = -32768.0f;
    if (iq > 32767.0f) iq = 32767.0f;
    if (iq < -32768.0f) iq = -32768.0f;
    out[n].r = (int16_t)lrintf(ir);
    out[n].i = (int16_t)lrintf(iq);
  }
  return (int)need;
}

int nr_v0_ssb_build_burst_iq(int nid2, c16_t *out, uint32_t out_len, float amp)
{
  const uint32_t sym_len = nr_v0_ssb_sym_len();
  const uint32_t burst_len = nr_v0_ssb_burst_len();
  if (!out || out_len < burst_len) {
    return -1;
  }
  memset(out, 0, sizeof(*out) * out_len);

  /* Keep V1 realistic-enough structure: 4 OFDM symbols, PSS on symbol #2. */
  if (nr_v0_pss_build_td_iq(nid2, &out[2U * sym_len], out_len - 2U * sym_len, amp) < 0) {
    return -1;
  }

  /* Add weak deterministic pilots on other symbols for non-empty SSB shell. */
  for (uint32_t s = 0; s < NR_V0_SSB_SYMS; s++) {
    if (s == 2U) {
      continue;
    }
    uint32_t base = s * sym_len;
    for (uint32_t n = 0; n < sym_len; n += 32U) {
      out[base + n].r = (int16_t)((((int)(n + s)) & 1) ? 800 : -800);
      out[base + n].i = (int16_t)((((int)(n + 2U * s)) & 1) ? 800 : -800);
    }
  }
  return (int)burst_len;
}

void nr_ssb_ref_cache_touch(const nr_ssb_ref_t *ref)
{
  (void)ref;
}
