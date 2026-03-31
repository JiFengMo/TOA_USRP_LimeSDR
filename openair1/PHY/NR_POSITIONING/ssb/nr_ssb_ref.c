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
  /* NR PSS m-sequence (38.211) 閳ワ拷 ported to match OAI initial conditions.
   * Output is BPSK (+1/-1) in float. */
  uint8_t x[NR_V0_PSS_LEN];
  static const int8_t x_initial[7] = {0, 1, 1, 0, 1, 1, 1};
  for (uint32_t i = 0; i < 7U; i++) {
    x[i] = (uint8_t)x_initial[i];
  }
  memset(&x[7U], 0, (NR_V0_PSS_LEN - 7U) * sizeof(uint8_t));

  for (uint32_t i = 0; i < (NR_V0_PSS_LEN - 7U); i++) {
    x[i + 7U] = (uint8_t)((x[i + 4U] + x[i]) & 1U);
  }

  for (uint32_t n = 0; n < NR_V0_PSS_LEN; n++) {
    const int m = ((int)n + 43 * nid2) % (int)NR_V0_PSS_LEN;
    seq[n] = x[(uint32_t)m] ? -1.0f : 1.0f;
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

  /* Map PSS (127 active subcarriers) to NFFT bins such that:
   * active subcarriers span k = -64 .. +62, which equals the SSB PSS resource
   * (subcarriers 56..182 within 240 RBs). */
  for (uint32_t m = 0; m < NR_V0_PSS_LEN; m++) {
    const int k = (int)m - 64; /* -64..+62 */
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

/* NR SSS generation (38.211 7.4.2.3), output BPSK (+1/-1) in float. */
static void nr_v0_build_sss_seq(int nid1, int nid2, float *seq)
{
  uint8_t x0[NR_V0_PSS_LEN];
  uint8_t x1[NR_V0_PSS_LEN];
  static const int8_t x_initial[7] = {1, 0, 0, 0, 0, 0, 0};
  for (uint32_t i = 0; i < 7U; i++) {
    x0[i] = (uint8_t)x_initial[i];
    x1[i] = (uint8_t)x_initial[i];
  }
  memset(&x0[7U], 0, (NR_V0_PSS_LEN - 7U) * sizeof(uint8_t));
  memset(&x1[7U], 0, (NR_V0_PSS_LEN - 7U) * sizeof(uint8_t));

  for (uint32_t i = 0; i < (NR_V0_PSS_LEN - 7U); i++) {
    x0[i + 7U] = (uint8_t)((x0[i + 4U] + x0[i]) & 1U);
    x1[i + 7U] = (uint8_t)((x1[i + 1U] + x1[i]) & 1U);
  }

  const int m0 = 15 * (nid1 / 112) + (5 * nid2);
  const int m1 = nid1 % 112;

  for (uint32_t n = 0; n < NR_V0_PSS_LEN; n++) {
    const int b0 = x0[(n + (uint32_t)m0) % NR_V0_PSS_LEN];
    const int b1 = x1[(n + (uint32_t)m1) % NR_V0_PSS_LEN];
    const float s0 = b0 ? -1.0f : 1.0f;
    const float s1 = b1 ? -1.0f : 1.0f;
    seq[n] = s0 * s1;
  }
}

static void nr_v0_sss_build_td_f_impl(int nid1, int nid2,
                                  float *td_i, float *td_q, uint32_t len)
{
  const uint32_t need = nr_v0_pss_td_len();
  if (!td_i || !td_q || len < need) {
    return;
  }

  float sss[NR_V0_PSS_LEN];
  float Xr[NR_V0_NFFT];
  float Xi[NR_V0_NFFT];
  nr_v0_build_sss_seq(nid1, nid2, sss);
  memset(Xr, 0, sizeof(Xr));
  memset(Xi, 0, sizeof(Xi));

  for (uint32_t m = 0; m < NR_V0_PSS_LEN; m++) {
    const int k = (int)m - 64; /* -64..+62 */
    int bin = (k >= 0) ? k : ((int)NR_V0_NFFT + k);
    if (bin >= 0 && bin < (int)NR_V0_NFFT) {
      Xr[(uint32_t)bin] = sss[m];
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

static int nr_v0_sss_build_td_iq(int nid1, int nid2,
                                 c16_t *out, uint32_t out_len, float amp)
{
  const uint32_t need = nr_v0_pss_td_len();
  if (!out || out_len < need) {
    return -1;
  }
  float ti[NR_V0_NFFT + NR_V0_CP];
  float tq[NR_V0_NFFT + NR_V0_CP];
  nr_v0_sss_build_td_f_impl(nid1, nid2, ti, tq, need);
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

/* Public wrapper: build SSS time-domain reference (float IQ) for correlation. */
void nr_v0_sss_build_td_f(int nid1, int nid2,
                           float *td_i, float *td_q, uint32_t len)
{
  nr_v0_sss_build_td_f_impl(nid1, nid2, td_i, td_q, len);
}

int nr_v0_ssb_build_burst_iq(int nid1, int nid2,
                              c16_t *out, uint32_t out_len, float amp)
{
  const uint32_t sym_len = nr_v0_ssb_sym_len();
  const uint32_t burst_len = nr_v0_ssb_burst_len();
  if (!out || out_len < burst_len) {
    return -1;
  }
  memset(out, 0, sizeof(*out) * out_len);

  /* Standard SS/PBCH mapping within SSB:
   * PSS on symbol 0, SSS on symbol 2. */
  if (nr_v0_pss_build_td_iq(nid2, &out[0U * sym_len], out_len - 0U * sym_len, amp) <
      0) {
    return -1;
  }
  if (nr_v0_sss_build_td_iq(nid1, nid2, &out[2U * sym_len],
                            out_len - 2U * sym_len, amp) < 0) {
    return -1;
  }
  return (int)burst_len;
}

void nr_ssb_ref_cache_touch(const nr_ssb_ref_t *ref)
{
  (void)ref;
}
