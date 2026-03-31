#include "openair1/PHY/NR_POSITIONING/nr_pos_types.h"
#include "openair1/PHY/NR_POSITIONING/nr_pos_api.h"

#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define NR_V0_PSS_LEN 127U
#define NR_V0_SSB_SC 240U
#define NR_V0_SSB_SYMS 4U
#define NR_V0_FS_HZ_FALLBACK 30720000.0

static uint32_t nr_v0_ssb_nfft_from_fs(double fs_hz)
{
  const double fs = (fs_hz > 1.0) ? fs_hz : NR_V0_FS_HZ_FALLBACK;
  uint32_t nfft = (uint32_t)llround(fs / 30000.0);
  if (nfft < 256U) {
    nfft = 256U;
  }
  if (nfft > 4096U) {
    nfft = 4096U;
  }
  if ((nfft & 1U) != 0U) {
    nfft++;
  }
  return nfft;
}

static uint32_t nr_v0_ssb_cp_from_nfft(uint32_t nfft)
{
  uint32_t cp = (uint32_t)llround((72.0 * (double)nfft) / 1024.0);
  return (cp > 0U) ? cp : 1U;
}

uint32_t nr_v0_ssb_nfft(double fs_hz)
{
  return nr_v0_ssb_nfft_from_fs(fs_hz);
}

uint32_t nr_v0_ssb_cp_len(double fs_hz)
{
  return nr_v0_ssb_cp_from_nfft(nr_v0_ssb_nfft_from_fs(fs_hz));
}

uint32_t nr_v0_pss_td_len(void)
{
  return nr_v0_ssb_sym_len_fs(NR_V0_FS_HZ_FALLBACK);
}

uint32_t nr_v0_ssb_sym_len(void)
{
  return nr_v0_ssb_sym_len_fs(NR_V0_FS_HZ_FALLBACK);
}

uint32_t nr_v0_ssb_burst_len(void)
{
  return nr_v0_ssb_burst_len_fs(NR_V0_FS_HZ_FALLBACK);
}

uint32_t nr_v0_ssb_sym_len_fs(double fs_hz)
{
  const uint32_t nfft = nr_v0_ssb_nfft_from_fs(fs_hz);
  return nfft + nr_v0_ssb_cp_from_nfft(nfft);
}

uint32_t nr_v0_ssb_burst_len_fs(double fs_hz)
{
  return NR_V0_SSB_SYMS * nr_v0_ssb_sym_len_fs(fs_hz);
}

static void nr_v0_map_real_seq_to_fd(const float *seq, uint32_t seq_len,
                                     float *Xr, float *Xi, uint32_t nfft,
                                     int first_sc)
{
  memset(Xr, 0, sizeof(*Xr) * (size_t)nfft);
  memset(Xi, 0, sizeof(*Xi) * (size_t)nfft);
  for (uint32_t m = 0; m < seq_len; m++) {
    const int k = first_sc + (int)m;
    int bin = (k >= 0) ? k : ((int)nfft + k);
    if (bin >= 0 && bin < (int)nfft) {
      Xr[(uint32_t)bin] = seq[m];
    }
  }
}

static int nr_v0_ofdm_mod_fd(const float *Xr, const float *Xi, uint32_t nfft,
                             uint32_t cp, float *td_i, float *td_q, uint32_t len)
{
  const uint32_t need = nfft + cp;
  if (!Xr || !Xi || !td_i || !td_q || len < need) {
    return -1;
  }

  float *xr = (float *)malloc(sizeof(*xr) * (size_t)nfft);
  float *xq = (float *)malloc(sizeof(*xq) * (size_t)nfft);
  if (!xr || !xq) {
    free(xr);
    free(xq);
    return -1;
  }

  for (uint32_t n = 0; n < nfft; n++) {
    double sr = 0.0;
    double si = 0.0;
    for (uint32_t k = 0; k < nfft; k++) {
      const double ph = 2.0 * M_PI * (double)(k * n) / (double)nfft;
      sr += (double)Xr[k] * cos(ph) - (double)Xi[k] * sin(ph);
      si += (double)Xr[k] * sin(ph) + (double)Xi[k] * cos(ph);
    }
    xr[n] = (float)(sr / (double)nfft);
    xq[n] = (float)(si / (double)nfft);
  }

  for (uint32_t n = 0; n < cp; n++) {
    td_i[n] = xr[nfft - cp + n];
    td_q[n] = xq[nfft - cp + n];
  }
  for (uint32_t n = 0; n < nfft; n++) {
    td_i[cp + n] = xr[n];
    td_q[cp + n] = xq[n];
  }

  free(xr);
  free(xq);
  return 0;
}

static void nr_v0_gen_prbs(uint32_t c_init, uint8_t *c, uint32_t len)
{
  enum { NC = 1600 };
  const uint32_t need = NC + len + 31U;
  uint8_t *x1 = (uint8_t *)calloc(need, sizeof(*x1));
  uint8_t *x2 = (uint8_t *)calloc(need, sizeof(*x2));
  if (!x1 || !x2) {
    free(x1);
    free(x2);
    if (c) {
      memset(c, 0, sizeof(*c) * (size_t)len);
    }
    return;
  }

  x1[0] = 1U;
  for (uint32_t i = 0; i < 31U; i++) {
    x2[i] = (uint8_t)((c_init >> i) & 1U);
  }
  for (uint32_t n = 0; n < (NC + len); n++) {
    x1[n + 31U] = (uint8_t)((x1[n + 3U] + x1[n]) & 1U);
    x2[n + 31U] = (uint8_t)((x2[n + 3U] + x2[n + 2U] + x2[n + 1U] + x2[n]) & 1U);
  }
  for (uint32_t n = 0; n < len; n++) {
    c[n] = (uint8_t)((x1[n + NC] + x2[n + NC]) & 1U);
  }

  free(x1);
  free(x2);
}

static void nr_v0_build_sss_seq(int nid1, int nid2, float *seq);

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

void nr_v0_sss_build_fd(int nid1, int nid2, float *seq, uint32_t len)
{
  if (!seq || len < NR_V0_PSS_LEN) {
    return;
  }
  nr_v0_build_sss_seq(nid1, nid2, seq);
}

void nr_v0_pss_build_td_f_fs(int nid2, double fs_hz,
                             float *td_i, float *td_q, uint32_t len)
{
  const uint32_t nfft = nr_v0_ssb_nfft_from_fs(fs_hz);
  const uint32_t cp = nr_v0_ssb_cp_from_nfft(nfft);
  const uint32_t need = nfft + cp;
  if (!td_i || !td_q || len < need) {
    return;
  }

  float pss[NR_V0_PSS_LEN];
  float *Xr = (float *)calloc(nfft, sizeof(*Xr));
  float *Xi = (float *)calloc(nfft, sizeof(*Xi));
  if (!Xr || !Xi) {
    free(Xr);
    free(Xi);
    return;
  }
  nr_v0_build_pss_seq(nid2, pss);
  nr_v0_map_real_seq_to_fd(pss, NR_V0_PSS_LEN, Xr, Xi, nfft, -64);
  (void)nr_v0_ofdm_mod_fd(Xr, Xi, nfft, cp, td_i, td_q, len);
  free(Xr);
  free(Xi);
}

void nr_v0_pss_build_td_f(int nid2, float *td_i, float *td_q, uint32_t len)
{
  nr_v0_pss_build_td_f_fs(nid2, NR_V0_FS_HZ_FALLBACK, td_i, td_q, len);
}

int nr_v0_pss_build_td_iq(int nid2, c16_t *out, uint32_t out_len, float amp)
{
  const uint32_t need = nr_v0_pss_td_len();
  if (!out || out_len < need) {
    return -1;
  }
  float *ti = (float *)malloc(sizeof(*ti) * (size_t)need);
  float *tq = (float *)malloc(sizeof(*tq) * (size_t)need);
  if (!ti || !tq) {
    free(ti);
    free(tq);
    return -1;
  }
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
  free(ti);
  free(tq);
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

static void nr_v0_sss_build_td_f_impl(int nid1, int nid2, double fs_hz,
                                      float *td_i, float *td_q, uint32_t len)
{
  const uint32_t nfft = nr_v0_ssb_nfft_from_fs(fs_hz);
  const uint32_t cp = nr_v0_ssb_cp_from_nfft(nfft);
  const uint32_t need = nfft + cp;
  if (!td_i || !td_q || len < need) {
    return;
  }

  float sss[NR_V0_PSS_LEN];
  float *Xr = (float *)calloc(nfft, sizeof(*Xr));
  float *Xi = (float *)calloc(nfft, sizeof(*Xi));
  if (!Xr || !Xi) {
    free(Xr);
    free(Xi);
    return;
  }
  nr_v0_build_sss_seq(nid1, nid2, sss);
  nr_v0_map_real_seq_to_fd(sss, NR_V0_PSS_LEN, Xr, Xi, nfft, -64);
  (void)nr_v0_ofdm_mod_fd(Xr, Xi, nfft, cp, td_i, td_q, len);
  free(Xr);
  free(Xi);
}

static int nr_v0_sss_build_td_iq(int nid1, int nid2,
                                 c16_t *out, uint32_t out_len, float amp)
{
  const uint32_t need = nr_v0_pss_td_len();
  if (!out || out_len < need) {
    return -1;
  }
  float *ti = (float *)malloc(sizeof(*ti) * (size_t)need);
  float *tq = (float *)malloc(sizeof(*tq) * (size_t)need);
  if (!ti || !tq) {
    free(ti);
    free(tq);
    return -1;
  }
  nr_v0_sss_build_td_f_impl(nid1, nid2, NR_V0_FS_HZ_FALLBACK, ti, tq, need);
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
  free(ti);
  free(tq);
  return (int)need;
}

/* Public wrapper: build SSS time-domain reference (float IQ) for correlation. */
void nr_v0_sss_build_td_f(int nid1, int nid2,
                           float *td_i, float *td_q, uint32_t len)
{
  nr_v0_sss_build_td_f_impl(nid1, nid2, NR_V0_FS_HZ_FALLBACK, td_i, td_q, len);
}

void nr_v0_sss_build_td_f_fs(int nid1, int nid2, double fs_hz,
                             float *td_i, float *td_q, uint32_t len)
{
  nr_v0_sss_build_td_f_impl(nid1, nid2, fs_hz, td_i, td_q, len);
}

int nr_v0_pbch_dmrs_build(int pci, int ssb_idx, int n_hf,
                          float *seq_i, float *seq_q, uint32_t max_len)
{
  enum { DMRS_RE = 144 };
  uint8_t c[2U * DMRS_RE];
  int i_ssb = (ssb_idx >= 0) ? ssb_idx : 0;
  if (i_ssb < 4 && (n_hf == 0 || n_hf == 1)) {
    i_ssb += 4 * n_hf;
  }
  if (!seq_i || !seq_q || max_len < DMRS_RE) {
    return -1;
  }
  const uint32_t c_init =
      ((((uint32_t)i_ssb + 1U) * (((uint32_t)pci >> 2) + 1U)) << 11) +
      (((uint32_t)i_ssb + 1U) << 6) + ((uint32_t)pci & 3U);
  nr_v0_gen_prbs(c_init, c, 2U * DMRS_RE);
  for (uint32_t m = 0; m < DMRS_RE; m++) {
    seq_i[m] = c[2U * m] ? -0.70710678118f : 0.70710678118f;
    seq_q[m] = c[2U * m + 1U] ? -0.70710678118f : 0.70710678118f;
  }
  return DMRS_RE;
}

int nr_v0_ssb_build_burst_iq(int nid1, int nid2,
                              c16_t *out, uint32_t out_len, float amp)
{
  const uint32_t nfft = nr_v0_ssb_nfft_from_fs(NR_V0_FS_HZ_FALLBACK);
  const uint32_t cp = nr_v0_ssb_cp_from_nfft(nfft);
  const uint32_t sym_len = nfft + cp;
  const uint32_t burst_len = nr_v0_ssb_burst_len();
  if (!out || out_len < burst_len) {
    return -1;
  }
  memset(out, 0, sizeof(*out) * out_len);
  const int pci = 3 * nid1 + nid2;
  const uint8_t v = (uint8_t)(pci & 3);
  float dmrs_i[144];
  float dmrs_q[144];
  uint16_t dmrs_rel[144];
  uint8_t dmrs_sym[144];
  uint16_t data_rel[432];
  uint8_t data_sym[432];
  uint8_t coded_bits[864];
  float pbch_i[432];
  float pbch_q[432];
  float *Xr = (float *)calloc(nfft, sizeof(*Xr));
  float *Xi = (float *)calloc(nfft, sizeof(*Xi));
  float *td_i = (float *)malloc(sizeof(*td_i) * (size_t)sym_len);
  float *td_q = (float *)malloc(sizeof(*td_q) * (size_t)sym_len);
  if (!Xr || !Xi || !td_i || !td_q) {
    free(Xr);
    free(Xi);
    free(td_i);
    free(td_q);
    return -1;
  }

  if (nr_v0_pbch_dmrs_build(pci, NR_V0_SYNTH_PBCH_SSB_IDX, 0, dmrs_i, dmrs_q, 144U) != 144) {
    free(Xr);
    free(Xi);
    free(td_i);
    free(td_q);
    return -1;
  }
  if (nr_pbch_dmrs_re_positions(v, dmrs_rel, dmrs_sym, 144U) != 144U) {
    free(Xr);
    free(Xi);
    free(td_i);
    free(td_q);
    return -1;
  }
  if (nr_pbch_data_re_positions(v, data_rel, data_sym, 432U) != 432U) {
    free(Xr);
    free(Xi);
    free(td_i);
    free(td_q);
    return -1;
  }
  if (nr_pbch_bch_encode((uint16_t)pci, NR_V0_SYNTH_PBCH_SSB_IDX,
                         NR_V0_SYNTH_PBCH_SFN, NR_V0_SYNTH_PBCH_HRF,
                         NR_V0_SYNTH_PBCH_MIB_PAYLOAD,
                         coded_bits, 864U) != 864) {
    free(Xr);
    free(Xi);
    free(td_i);
    free(td_q);
    return -1;
  }
  for (uint32_t m = 0U; m < 432U; m++) {
    pbch_i[m] = coded_bits[2U * m] ? -0.70710678118f : 0.70710678118f;
    pbch_q[m] = coded_bits[2U * m + 1U] ? -0.70710678118f : 0.70710678118f;
  }

  for (uint32_t sym = 0; sym < NR_V0_SSB_SYMS; sym++) {
    memset(Xr, 0, sizeof(*Xr) * (size_t)nfft);
    memset(Xi, 0, sizeof(*Xi) * (size_t)nfft);

    if (sym == 0U) {
      float pss[NR_V0_PSS_LEN];
      nr_v0_build_pss_seq(nid2, pss);
      nr_v0_map_real_seq_to_fd(pss, NR_V0_PSS_LEN, Xr, Xi, nfft, -64);
    } else if (sym == 2U) {
      float sss[NR_V0_PSS_LEN];
      nr_v0_build_sss_seq(nid1, nid2, sss);
      for (uint32_t m = 0; m < NR_V0_PSS_LEN; m++) {
        const int k = -120 + 56 + (int)m;
        int bin = (k >= 0) ? k : ((int)nfft + k);
        Xr[(uint32_t)bin] = sss[m];
      }
    }

    for (uint32_t m = 0U; m < 144U; m++) {
      if (dmrs_sym[m] == sym) {
        const int k = (int)dmrs_rel[m] - 120;
        const int bin = (k >= 0) ? k : ((int)nfft + k);
        Xr[(uint32_t)bin] = dmrs_i[m];
        Xi[(uint32_t)bin] = dmrs_q[m];
      }
    }
    for (uint32_t m = 0U; m < 432U; m++) {
      if (data_sym[m] == sym) {
        const int k = (int)data_rel[m] - 120;
        const int bin = (k >= 0) ? k : ((int)nfft + k);
        Xr[(uint32_t)bin] = pbch_i[m];
        Xi[(uint32_t)bin] = pbch_q[m];
      }
    }

    if (nr_v0_ofdm_mod_fd(Xr, Xi, nfft, cp, td_i, td_q, sym_len) != 0) {
      free(Xr);
      free(Xi);
      free(td_i);
      free(td_q);
      return -1;
    }
    for (uint32_t n = 0; n < sym_len; n++) {
      float ir = amp * td_i[n];
      float iq = amp * td_q[n];
      if (ir > 32767.0f) ir = 32767.0f;
      if (ir < -32768.0f) ir = -32768.0f;
      if (iq > 32767.0f) iq = 32767.0f;
      if (iq < -32768.0f) iq = -32768.0f;
      out[sym * sym_len + n].r = (int16_t)lrintf(ir);
      out[sym * sym_len + n].i = (int16_t)lrintf(iq);
    }
  }

  free(Xr);
  free(Xi);
  free(td_i);
  free(td_q);
  return (int)burst_len;
}

void nr_ssb_ref_cache_touch(const nr_ssb_ref_t *ref)
{
  (void)ref;
}
