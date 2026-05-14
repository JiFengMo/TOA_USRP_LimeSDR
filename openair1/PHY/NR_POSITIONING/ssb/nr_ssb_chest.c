#include "openair1/PHY/NR_POSITIONING/nr_pos_api.h"
#include "openairinterface5g/openair1/PHY/NR_UE_ESTIMATION/filt16a_32.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#define NR_SSB_TOTAL_RE (NR_SSB_RE_ROWS * NR_SSB_RE_COLS)

static inline uint32_t nr_ssb_re_idx(uint8_t sym, uint16_t rel)
{
  return (uint32_t)sym * NR_SSB_RE_COLS + (uint32_t)rel;
}

static void nr_ssb_pbch_select_oai_filters(uint8_t v,
                                           const int16_t **fl,
                                           const int16_t **fm,
                                           const int16_t **fr)
{
  switch (v & 3U) {
    case 0U:
      *fl = filt16a_l0;
      *fm = filt16a_m0;
      *fr = filt16a_r0;
      break;
    case 1U:
      *fl = filt16a_l1;
      *fm = filt16a_m1;
      *fr = filt16a_r1;
      break;
    case 2U:
      *fl = filt16a_l2;
      *fm = filt16a_m2;
      *fr = filt16a_r2;
      break;
    default:
      *fl = filt16a_l3;
      *fm = filt16a_m3;
      *fr = filt16a_r3;
      break;
  }
}

static cf32_t nr_ssb_pbch_dmrs_ls(const cf32_t y, float xr, float xi)
{
  const float den = xr * xr + xi * xi + 1.0e-6f;
  return (cf32_t){
      .r = (y.r * xr + y.i * xi) / den,
      .i = (y.i * xr - y.r * xi) / den,
  };
}

static void nr_ssb_pbch_filter_accumulate(cf32_t *dst,
                                          uint8_t *valid,
                                          uint8_t sym,
                                          uint16_t rel0,
                                          cf32_t h_ls,
                                          const int16_t *filt16)
{
  if (!dst || !valid || !filt16 || sym >= NR_SSB_RE_ROWS) {
    return;
  }

  for (uint32_t t = 0U; t < 16U; t++) {
    const uint32_t rel = (uint32_t)rel0 + t;
    const float w = (float)filt16[t] / 16384.0f;
    const uint32_t idx = nr_ssb_re_idx(sym, (uint16_t)rel);
    if (rel >= NR_SSB_RE_COLS || w == 0.0f) {
      continue;
    }
    dst[idx].r += w * h_ls.r;
    dst[idx].i += w * h_ls.i;
    valid[idx] = 1U;
  }
}

static int nr_ssb_pbch_oai_channel_estimation(const nr_ssb_grid_t *grid,
                                              uint16_t pci,
                                              uint8_t ssb_idx,
                                              uint8_t n_hf,
                                              nr_chest_full_t *hf,
                                              float *dmrs_metric)
{
  static __thread cf32_t tls_h_full_pbch[NR_SSB_TOTAL_RE];
  static __thread uint8_t tls_valid_pbch[NR_SSB_TOTAL_RE];
  float dmrs_i[144];
  float dmrs_q[144];
  uint16_t dmrs_rel[144];
  uint8_t dmrs_sym[144];
  const int16_t *fl = NULL;
  const int16_t *fm = NULL;
  const int16_t *fr = NULL;
  const uint8_t v = (uint8_t)(pci & 3U);
  double cr = 0.0;
  double ci = 0.0;
  double px = 0.0;
  const double pr = 144.0;
  uint32_t pilot = 0U;

  if (dmrs_metric) {
    *dmrs_metric = -1.0f;
  }
  if (!grid || !grid->valid || !hf) {
    return -1;
  }
  if (nr_pbch_dmrs_re_positions(v, dmrs_rel, dmrs_sym, 144U) != 144U) {
    return -1;
  }
  if (nr_v0_pbch_dmrs_build((int)pci, (int)ssb_idx, (int)n_hf, dmrs_i, dmrs_q, 144U) != 144) {
    return -1;
  }

  nr_ssb_pbch_select_oai_filters(v, &fl, &fm, &fr);
  memset(tls_h_full_pbch, 0, sizeof(tls_h_full_pbch));
  memset(tls_valid_pbch, 0, sizeof(tls_valid_pbch));

  for (uint8_t sym = 1U; sym <= 3U; sym++) {
    const uint16_t seg0 = (sym == 2U) ? 48U : 240U;
    for (uint16_t rb0 = 0U; rb0 < seg0; rb0 += 12U) {
      const cf32_t y0 = grid->re[sym][rb0 + v];
      const cf32_t y1 = grid->re[sym][rb0 + v + 4U];
      const cf32_t y2 = grid->re[sym][rb0 + v + 8U];
      const cf32_t h0 = nr_ssb_pbch_dmrs_ls(y0, dmrs_i[pilot], dmrs_q[pilot]);
      cr += (double)y0.r * (double)dmrs_i[pilot] + (double)y0.i * (double)dmrs_q[pilot];
      ci += (double)y0.i * (double)dmrs_i[pilot] - (double)y0.r * (double)dmrs_q[pilot];
      px += (double)y0.r * (double)y0.r + (double)y0.i * (double)y0.i;
      pilot++;
      const cf32_t h1 = nr_ssb_pbch_dmrs_ls(y1, dmrs_i[pilot], dmrs_q[pilot]);
      cr += (double)y1.r * (double)dmrs_i[pilot] + (double)y1.i * (double)dmrs_q[pilot];
      ci += (double)y1.i * (double)dmrs_i[pilot] - (double)y1.r * (double)dmrs_q[pilot];
      px += (double)y1.r * (double)y1.r + (double)y1.i * (double)y1.i;
      pilot++;
      const cf32_t h2 = nr_ssb_pbch_dmrs_ls(y2, dmrs_i[pilot], dmrs_q[pilot]);
      cr += (double)y2.r * (double)dmrs_i[pilot] + (double)y2.i * (double)dmrs_q[pilot];
      ci += (double)y2.i * (double)dmrs_i[pilot] - (double)y2.r * (double)dmrs_q[pilot];
      px += (double)y2.r * (double)y2.r + (double)y2.i * (double)y2.i;
      pilot++;
      nr_ssb_pbch_filter_accumulate(tls_h_full_pbch, tls_valid_pbch, sym, rb0, h0, fl);
      nr_ssb_pbch_filter_accumulate(tls_h_full_pbch, tls_valid_pbch, sym, rb0, h1, fm);
      nr_ssb_pbch_filter_accumulate(tls_h_full_pbch, tls_valid_pbch, sym, rb0, h2, fr);
    }

    if (sym == 2U) {
      for (uint16_t rb0 = 192U; rb0 < 240U; rb0 += 12U) {
        const cf32_t y0 = grid->re[sym][rb0 + v];
        const cf32_t y1 = grid->re[sym][rb0 + v + 4U];
        const cf32_t y2 = grid->re[sym][rb0 + v + 8U];
        const cf32_t h0 = nr_ssb_pbch_dmrs_ls(y0, dmrs_i[pilot], dmrs_q[pilot]);
        cr += (double)y0.r * (double)dmrs_i[pilot] + (double)y0.i * (double)dmrs_q[pilot];
        ci += (double)y0.i * (double)dmrs_i[pilot] - (double)y0.r * (double)dmrs_q[pilot];
        px += (double)y0.r * (double)y0.r + (double)y0.i * (double)y0.i;
        pilot++;
        const cf32_t h1 = nr_ssb_pbch_dmrs_ls(y1, dmrs_i[pilot], dmrs_q[pilot]);
        cr += (double)y1.r * (double)dmrs_i[pilot] + (double)y1.i * (double)dmrs_q[pilot];
        ci += (double)y1.i * (double)dmrs_i[pilot] - (double)y1.r * (double)dmrs_q[pilot];
        px += (double)y1.r * (double)y1.r + (double)y1.i * (double)y1.i;
        pilot++;
        const cf32_t h2 = nr_ssb_pbch_dmrs_ls(y2, dmrs_i[pilot], dmrs_q[pilot]);
        cr += (double)y2.r * (double)dmrs_i[pilot] + (double)y2.i * (double)dmrs_q[pilot];
        ci += (double)y2.i * (double)dmrs_i[pilot] - (double)y2.r * (double)dmrs_q[pilot];
        px += (double)y2.r * (double)y2.r + (double)y2.i * (double)y2.i;
        pilot++;
        nr_ssb_pbch_filter_accumulate(tls_h_full_pbch, tls_valid_pbch, sym, rb0, h0, fl);
        nr_ssb_pbch_filter_accumulate(tls_h_full_pbch, tls_valid_pbch, sym, rb0, h1, fm);
        nr_ssb_pbch_filter_accumulate(tls_h_full_pbch, tls_valid_pbch, sym, rb0, h2, fr);
      }
    }
  }

  if (pilot != 144U) {
    return -1;
  }

  hf->h_full = tls_h_full_pbch;
  hf->valid_re = tls_valid_pbch;
  hf->n_re = NR_SSB_TOTAL_RE;
  if (dmrs_metric) {
    *dmrs_metric = (float)(sqrt(cr * cr + ci * ci) / (sqrt(px * pr) + 1.0e-9));
  }
  return 0;
}

float nr_ssb_pbch_dmrs_corr_metric(const nr_ssb_grid_t *grid,
                                   uint16_t pci,
                                   uint8_t ssb_idx,
                                   uint8_t n_hf)
{
  float dmrs_i[144];
  float dmrs_q[144];
  uint16_t dmrs_rel[144];
  uint8_t dmrs_sym[144];
  const uint8_t v = (uint8_t)(pci & 3U);
  double cumul_r = 0.0;
  double cumul_i = 0.0;
  double rx_pow = 0.0;
  double ref_pow = 0.0;

  if (!grid || !grid->valid) {
    return -1.0f;
  }

  if (nr_pbch_dmrs_re_positions(v, dmrs_rel, dmrs_sym, 144U) != 144U) {
    return -1.0f;
  }
  if (nr_v0_pbch_dmrs_build((int)pci, (int)ssb_idx, (int)n_hf, dmrs_i, dmrs_q, 144U) != 144) {
    return -1.0f;
  }

  for (uint8_t sym = 1U; sym <= 3U; sym++) {
    double sym_r = 0.0;
    double sym_i = 0.0;
    for (uint32_t m = 0U; m < 144U; m++) {
      if (dmrs_sym[m] != sym) {
        continue;
      }
      const uint16_t rel = dmrs_rel[m];
      const cf32_t y = grid->re[sym][rel];
      const float xr = dmrs_i[m];
      const float xi = dmrs_q[m];
      sym_r += (double)y.r * (double)xr + (double)y.i * (double)xi;
      sym_i += (double)y.i * (double)xr - (double)y.r * (double)xi;
      rx_pow += (double)y.r * (double)y.r + (double)y.i * (double)y.i;
      ref_pow += (double)xr * (double)xr + (double)xi * (double)xi;
    }
    cumul_r += sym_r;
    cumul_i += sym_i;
  }
  return (float)(sqrt(cumul_r * cumul_r + cumul_i * cumul_i) /
                 (sqrt(rx_pow * ref_pow) + 1.0e-9));
}

static float nr_ssb_pbch_noise_cpe(const nr_ssb_grid_t *grid,
                                   const nr_chest_full_t *hf,
                                   uint16_t pci,
                                   uint8_t ssb_idx,
                                   uint8_t n_hf,
                                   float *cpe_rad)
{
  float dmrs_i[144];
  float dmrs_q[144];
  uint16_t dmrs_rel[144];
  uint8_t dmrs_sym[144];
  const uint8_t v = (uint8_t)(pci & 3U);
  double err_pow = 0.0;
  double sig_pow = 0.0;
  double cr = 0.0;
  double ci = 0.0;
  uint32_t used = 0U;

  if (cpe_rad) {
    *cpe_rad = 0.0f;
  }
  if (!grid || !grid->valid || !hf || !hf->h_full) {
    return 1.0f;
  }
  if (nr_pbch_dmrs_re_positions(v, dmrs_rel, dmrs_sym, 144U) != 144U) {
    return 1.0f;
  }
  if (nr_v0_pbch_dmrs_build((int)pci, (int)ssb_idx, (int)n_hf, dmrs_i, dmrs_q, 144U) != 144) {
    return 1.0f;
  }

  for (uint32_t m = 0U; m < 144U; m++) {
    const uint8_t sym = dmrs_sym[m];
    const uint16_t rel = dmrs_rel[m];
    const cf32_t y = grid->re[sym][rel];
    const cf32_t h = hf->h_full[nr_ssb_re_idx(sym, rel)];
    const float xr = dmrs_i[m];
    const float xi = dmrs_q[m];
    const float pr = h.r * xr - h.i * xi;
    const float pi = h.r * xi + h.i * xr;
    const float er = y.r - pr;
    const float ei = y.i - pi;
    err_pow += (double)er * (double)er + (double)ei * (double)ei;
    sig_pow += (double)pr * (double)pr + (double)pi * (double)pi;
    cr += (double)y.r * (double)pr + (double)y.i * (double)pi;
    ci += (double)y.i * (double)pr - (double)y.r * (double)pi;
    used++;
  }

  if (used == 0U) {
    return 1.0f;
  }
  if (cpe_rad) {
    *cpe_rad = (float)atan2(ci, cr);
  }
  return (float)((err_pow / (double)used) / ((sig_pow / (double)used) + 1.0e-6));
}

static int nr_ssb_pbch_build_llr(const nr_ssb_grid_t *grid,
                                 const nr_chest_full_t *hf,
                                 uint16_t pci,
                                 float noise_var,
                                 float cpe_rad,
                                 float *llr)
{
  uint16_t data_rel[432];
  uint8_t data_sym[432];
  const uint8_t v = (uint8_t)(pci & 3U);
  const float cph = cosf(-cpe_rad);
  const float sph = sinf(-cpe_rad);

  if (!grid || !grid->valid || !hf || !hf->h_full || !llr) {
    return -1;
  }
  if (nr_pbch_data_re_positions(v, data_rel, data_sym, 432U) != 432U) {
    return -1;
  }

  for (uint32_t m = 0U; m < 432U; m++) {
    const uint8_t sym = data_sym[m];
    const uint16_t rel = data_rel[m];
    const cf32_t y = grid->re[sym][rel];
    const cf32_t h = hf->h_full[nr_ssb_re_idx(sym, rel)];
    const float ch_pow = h.r * h.r + h.i * h.i + 1.0e-6f;
    const float zr = y.r * h.r + y.i * h.i;
    const float zi = y.i * h.r - y.r * h.i;
    const float rr = zr * cph - zi * sph;
    const float ri = zr * sph + zi * cph;
    const float eff_noise = fmaxf(noise_var * ch_pow, 0.05f);
    llr[2U * m] = 2.0f * rr / eff_noise;
    llr[2U * m + 1U] = 2.0f * ri / eff_noise;
  }
  return 0;
}

static void nr_ssb_ls_from_ref(const nr_ssb_grid_t *grid,
                               uint8_t sym,
                               uint16_t rel,
                               float xr,
                               float xi,
                               cf32_t *h_ls,
                               uint8_t *valid)
{
  const uint32_t idx = nr_ssb_re_idx(sym, rel);
  const cf32_t y = grid->re[sym][rel];
  const float den = xr * xr + xi * xi + 1.0e-6f;
  h_ls[idx].r = (y.r * xr + y.i * xi) / den;
  h_ls[idx].i = (y.i * xr - y.r * xi) / den;
  valid[idx] = 1U;
}

static int nr_ssb_interp_symbol(const nr_chest_t *h, uint8_t sym, uint16_t rel, cf32_t *hout)
{
  int left = -1;
  int right = -1;
  if (!h || !h->h_ls || !h->valid_re || !hout || sym >= NR_SSB_RE_ROWS || rel >= NR_SSB_RE_COLS) {
    return -1;
  }
  for (int k = (int)rel; k >= 0; k--) {
    if (h->valid_re[nr_ssb_re_idx(sym, (uint16_t)k)]) {
      left = k;
      break;
    }
  }
  for (int k = (int)rel; k < NR_SSB_RE_COLS; k++) {
    if (h->valid_re[nr_ssb_re_idx(sym, (uint16_t)k)]) {
      right = k;
      break;
    }
  }
  if (left < 0 && right < 0) {
    return -1;
  }
  if (left < 0) {
    *hout = h->h_ls[nr_ssb_re_idx(sym, (uint16_t)right)];
    return 0;
  }
  if (right < 0 || right == left) {
    *hout = h->h_ls[nr_ssb_re_idx(sym, (uint16_t)left)];
    return 0;
  }
  {
    const cf32_t a = h->h_ls[nr_ssb_re_idx(sym, (uint16_t)left)];
    const cf32_t b = h->h_ls[nr_ssb_re_idx(sym, (uint16_t)right)];
    const float t = (float)((int)rel - left) / (float)(right - left);
    hout->r = (1.0f - t) * a.r + t * b.r;
    hout->i = (1.0f - t) * a.i + t * b.i;
  }
  return 0;
}

int nr_ssb_ls_estimate(const nr_ssb_grid_t *grid, const nr_sync_state_t *sync, nr_chest_t *h)
{
  static __thread cf32_t tls_h_ls[NR_SSB_TOTAL_RE];
  static __thread uint8_t tls_valid[NR_SSB_TOTAL_RE];
  float pss_ref[127];
  float sss_ref[127];
  float dmrs_i[144];
  float dmrs_q[144];
  uint16_t dmrs_rel[144];
  uint8_t dmrs_sym[144];
  const uint16_t pci = sync ? sync->pci : 0U;
  const uint8_t nid2 = (uint8_t)(pci % 3U);
  const uint16_t nid1 = (uint16_t)(pci / 3U);
  const uint8_t v = (uint8_t)(pci & 3U);

  if (!grid || !grid->valid || !sync || !h) {
    return -1;
  }

  memset(tls_h_ls, 0, sizeof(tls_h_ls));
  memset(tls_valid, 0, sizeof(tls_valid));

  nr_v0_pss_build_fd((int)nid2, pss_ref, 127U);
  nr_v0_sss_build_fd((int)nid1, (int)nid2, sss_ref, 127U);
  for (uint16_t m = 0U; m < 127U; m++) {
    nr_ssb_ls_from_ref(grid, 0U, (uint16_t)(56U + m), pss_ref[m], 0.0f, tls_h_ls, tls_valid);
    nr_ssb_ls_from_ref(grid, 2U, (uint16_t)(56U + m), sss_ref[m], 0.0f, tls_h_ls, tls_valid);
  }

  if (nr_pbch_dmrs_re_positions(v, dmrs_rel, dmrs_sym, 144U) != 144U) {
    return -1;
  }
  if (nr_v0_pbch_dmrs_build((int)pci, (int)sync->ssb_index,
                            (int)sync->pbch_dmrs_n_hf, dmrs_i, dmrs_q, 144U) != 144) {
    return -1;
  }
  for (uint32_t m = 0U; m < 144U; m++) {
    nr_ssb_ls_from_ref(grid, dmrs_sym[m], dmrs_rel[m], dmrs_i[m], dmrs_q[m], tls_h_ls, tls_valid);
  }

  h->h_ls = tls_h_ls;
  h->valid_re = tls_valid;
  h->n_re = NR_SSB_TOTAL_RE;
  return 0;
}

int nr_ssb_interp_channel(const nr_chest_t *h, nr_chest_full_t *hf)
{
  static __thread cf32_t tls_h_full[NR_SSB_TOTAL_RE];
  static __thread cf32_t tls_h_smooth[NR_SSB_TOTAL_RE];
  static __thread uint8_t tls_valid[NR_SSB_TOTAL_RE];

  if (!h || !h->h_ls || !h->valid_re || !hf) {
    return -1;
  }

  memset(tls_h_full, 0, sizeof(tls_h_full));
  memset(tls_h_smooth, 0, sizeof(tls_h_smooth));
  memset(tls_valid, 1, sizeof(tls_valid));

  for (uint8_t sym = 0U; sym < NR_SSB_RE_ROWS; sym++) {
    for (uint16_t rel = 0U; rel < NR_SSB_RE_COLS; rel++) {
      cf32_t val = {0};
      const uint32_t idx = nr_ssb_re_idx(sym, rel);
      if (h->valid_re[idx]) {
        tls_h_full[idx] = h->h_ls[idx];
      } else if (nr_ssb_interp_symbol(h, sym, rel, &val) == 0) {
        tls_h_full[idx] = val;
      }
    }
  }

  for (uint8_t sym = 0U; sym < NR_SSB_RE_ROWS; sym++) {
    for (uint16_t rel = 0U; rel < NR_SSB_RE_COLS; rel++) {
      cf32_t acc = {0};
      float wsum = 0.0f;
      for (int ds = -1; ds <= 1; ds++) {
        const int sym2 = (int)sym + ds;
        const float w = (ds == 0) ? 0.5f : 0.25f;
        if (sym2 < 0 || sym2 >= NR_SSB_RE_ROWS) {
          continue;
        }
        acc.r += w * tls_h_full[nr_ssb_re_idx((uint8_t)sym2, rel)].r;
        acc.i += w * tls_h_full[nr_ssb_re_idx((uint8_t)sym2, rel)].i;
        wsum += w;
      }
      if (wsum > 0.0f) {
        acc.r /= wsum;
        acc.i /= wsum;
      }
      tls_h_smooth[nr_ssb_re_idx(sym, rel)] = acc;
    }
  }

  hf->h_full = tls_h_smooth;
  hf->valid_re = tls_valid;
  hf->n_re = NR_SSB_TOTAL_RE;
  return 0;
}

int nr_ssb_build_cir(const nr_chest_full_t *hf, nr_cir_t *cir)
{
  static __thread cf32_t tls_cir[NR_SSB_RE_COLS];
  cf32_t h_avg[NR_SSB_RE_COLS];
  float max_mag = 0.0f;
  float mean_mag = 0.0f;

  if (!hf || !hf->h_full || !cir) {
    return -1;
  }

  memset(tls_cir, 0, sizeof(tls_cir));
  memset(h_avg, 0, sizeof(h_avg));

  for (uint16_t rel = 0U; rel < NR_SSB_RE_COLS; rel++) {
    for (uint8_t sym = 0U; sym < NR_SSB_RE_ROWS; sym++) {
      h_avg[rel].r += hf->h_full[nr_ssb_re_idx(sym, rel)].r;
      h_avg[rel].i += hf->h_full[nr_ssb_re_idx(sym, rel)].i;
    }
    h_avg[rel].r *= 0.25f;
    h_avg[rel].i *= 0.25f;
  }

  for (uint32_t n = 0U; n < NR_SSB_RE_COLS; n++) {
    double sr = 0.0;
    double si = 0.0;
    for (uint32_t rel = 0U; rel < NR_SSB_RE_COLS; rel++) {
      const int k = (int)rel - 120;
      const double ph = 2.0 * M_PI * (double)k * (double)n / (double)NR_SSB_RE_COLS;
      const double c = cos(ph);
      const double s = sin(ph);
      sr += (double)h_avg[rel].r * c - (double)h_avg[rel].i * s;
      si += (double)h_avg[rel].r * s + (double)h_avg[rel].i * c;
    }
    tls_cir[n].r = (float)(sr / (double)NR_SSB_RE_COLS);
    tls_cir[n].i = (float)(si / (double)NR_SSB_RE_COLS);
    {
      const float mag = hypotf(tls_cir[n].r, tls_cir[n].i);
      if (mag > max_mag) {
        max_mag = mag;
      }
      mean_mag += mag;
    }
  }

  cir->cir = tls_cir;
  cir->cir_len = NR_SSB_RE_COLS;
  cir->os_factor = 1U;
  cir->peak_metric = max_mag / ((mean_mag / (float)NR_SSB_RE_COLS) + 1.0e-6f);
  return 0;
}

int nr_ssb_pbch_prepare_frontend(const nr_ssb_grid_t *grid,
                                 uint16_t pci,
                                 uint8_t ssb_idx,
                                 uint8_t n_hf,
                                 float *dmrs_metric,
                                 float *noise_var,
                                 float *cpe_rad,
                                 float *llr)
{
  nr_chest_full_t hf;
  float local_metric = -1.0f;
  float local_noise = 1.0f;
  float local_cpe = 0.0f;

  memset(&hf, 0, sizeof(hf));

  if (nr_ssb_pbch_oai_channel_estimation(grid, pci, ssb_idx, n_hf, &hf, &local_metric) != 0) {
    return -1;
  }

  local_noise = nr_ssb_pbch_noise_cpe(grid, &hf, pci, ssb_idx, n_hf, &local_cpe);
  if (llr && nr_ssb_pbch_build_llr(grid, &hf, pci, local_noise, local_cpe, llr) != 0) {
    return -1;
  }

  if (dmrs_metric) {
    *dmrs_metric = local_metric;
  }
  if (noise_var) {
    *noise_var = local_noise;
  }
  if (cpe_rad) {
    *cpe_rad = local_cpe;
  }
  return 0;
}
