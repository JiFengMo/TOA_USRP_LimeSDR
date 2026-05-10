#include "openair1/PHY/NR_POSITIONING/nr_pos_api.h"

#include <math.h>
#include <string.h>
#include <stdlib.h>

static void nr_pdcch_gen_prbs(uint32_t c_init, uint8_t *c, uint32_t len)
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
  for (uint32_t i = 0U; i < 31U; i++) {
    x2[i] = (uint8_t)((c_init >> i) & 1U);
  }
  for (uint32_t n = 0U; n < (NC + len); n++) {
    x1[n + 31U] = (uint8_t)((x1[n + 3U] + x1[n]) & 1U);
    x2[n + 31U] = (uint8_t)((x2[n + 3U] + x2[n + 2U] + x2[n + 1U] + x2[n]) & 1U);
  }
  for (uint32_t n = 0U; n < len; n++) {
    c[n] = (uint8_t)((x1[n + NC] + x2[n + NC]) & 1U);
  }

  free(x1);
  free(x2);
}

static uint32_t nr_pdcch_dmrs_cinit(uint16_t nid, uint32_t slot, uint32_t symbol)
{
  const uint64_t x2tmp0 =
      ((((uint64_t)14U * slot + symbol + 1U) * ((((uint64_t)nid) << 1U) + 1U)) << 17U);
  return (uint32_t)((x2tmp0 + (((uint64_t)nid) << 1U)) % (1ULL << 31U));
}

static int nr_pdcch_dmrs_ref_build(uint16_t nid,
                                   uint32_t slot,
                                   uint32_t symbol,
                                   uint32_t num_rbs,
                                   cf32_t *out,
                                   uint32_t max_re)
{
  const uint32_t num_re = num_rbs * NR_PDCCH_DMRS_RE_PER_RB;
  const uint32_t num_bits = num_re * 2U;
  uint8_t bits[NR_CORESET0_MAX_RB * NR_PDCCH_DMRS_RE_PER_RB * 2U];
  const float inv_sqrt2 = 0.70710678118f;

  if (!out || num_re > max_re || num_bits > (uint32_t)sizeof(bits)) {
    return -1;
  }

  nr_pdcch_gen_prbs(nr_pdcch_dmrs_cinit(nid, slot, symbol), bits, num_bits);

  for (uint32_t m = 0U; m < num_re; m++) {
    const uint8_t b0 = bits[2U * m];
    const uint8_t b1 = bits[2U * m + 1U];
    out[m].r = b0 ? -inv_sqrt2 : inv_sqrt2;
    out[m].i = b1 ? -inv_sqrt2 : inv_sqrt2;
  }

  return (int)num_re;
}

static int nr_is_pdcch_dmrs_sc(uint32_t sc_in_rb)
{
  return (sc_in_rb == 1U || sc_in_rb == 5U || sc_in_rb == 9U);
}

static int nr_comp_u16(const void *a, const void *b)
{
  const uint16_t av = *(const uint16_t *)a;
  const uint16_t bv = *(const uint16_t *)b;
  return (av > bv) - (av < bv);
}

static uint8_t nr_ceil_log2_u32(uint32_t x)
{
  uint8_t bits = 0U;

  if (x <= 1U) {
    return 0U;
  }

  x--;
  while (x > 0U) {
    bits++;
    x >>= 1U;
  }

  return bits;
}

static uint32_t nr_dci_crc24c_bits(const uint8_t *bits, uint32_t len)
{
  const uint32_t poly = 0x1B2B117U;
  uint32_t crc = 0U;

  for (uint32_t i = 0U; i < len; i++) {
    const uint32_t bit = bits[i] & 1U;
    const uint32_t msb = ((crc >> 23U) & 1U) ^ bit;
    crc = (crc << 1U) & 0xFFFFFFU;
    if (msb) {
      crc ^= (poly & 0xFFFFFFU);
    }
  }

  return crc & 0xFFFFFFU;
}

static uint16_t nr_dci_polar_output_length(uint16_t K, uint16_t E)
{
  const uint8_t n_max = 9U;
  const uint8_t n_min = 5U;
  const uint8_t ceil_log2_e = nr_ceil_log2_u32(E);
  uint8_t n1 = ceil_log2_e;
  uint8_t n2 = nr_ceil_log2_u32((uint32_t)K * 8U);
  uint8_t n = n_max;

  if (ceil_log2_e > 0U &&
      (8U * (uint32_t)E) <= (9U * (1U << (ceil_log2_e - 1U))) &&
      (16U * (uint32_t)K) < (9U * (uint32_t)E)) {
    n1 = (uint8_t)(ceil_log2_e - 1U);
  }

  if (n > n1) {
    n = n1;
  }
  if (n > n2) {
    n = n2;
  }
  if (n < n_min) {
    n = n_min;
  }

  return (uint16_t)(1U << n);
}

static void nr_dci_polar_rate_matching_pattern(uint16_t *rmp,
                                               uint16_t *J,
                                               uint16_t K,
                                               uint16_t N,
                                               uint16_t E)
{
  const uint8_t *P_i = nr_polar_subblock_interleaver_32();
  uint16_t y[512];

  if (!rmp || !J || !P_i || N > 512U) {
    return;
  }

  memset(y, 0, sizeof(y));
  for (uint16_t m = 0U; m < N; m++) {
    const uint16_t i = (uint16_t)((32U * (uint32_t)m) / (uint32_t)N);
    J[m] = (uint16_t)((uint32_t)P_i[i] * (N / 32U) + (m % (N / 32U)));
    y[m] = J[m];
  }

  if (E >= N) {
    for (uint16_t k = 0U; k < E; k++) {
      rmp[k] = y[k % N];
    }
  } else if ((16U * (uint32_t)K) <= (7U * (uint32_t)E)) {
    for (uint16_t k = 0U; k < E; k++) {
      rmp[k] = y[k + N - E];
    }
  } else {
    for (uint16_t k = 0U; k < E; k++) {
      rmp[k] = y[k];
    }
  }
}

static void nr_dci_polar_rate_unmatch(const float *input,
                                      float *output,
                                      const uint16_t *rmp,
                                      uint16_t K,
                                      uint16_t N,
                                      uint16_t E)
{
  if (!input || !output || !rmp) {
    return;
  }

  if (E >= N) {
    memset(output, 0, sizeof(*output) * (size_t)N);
    for (uint16_t i = 0U; i < E; i++) {
      output[rmp[i]] += input[i];
    }
  } else {
    if ((16U * (uint32_t)K) <= (7U * (uint32_t)E)) {
      memset(output, 0, sizeof(*output) * (size_t)N);
    } else {
      for (uint16_t i = 0U; i < N; i++) {
        output[i] = 32767.0f;
      }
    }
    for (uint16_t i = 0U; i < E; i++) {
      output[rmp[i]] = input[i];
    }
  }
}

static int nr_dci_build_info_set(uint16_t K,
                                 uint16_t N,
                                 uint16_t E,
                                 const uint16_t *J,
                                 uint16_t *info_pos,
                                 uint8_t *frozen)
{
  const uint16_t *Q = nr_polar_reliability_sequence_512();
  int16_t QF[513];
  int16_t QI[513];

  if (!Q || !J || !info_pos || !frozen || N != 512U || K > 164U) {
    return -1;
  }

  for (uint16_t i = 0U; i <= N; i++) {
    QF[i] = -1;
    QI[i] = -1;
  }
  memset(frozen, 1, sizeof(*frozen) * (size_t)N);

  if (E < N) {
    if ((16U * (uint32_t)K) <= (7U * (uint32_t)E)) {
      for (uint16_t n = 0U; n <= (uint16_t)(N - E - 1U); n++) {
        QF[++QF[N]] = (int16_t)J[n];
      }
      if ((4U * (uint32_t)E) >= (3U * (uint32_t)N)) {
        const uint16_t limit = (uint16_t)(((3U * N - 2U * E) + 3U) / 4U);
        for (uint16_t n = 0U; n < limit; n++) {
          QF[++QF[N]] = (int16_t)n;
        }
      } else {
        const uint16_t limit = (uint16_t)(((9U * N - 4U * E) + 15U) / 16U);
        for (uint16_t n = 0U; n < limit; n++) {
          QF[++QF[N]] = (int16_t)n;
        }
      }
    } else {
      for (uint16_t n = E; n <= (uint16_t)(N - 1U); n++) {
        QF[++QF[N]] = (int16_t)J[n];
      }
    }
  }

  for (uint16_t n = 0U; n < N; n++) {
    int m = 0;
    for (m = 0; m <= QF[N]; m++) {
      if ((int16_t)Q[n] == QF[m]) {
        break;
      }
    }
    if (m > QF[N]) {
      QI[++QI[N]] = (int16_t)Q[n];
    }
  }

  if (QI[N] + 1 < K) {
    return -1;
  }
  for (uint16_t n = 0U; n < K; n++) {
    const int ind = QI[N] + (int)n - ((int)K - 1);
    info_pos[n] = (uint16_t)QI[ind];
    frozen[info_pos[n]] = 0U;
  }
  qsort(info_pos, K, sizeof(*info_pos), nr_comp_u16);
  return 0;
}

static float nr_dci_polar_f(float a, float b)
{
  const float aa = fabsf(a);
  const float ab = fabsf(b);
  const float minab = (aa < ab) ? aa : ab;
  return ((a < 0.0f) != (b < 0.0f)) ? -minab : minab;
}

static void nr_dci_polar_sc_decode_rec(const float *alpha,
                                       uint16_t N,
                                       uint16_t offset,
                                       const uint8_t *frozen,
                                       uint8_t *uhat,
                                       uint8_t *beta)
{
  if (N == 1U) {
    const uint8_t bit = frozen[offset] ? 0U : (uint8_t)(alpha[0] <= 0.0f);
    uhat[offset] = bit;
    beta[0] = bit;
    return;
  }

  {
    const uint16_t half = (uint16_t)(N >> 1U);
    float alpha_l[half];
    float alpha_r[half];
    uint8_t beta_l[half];
    uint8_t beta_r[half];

    memset(alpha_l, 0, sizeof(alpha_l));
    memset(alpha_r, 0, sizeof(alpha_r));
    memset(beta_l, 0, sizeof(beta_l));
    memset(beta_r, 0, sizeof(beta_r));

    for (uint16_t i = 0U; i < half; i++) {
      alpha_l[i] = nr_dci_polar_f(alpha[i], alpha[i + half]);
    }
    nr_dci_polar_sc_decode_rec(alpha_l, half, offset, frozen, uhat, beta_l);

    for (uint16_t i = 0U; i < half; i++) {
      alpha_r[i] = beta_l[i]
                     ? (alpha[i + half] - alpha[i])
                     : (alpha[i + half] + alpha[i]);
    }
    nr_dci_polar_sc_decode_rec(alpha_r, half, (uint16_t)(offset + half),
                               frozen, uhat, beta_r);

    for (uint16_t i = 0U; i < half; i++) {
      beta[i] = (uint8_t)((beta_l[i] + beta_r[i]) & 1U);
      beta[i + half] = beta_r[i];
    }
  }
}

static void nr_dci_polar_input_deinterleave(const uint8_t *in,
                                            uint16_t K,
                                            uint8_t *out)
{
  const uint8_t *pi = nr_polar_input_interleaver_164();
  const uint16_t base = (uint16_t)(164U - K);
  uint16_t j = 0U;

  if (!in || !out || !pi || K > 164U) {
    return;
  }
  memset(out, 0, sizeof(*out) * (size_t)K);
  for (uint16_t i = 0U; i < 164U; i++) {
    if (pi[i] >= base) {
      out[pi[i] - base] = in[j++];
    }
  }
}

static uint32_t nr_dci_crc24c_with_ones(const uint8_t *payload_bits,
                                        uint16_t payload_len)
{
  uint8_t bits[24U + 64U];

  if (!payload_bits || payload_len > 64U) {
    return 0U;
  }
  memset(bits, 1, 24U);
  memcpy(&bits[24U], payload_bits, sizeof(*payload_bits) * (size_t)payload_len);
  return nr_dci_crc24c_bits(bits, payload_len + 24U);
}

static uint32_t nr_dci_payload_read(uint64_t payload, uint16_t *pos, uint8_t nbits)
{
  uint64_t mask = 0U;

  if (!pos || nbits == 0U || *pos < nbits) {
    return 0U;
  }
  *pos = (uint16_t)(*pos - nbits);
  mask = (nbits >= 64U) ? UINT64_MAX : ((1ULL << nbits) - 1ULL);
  return (uint32_t)((payload >> *pos) & mask);
}

static void nr_dci10_si_parse_fields(nr_pdcch_dci10_si_obs_t *dci)
{
  uint16_t pos = dci ? dci->dci_payload_bits : 0U;

  if (!dci || dci->dci_payload_bits == 0U) {
    return;
  }

  dci->frequency_domain_assignment =
      (uint16_t)nr_dci_payload_read(dci->payload, &pos, dci->freq_assignment_bits);
  dci->time_domain_assignment =
      (uint8_t)nr_dci_payload_read(dci->payload, &pos, 4U);
  dci->vrb_to_prb_mapping =
      (uint8_t)nr_dci_payload_read(dci->payload, &pos, 1U);
  dci->mcs =
      (uint8_t)nr_dci_payload_read(dci->payload, &pos, 5U);
  dci->redundancy_version =
      (uint8_t)nr_dci_payload_read(dci->payload, &pos, 2U);
  dci->system_info_indicator =
      (uint8_t)nr_dci_payload_read(dci->payload, &pos, 1U);
  if (pos > 0U) {
    dci->reserved_bits = (uint16_t)(dci->payload & ((1ULL << pos) - 1ULL));
  }
}

static int nr_dci10_si_decode_sc(const float *llr,
                                 uint16_t E,
                                 uint16_t payload_bits,
                                 uint16_t rnti,
                                 uint64_t *payload,
                                 uint32_t *crc_xor)
{
  const uint16_t K = (uint16_t)(payload_bits + 24U);
  const uint16_t N = nr_dci_polar_output_length(K, E);
  uint16_t rmp[NR_PDCCH_MAX_E_BITS];
  uint16_t J[512];
  uint16_t info_pos[164];
  uint8_t frozen[512];
  float d_tilde[512];
  uint8_t uhat[512];
  uint8_t beta[512];
  uint8_t cprime[164];
  uint8_t bits[164];
  uint32_t recv_crc = 0U;
  uint32_t calc_crc = 0U;
  uint64_t decoded_payload = 0U;

  if (!llr || !payload || !crc_xor || payload_bits == 0U ||
      payload_bits > 64U || K > 164U || E > NR_PDCCH_MAX_E_BITS ||
      N != 512U) {
    return -1;
  }

  memset(rmp, 0, sizeof(rmp));
  memset(J, 0, sizeof(J));
  memset(info_pos, 0, sizeof(info_pos));
  memset(frozen, 1, sizeof(frozen));
  memset(d_tilde, 0, sizeof(d_tilde));
  memset(uhat, 0, sizeof(uhat));
  memset(beta, 0, sizeof(beta));
  memset(cprime, 0, sizeof(cprime));
  memset(bits, 0, sizeof(bits));

  nr_dci_polar_rate_matching_pattern(rmp, J, K, N, E);
  if (nr_dci_build_info_set(K, N, E, J, info_pos, frozen) != 0) {
    return -1;
  }
  nr_dci_polar_rate_unmatch(llr, d_tilde, rmp, K, N, E);
  nr_dci_polar_sc_decode_rec(d_tilde, N, 0U, frozen, uhat, beta);

  for (uint16_t i = 0U; i < K; i++) {
    cprime[i] = uhat[info_pos[i]];
  }
  nr_dci_polar_input_deinterleave(cprime, K, bits);

  for (uint16_t i = 0U; i < payload_bits; i++) {
    decoded_payload = (decoded_payload << 1U) | (uint64_t)(bits[i] & 1U);
  }
  for (uint16_t i = 0U; i < 24U; i++) {
    recv_crc = (recv_crc << 1U) | (uint32_t)(bits[payload_bits + i] & 1U);
  }
  calc_crc = nr_dci_crc24c_with_ones(bits, payload_bits);

  *payload = decoded_payload;
  *crc_xor = (calc_crc ^ recv_crc) & 0xFFFFFFU;
  return ((*crc_xor & 0xFFFFU) == rnti && (*crc_xor >> 16U) == 0U) ? 0 : 1;
}

static int nr_cce_to_reg_interleaving(int R, int k, int n_shift, int C, int L, int N_regs)
{
  if (R == 0) {
    return k;
  }
  {
    const int c = k / R;
    const int r = k % R;
    return (r * C + c + n_shift) % (N_regs / L);
  }
}

static int nr_type0_pdcch_fill_reg_list(const nr_sib1_rx_plan_t *plan,
                                        uint8_t aggregation_level,
                                        uint16_t first_cce,
                                        uint16_t *reg_list,
                                        uint16_t max_regs)
{
  const int bsize = plan ? plan->reg_bundle_size : 0;
  const int R = plan ? plan->interleaver_size : 0;
  const int n_shift = plan ? plan->shift_index : 0;
  const int n_rb = plan ? plan->coreset_num_rbs : 0;
  const int dur = plan ? plan->coreset_num_symbols : 0;
  const int L = aggregation_level;
  const int N_regs = n_rb * dur;
  int C = 0;
  uint16_t list_idx = 0U;

  if (!plan || !reg_list || !plan->valid || L == 0 || L < 4 || bsize <= 0 || dur <= 0 || n_rb <= 0) {
    return -1;
  }
  if ((L * NR_PDCCH_REG_PER_CCE) > max_regs) {
    return -1;
  }
  if ((N_regs % NR_PDCCH_REG_PER_CCE) != 0) {
    return -1;
  }
  if ((first_cce + L) > (N_regs / NR_PDCCH_REG_PER_CCE)) {
    return -1;
  }

  if (plan->cce_reg_mapping_interleaved) {
    if ((N_regs % (bsize * R)) != 0) {
      return -1;
    }
    C = N_regs / (bsize * R);
  }

  for (uint8_t cce_idx = 0U; cce_idx < L; cce_idx++) {
    const int cce = (int)first_cce + cce_idx;
    for (uint8_t bundle_idx = 0U; bundle_idx < (NR_PDCCH_REG_PER_CCE / bsize); bundle_idx++) {
      const uint8_t k = (uint8_t)((NR_PDCCH_REG_PER_CCE * cce / bsize) + bundle_idx);
      const int f = plan->cce_reg_mapping_interleaved
                      ? nr_cce_to_reg_interleaving(R, k, n_shift, C, bsize, N_regs)
                      : k;

      for (uint8_t reg_idx = 0U; reg_idx < (bsize / dur); reg_idx++) {
        if (list_idx >= max_regs) {
          return -1;
        }
        reg_list[list_idx++] = (uint16_t)(f * bsize / dur + reg_idx);
      }
    }
  }

  qsort(reg_list, list_idx, sizeof(reg_list[0]), nr_comp_u16);
  return (int)list_idx;
}

int nr_type0_coreset0_extract_dmrs(const nr_cell_ctx_t *cell,
                                   const nr_iq_block_t *blk,
                                   const nr_coreset0_grid_t *grid,
                                   nr_coreset0_dmrs_obs_t *obs)
{
  nr_coreset0_grid_t local_grid;
  const nr_coreset0_grid_t *use_grid = grid;
  double dmrs_pow_sum = 0.0;
  double data_pow_sum = 0.0;
  double h_pow_sum = 0.0;
  double corr_r = 0.0;
  double corr_i = 0.0;
  double evm_sum = 0.0;
  uint32_t evm_count = 0U;

  if (!obs) {
    return -1;
  }
  memset(obs, 0, sizeof(*obs));

  if (!cell || !cell->valid || !cell->mib.valid || !cell->sib1_rx.valid) {
    return -1;
  }

  if (!use_grid || !use_grid->valid) {
    if (!blk || nr_type0_coreset0_extract_grid(cell, blk, &local_grid) != 0 || !local_grid.valid) {
      return -1;
    }
    use_grid = &local_grid;
  }

  obs->valid = 1U;
  obs->frame = use_grid->frame;
  obs->slot = use_grid->slot;
  obs->first_symbol_index = use_grid->first_symbol_index;
  obs->num_symbols = use_grid->num_symbols;
  obs->num_rbs = use_grid->num_rbs;
  obs->num_dmrs_re = use_grid->num_rbs * NR_PDCCH_DMRS_RE_PER_RB * use_grid->num_symbols;
  obs->num_data_re = use_grid->num_rbs * NR_PDCCH_DATA_RE_PER_RB * use_grid->num_symbols;
  obs->dmrs_scrambling_id = cell->sib1_rx.dmrs_scrambling_id;
  obs->cce_reg_mapping_interleaved = cell->sib1_rx.cce_reg_mapping_interleaved;
  obs->reg_bundle_size = cell->sib1_rx.reg_bundle_size;
  obs->interleaver_size = cell->sib1_rx.interleaver_size;
  obs->shift_index = cell->sib1_rx.shift_index;

  for (uint32_t sym = 0U; sym < use_grid->num_symbols && sym < NR_CORESET0_MAX_SYMBOLS; sym++) {
    cf32_t dmrs_ref[NR_CORESET0_MAX_RB * NR_PDCCH_DMRS_RE_PER_RB];
    const uint32_t abs_symbol = use_grid->first_symbol_index + sym;
    double h_sym_r = 0.0;
    double h_sym_i = 0.0;
    uint32_t dmrs_idx = 0U;
    uint32_t sym_dmrs_count = 0U;

    if (nr_pdcch_dmrs_ref_build(obs->dmrs_scrambling_id,
                                use_grid->slot,
                                abs_symbol,
                                use_grid->num_rbs,
                                dmrs_ref,
                                NR_CORESET0_MAX_RB * NR_PDCCH_DMRS_RE_PER_RB) < 0) {
      obs->valid = 0U;
      return -1;
    }

    for (uint32_t rb = 0U; rb < use_grid->num_rbs; rb++) {
      for (uint32_t sc = 0U; sc < 12U; sc++) {
        const uint32_t grid_sc = rb * 12U + sc;
        const cf32_t y = use_grid->re[sym][grid_sc];
        const double y_pow = (double)y.r * (double)y.r + (double)y.i * (double)y.i;

        if (nr_is_pdcch_dmrs_sc(sc)) {
          const cf32_t ref = dmrs_ref[dmrs_idx++];
          const double h_r = (double)y.r * (double)ref.r + (double)y.i * (double)ref.i;
          const double h_i = (double)y.i * (double)ref.r - (double)y.r * (double)ref.i;
          dmrs_pow_sum += y_pow;
          h_pow_sum += h_r * h_r + h_i * h_i;
          corr_r += h_r;
          corr_i += h_i;
          h_sym_r += h_r;
          h_sym_i += h_i;
          sym_dmrs_count++;
        } else {
          data_pow_sum += y_pow;
        }
      }
    }

    if (sym_dmrs_count > 0U) {
      obs->h_avg[sym].r = (float)(h_sym_r / (double)sym_dmrs_count);
      obs->h_avg[sym].i = (float)(h_sym_i / (double)sym_dmrs_count);
    }

    dmrs_idx = 0U;
    for (uint32_t rb = 0U; rb < use_grid->num_rbs; rb++) {
      for (uint32_t sc = 0U; sc < 12U; sc++) {
        const uint32_t grid_sc = rb * 12U + sc;
        if (!nr_is_pdcch_dmrs_sc(sc)) {
          continue;
        }
        {
          const cf32_t y = use_grid->re[sym][grid_sc];
          const cf32_t ref = dmrs_ref[dmrs_idx++];
          const double est_r = (double)obs->h_avg[sym].r * (double)ref.r -
                               (double)obs->h_avg[sym].i * (double)ref.i;
          const double est_i = (double)obs->h_avg[sym].r * (double)ref.i +
                               (double)obs->h_avg[sym].i * (double)ref.r;
          const double err_r = (double)y.r - est_r;
          const double err_i = (double)y.i - est_i;
          evm_sum += err_r * err_r + err_i * err_i;
          evm_count++;
        }
      }
    }
  }

  if (obs->num_dmrs_re > 0U) {
    obs->avg_dmrs_power = (float)(dmrs_pow_sum / (double)obs->num_dmrs_re);
    obs->avg_h_power = (float)(h_pow_sum / (double)obs->num_dmrs_re);
    obs->corr_mag = (float)(sqrt(corr_r * corr_r + corr_i * corr_i) / (double)obs->num_dmrs_re);
  }
  if (obs->num_data_re > 0U) {
    obs->avg_data_power = (float)(data_pow_sum / (double)obs->num_data_re);
  }
  if (evm_count > 0U) {
    obs->evm_rms = (float)sqrt(evm_sum / (double)evm_count);
  }

  return 0;
}

uint16_t nr_type0_dci10_si_payload_bits(const nr_sib1_rx_plan_t *plan,
                                        uint8_t *freq_assignment_bits)
{
  int n_rb = 0;
  uint32_t riv_states = 0U;
  uint8_t f_bits = 0U;

  if (freq_assignment_bits) {
    *freq_assignment_bits = 0U;
  }
  if (!plan || !plan->valid) {
    return 0U;
  }

  /*
   * OAI follows 38.212 Section 7.3.1.0 for DCI 1_0 size alignment:
   * for CORESET0/common search space, the frequency assignment size is
   * derived from the CORESET0 RB span, then the fixed SI-RNTI format 1_0
   * size is 28 + that field width.
   */
  n_rb = plan->coreset0_present ? plan->coreset_num_rbs : plan->bwp_size_rb;
  if (n_rb <= 0) {
    return 0U;
  }

  riv_states = (uint32_t)n_rb * (uint32_t)(n_rb + 1) / 2U;
  f_bits = nr_ceil_log2_u32(riv_states);
  if (freq_assignment_bits) {
    *freq_assignment_bits = f_bits;
  }

  return (uint16_t)(28U + f_bits);
}

int nr_type0_pdcch_build_dci10_si_softbits(const nr_cell_ctx_t *cell,
                                           const nr_pdcch_candidate_obs_t *cand,
                                           nr_pdcch_dci10_si_obs_t *dci)
{
  nr_sib1_rx_plan_t local_plan;
  const nr_sib1_rx_plan_t *plan = cell ? &cell->sib1_rx : NULL;
  uint8_t scrambling[NR_PDCCH_MAX_E_BITS];
  uint8_t freq_bits = 0U;
  uint16_t payload_bits = 0U;
  uint16_t encoded_bits = 0U;
  double sum_abs = 0.0;
  double max_abs = 0.0;
  uint16_t positives = 0U;
  uint64_t decoded_payload = 0U;
  uint32_t crc_xor = 0U;
  int decode_rc = -1;

  if (!dci) {
    return -1;
  }
  memset(dci, 0, sizeof(*dci));

  if (!cell || !cell->valid || !plan || !plan->valid || !cand || !cand->valid) {
    return -1;
  }

  memset(&local_plan, 0, sizeof(local_plan));
  if (nr_sib1_rx_plan_for_frame_slot(cell, cand->frame, cand->slot, &local_plan) == 0 &&
      local_plan.valid) {
    plan = &local_plan;
  }

  encoded_bits = (uint16_t)(cand->num_data_re * 2U);
  if (encoded_bits == 0U || encoded_bits > NR_PDCCH_MAX_E_BITS ||
      encoded_bits != (uint16_t)cand->aggregation_level * NR_PDCCH_BITS_PER_CCE) {
    return -1;
  }

  payload_bits = nr_type0_dci10_si_payload_bits(plan, &freq_bits);
  if (payload_bits == 0U) {
    return -1;
  }

  nr_pdcch_gen_prbs((((uint32_t)plan->si_rnti << 16U) + plan->dmrs_scrambling_id) % (1U << 31U),
                    scrambling,
                    encoded_bits);

  for (uint16_t re = 0U; re < cand->num_data_re; re++) {
    float llr_i = cand->eq_re[re].r;
    float llr_q = cand->eq_re[re].i;
    const uint16_t bit_i = (uint16_t)(2U * re);
    const uint16_t bit_q = (uint16_t)(2U * re + 1U);
    double abs_i = 0.0;
    double abs_q = 0.0;

    if (scrambling[bit_i]) {
      llr_i = -llr_i;
    }
    if (scrambling[bit_q]) {
      llr_q = -llr_q;
    }

    dci->llr[bit_i] = llr_i;
    dci->llr[bit_q] = llr_q;
    if (llr_i > 0.0f) {
      positives++;
    }
    if (llr_q > 0.0f) {
      positives++;
    }
    abs_i = fabs((double)llr_i);
    abs_q = fabs((double)llr_q);
    sum_abs += abs_i + abs_q;
    if (abs_i > max_abs) {
      max_abs = abs_i;
    }
    if (abs_q > max_abs) {
      max_abs = abs_q;
    }
  }

  dci->valid = 1U;
  dci->frame = cand->frame;
  dci->slot = cand->slot;
  dci->candidate_index = cand->candidate_index;
  dci->aggregation_level = cand->aggregation_level;
  dci->first_cce = cand->first_cce;
  dci->rnti = plan->si_rnti;
  dci->scrambling_id = plan->dmrs_scrambling_id;
  dci->encoded_bits = encoded_bits;
  dci->dci_payload_bits = payload_bits;
  dci->freq_assignment_bits = freq_bits;
  dci->mean_abs_llr = (float)(sum_abs / (double)encoded_bits);
  dci->max_abs_llr = (float)max_abs;
  dci->positive_fraction = (float)positives / (float)encoded_bits;

  decode_rc = nr_dci10_si_decode_sc(dci->llr,
                                    encoded_bits,
                                    payload_bits,
                                    dci->rnti,
                                    &decoded_payload,
                                    &crc_xor);
  if (decode_rc >= 0) {
    dci->decoder_ready = 1U;
    dci->crc_ok = (uint8_t)(decode_rc == 0);
    dci->crc_xor = crc_xor;
    dci->payload = decoded_payload;
    nr_dci10_si_parse_fields(dci);
  }
  return 0;
}

int nr_type0_pdcch_extract_candidate(const nr_cell_ctx_t *cell,
                                     const nr_iq_block_t *blk,
                                     const nr_coreset0_grid_t *grid,
                                     const nr_coreset0_dmrs_obs_t *dmrs,
                                     uint8_t candidate_index,
                                     nr_pdcch_candidate_obs_t *cand)
{
  nr_coreset0_grid_t local_grid;
  nr_coreset0_dmrs_obs_t local_dmrs;
  nr_sib1_rx_plan_t local_plan;
  const nr_coreset0_grid_t *use_grid = grid;
  const nr_coreset0_dmrs_obs_t *use_dmrs = dmrs;
  const nr_sib1_rx_plan_t *plan = cell ? &cell->sib1_rx : NULL;
  double data_pow_sum = 0.0;
  double eq_pow_sum = 0.0;
  uint16_t num_regs = 0U;
  uint16_t out_re = 0U;

  if (!cand) {
    return -1;
  }
  memset(cand, 0, sizeof(*cand));

  if (!cell || !cell->valid || !plan || !plan->valid) {
    return -1;
  }

  if (!use_grid || !use_grid->valid) {
    if (!blk || nr_type0_coreset0_extract_grid(cell, blk, &local_grid) != 0 || !local_grid.valid) {
      return -1;
    }
    use_grid = &local_grid;
  }

  memset(&local_plan, 0, sizeof(local_plan));
  if (nr_sib1_rx_plan_for_frame_slot(cell, use_grid->frame, use_grid->slot, &local_plan) == 0 &&
      local_plan.valid) {
    plan = &local_plan;
  }
  if (candidate_index >= plan->candidate_count) {
    return -1;
  }

  if (!use_dmrs || !use_dmrs->valid) {
    if (nr_type0_coreset0_extract_dmrs(cell, blk, use_grid, &local_dmrs) != 0 || !local_dmrs.valid) {
      return -1;
    }
    use_dmrs = &local_dmrs;
  }

  cand->candidate_index = candidate_index;
  cand->aggregation_level = plan->candidates[candidate_index].aggregation_level;
  cand->first_cce = plan->candidates[candidate_index].first_cce;
  cand->frame = use_grid->frame;
  cand->slot = use_grid->slot;
  cand->n_cce_total =
      (uint16_t)((use_grid->num_rbs * use_grid->num_symbols) / NR_PDCCH_REG_PER_CCE);

  {
    const int rc = nr_type0_pdcch_fill_reg_list(plan,
                                                cand->aggregation_level,
                                                cand->first_cce,
                                                cand->reg_list,
                                                NR_PDCCH_MAX_CAND_REG);
    if (rc <= 0) {
      return -1;
    }
    num_regs = (uint16_t)rc;
    cand->num_regs = num_regs;
  }

  for (uint32_t sym = 0U; sym < use_grid->num_symbols && sym < NR_CORESET0_MAX_SYMBOLS; sym++) {
    const cf32_t h = use_dmrs->h_avg[sym];
    const double h_pow = (double)h.r * (double)h.r + (double)h.i * (double)h.i;

    for (uint16_t reg_i = 0U; reg_i < num_regs; reg_i++) {
      const uint16_t rb = cand->reg_list[reg_i];
      if (rb >= use_grid->num_rbs) {
        return -1;
      }
      for (uint32_t sc = 0U; sc < 12U; sc++) {
        const uint32_t grid_sc = (uint32_t)rb * 12U + sc;
        const cf32_t y = use_grid->re[sym][grid_sc];
        const double y_pow = (double)y.r * (double)y.r + (double)y.i * (double)y.i;
        cf32_t eq = {0.0f, 0.0f};

        if (nr_is_pdcch_dmrs_sc(sc)) {
          continue;
        }
        if (out_re >= NR_PDCCH_MAX_CAND_DATA_RE) {
          return -1;
        }

        if (h_pow > 1.0e-9) {
          eq.r = (float)(((double)y.r * (double)h.r + (double)y.i * (double)h.i) / h_pow);
          eq.i = (float)(((double)y.i * (double)h.r - (double)y.r * (double)h.i) / h_pow);
        }

        cand->data_re[out_re] = y;
        cand->eq_re[out_re] = eq;
        data_pow_sum += y_pow;
        eq_pow_sum += (double)eq.r * (double)eq.r + (double)eq.i * (double)eq.i;
        out_re++;
      }
    }
  }

  cand->num_data_re = out_re;
  if (out_re > 0U) {
    cand->avg_data_power = (float)(data_pow_sum / (double)out_re);
    cand->avg_eq_power = (float)(eq_pow_sum / (double)out_re);
  }
  cand->valid = 1U;
  return 0;
}
