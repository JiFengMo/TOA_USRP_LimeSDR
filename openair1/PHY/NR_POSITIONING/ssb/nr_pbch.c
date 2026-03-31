#include "openair1/PHY/NR_POSITIONING/nr_pos_api.h"

#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define NR_PBCH_A 32U
#define NR_PBCH_CRC 24U
#define NR_PBCH_K (NR_PBCH_A + NR_PBCH_CRC)
#define NR_PBCH_N 512U
#define NR_PBCH_E 864U
#define NR_PBCH_QPSK_RE (NR_PBCH_E / 2U)
#define NR_PBCH_DMRS_RE 144U

static const uint8_t g_pbch_payload_g[NR_PBCH_A] = {
    16, 23, 18, 17, 8, 30, 10, 6,
    24, 7, 0, 5, 3, 2, 1, 4,
    9, 11, 12, 13, 14, 15, 19, 20,
    21, 22, 25, 26, 27, 28, 29, 31};

static const uint8_t g_polar_il_max[164] = {
    0,   2,   4,   7,   9,   14,  19,  20,  24,  25,  26,  28,  31,  34,
    42,  45,  49,  50,  51,  53,  54,  56,  58,  59,  61,  62,  65,  66,
    67,  69,  70,  71,  72,  76,  77,  81,  82,  83,  87,  88,  89,  91,
    93,  95,  98,  101, 104, 106, 108, 110, 111, 113, 115, 118, 119, 120,
    122, 123, 126, 127, 129, 132, 134, 138, 139, 140, 1,   3,   5,   8,
    10,  15,  21,  27,  29,  32,  35,  43,  46,  52,  55,  57,  60,  63,
    68,  73,  78,  84,  90,  92,  94,  96,  99,  102, 105, 107, 109, 112,
    114, 116, 121, 124, 128, 130, 133, 135, 141, 6,   11,  16,  22,  30,
    33,  36,  44,  47,  64,  74,  79,  85,  97,  100, 103, 117, 125, 131,
    136, 142, 12,  17,  23,  37,  48,  75,  80,  86,  137, 143, 13,  18,
    38,  144, 39,  145, 40,  146, 41,  147, 148, 149, 150, 151, 152, 153,
    154, 155, 156, 157, 158, 159, 160, 161, 162, 163};

static const uint8_t g_subblock_perm[32] = {
    0,  1,  2,  4,  3,  5,  6,  7,
    8,  16, 9,  17, 10, 18, 11, 19,
    12, 20, 13, 21, 14, 22, 15, 23,
    24, 25, 26, 28, 27, 29, 30, 31};

static void nr_pbch_gen_prbs(uint32_t c_init, uint8_t *c, uint32_t len)
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

static uint32_t nr_crc24c_bits(const uint8_t *bits, uint32_t len)
{
  const uint32_t poly = 0x1864CFBU;
  uint32_t crc = 0U;
  for (uint32_t i = 0; i < len; i++) {
    const uint32_t bit = bits[i] & 1U;
    const uint32_t msb = ((crc >> 23) & 1U) ^ bit;
    crc = ((crc << 1) & 0xFFFFFFU);
    if (msb) {
      crc ^= (poly & 0xFFFFFFU);
    }
  }
  return crc & 0xFFFFFFU;
}

static void nr_pbch_append_crc24c(const uint8_t *payload, uint8_t *out56)
{
  const uint32_t crc = nr_crc24c_bits(payload, NR_PBCH_A);
  memcpy(out56, payload, NR_PBCH_A * sizeof(*payload));
  for (uint32_t i = 0; i < NR_PBCH_CRC; i++) {
    out56[NR_PBCH_A + i] = (uint8_t)((crc >> (23U - i)) & 1U);
  }
}

static int nr_pbch_crc_ok(const uint8_t *bits56)
{
  uint32_t recv = 0U;
  const uint32_t calc = nr_crc24c_bits(bits56, NR_PBCH_A);
  for (uint32_t i = 0; i < NR_PBCH_CRC; i++) {
    recv = (recv << 1U) | (uint32_t)(bits56[NR_PBCH_A + i] & 1U);
  }
  return calc == recv;
}

static void nr_pbch_build_payload_bits(uint16_t sfn, uint8_t hrf,
                                       uint32_t mib_payload, uint8_t *a32)
{
  if (!a32) {
    return;
  }
  for (uint32_t i = 0; i < 24U; i++) {
    a32[i] = (uint8_t)((mib_payload >> (23U - i)) & 1U);
  }
  a32[24] = (uint8_t)((sfn >> 3) & 1U);
  a32[25] = (uint8_t)((sfn >> 2) & 1U);
  a32[26] = (uint8_t)((sfn >> 1) & 1U);
  a32[27] = (uint8_t)(sfn & 1U);
  a32[28] = (uint8_t)(hrf & 1U);
  a32[29] = 0U;
  a32[30] = 0U;
  a32[31] = 0U;
}

static void nr_pbch_payload_map(uint8_t *in_to_out32, uint8_t *scramble_mask32)
{
  uint32_t j_sfn = 0U;
  uint32_t j_hrf = 10U;
  uint32_t j_ssb = 11U;
  uint32_t j_other = 14U;

  if (!in_to_out32) {
    return;
  }
  memset(in_to_out32, 0, NR_PBCH_A * sizeof(*in_to_out32));
  if (scramble_mask32) {
    memset(scramble_mask32, 1, NR_PBCH_A * sizeof(*scramble_mask32));
  }

  for (uint32_t i = 0; i < NR_PBCH_A; i++) {
    uint32_t j = 0U;
    if (i < 6U || (i >= 24U && i <= 27U)) {
      j = j_sfn++;
    } else if (i == 28U) {
      j = j_hrf++;
    } else if (i >= 29U) {
      j = j_ssb++;
    } else {
      j = j_other++;
    }
    in_to_out32[i] = g_pbch_payload_g[j];
    if (scramble_mask32 && (i == 25U || i == 26U || i == 28U)) {
      scramble_mask32[in_to_out32[i]] = 0U;
    }
  }
}

static void nr_pbch_payload_interleave(const uint8_t *in32, uint8_t *out32,
                                       uint8_t *scramble_mask32)
{
  uint8_t map[NR_PBCH_A];

  if (!in32 || !out32) {
    return;
  }
  memset(out32, 0, NR_PBCH_A * sizeof(*out32));
  nr_pbch_payload_map(map, scramble_mask32);
  for (uint32_t i = 0; i < NR_PBCH_A; i++) {
    out32[map[i]] = in32[i];
  }
}

static void nr_pbch_payload_deinterleave(const uint8_t *in32, uint8_t *out32)
{
  uint8_t map[NR_PBCH_A];

  if (!in32 || !out32) {
    return;
  }
  nr_pbch_payload_map(map, NULL);
  for (uint32_t i = 0; i < NR_PBCH_A; i++) {
    out32[i] = in32[map[i]];
  }
}

static void nr_pbch_payload_scramble(uint16_t pci, uint16_t sfn, uint8_t *bits32)
{
  uint8_t tmp32[NR_PBCH_A];
  uint8_t mask[NR_PBCH_A];
  uint8_t c[29U * 4U];
  uint32_t j = 0U;
  const uint32_t v = (uint32_t)((sfn >> 1) & 3U);

  nr_pbch_payload_interleave(bits32, tmp32, mask);
  nr_pbch_gen_prbs((uint32_t)pci, c, (v + 1U) * 29U);
  for (uint32_t i = 0; i < NR_PBCH_A; i++) {
    if (!mask[i]) {
      continue;
    }
    tmp32[i] ^= c[v * 29U + j];
    j++;
  }
  memcpy(bits32, tmp32, sizeof(tmp32));
}

static int nr_pbch_payload_descramble(uint16_t pci, const uint8_t *scrambled32,
                                      uint8_t *plain32)
{
  uint8_t ref_plain[NR_PBCH_A];
  uint8_t ref_il[NR_PBCH_A];
  uint8_t mask[NR_PBCH_A];
  uint8_t tmp32[NR_PBCH_A];
  uint8_t tmp_orig[NR_PBCH_A];
  uint8_t c[29U * 4U];
  uint32_t j = 0U;
  uint32_t v = 0U;

  if (!scrambled32 || !plain32) {
    return -1;
  }
  memcpy(tmp32, scrambled32, sizeof(tmp32));
  memset(ref_plain, 0, sizeof(ref_plain));
  nr_pbch_payload_interleave(ref_plain, ref_il, mask);
  nr_pbch_payload_deinterleave(scrambled32, tmp_orig);
  v = ((uint32_t)tmp_orig[25] << 1U) | (uint32_t)tmp_orig[26];
  nr_pbch_gen_prbs((uint32_t)pci, c, (v + 1U) * 29U);
  for (uint32_t i = 0; i < NR_PBCH_A; i++) {
    if (!mask[i]) {
      continue;
    }
    tmp32[i] ^= c[v * 29U + j];
    j++;
  }
  nr_pbch_payload_deinterleave(tmp32, plain32);
  return 0;
}

static void nr_polar_input_interleave(const uint8_t *in, uint32_t K, uint8_t *out)
{
  uint32_t j = 0U;
  const uint32_t base = 164U - K;
  if (!in || !out || K > 164U) {
    return;
  }
  for (uint32_t i = 0; i < 164U; i++) {
    if (g_polar_il_max[i] >= base) {
      out[j++] = in[g_polar_il_max[i] - base];
    }
  }
}

static void nr_polar_input_deinterleave(const uint8_t *in, uint32_t K, uint8_t *out)
{
  uint32_t j = 0U;
  const uint32_t base = 164U - K;
  if (!in || !out || K > 164U) {
    return;
  }
  memset(out, 0, K * sizeof(*out));
  for (uint32_t i = 0; i < 164U; i++) {
    if (g_polar_il_max[i] >= base) {
      out[g_polar_il_max[i] - base] = in[j++];
    }
  }
}

static double nr_polar_pw_score(uint32_t idx, uint32_t n)
{
  const double beta = 1.189207115002721;
  double w = 0.0;
  double p = 1.0;
  uint32_t x = idx;
  for (uint32_t j = 0; j < n; j++) {
    if (x & 1U) {
      w += p;
    }
    p *= beta;
    x >>= 1U;
  }
  return w;
}

typedef struct {
  uint16_t idx;
  double score;
} nr_polar_rank_t;

typedef struct {
  uint16_t pos;
  float abs_llr;
} nr_pbch_weak_bit_t;

static int nr_polar_rank_cmp(const void *a, const void *b)
{
  const nr_polar_rank_t *ra = (const nr_polar_rank_t *)a;
  const nr_polar_rank_t *rb = (const nr_polar_rank_t *)b;
  if (ra->score < rb->score) {
    return -1;
  }
  if (ra->score > rb->score) {
    return 1;
  }
  return (int)ra->idx - (int)rb->idx;
}

static int nr_pbch_weak_bit_cmp(const void *a, const void *b)
{
  const nr_pbch_weak_bit_t *wa = (const nr_pbch_weak_bit_t *)a;
  const nr_pbch_weak_bit_t *wb = (const nr_pbch_weak_bit_t *)b;
  if (wa->abs_llr < wb->abs_llr) {
    return -1;
  }
  if (wa->abs_llr > wb->abs_llr) {
    return 1;
  }
  return (int)wa->pos - (int)wb->pos;
}

static void nr_pbch_build_info_set(uint16_t *info_pos, uint8_t *frozen)
{
  nr_polar_rank_t ranks[NR_PBCH_N];
  for (uint32_t i = 0; i < NR_PBCH_N; i++) {
    ranks[i].idx = (uint16_t)i;
    ranks[i].score = nr_polar_pw_score(i, 9U);
  }
  qsort(ranks, NR_PBCH_N, sizeof(ranks[0]), nr_polar_rank_cmp);
  if (frozen) {
    memset(frozen, 1, NR_PBCH_N * sizeof(*frozen));
  }
  for (uint32_t i = 0; i < NR_PBCH_K; i++) {
    info_pos[i] = ranks[NR_PBCH_N - NR_PBCH_K + i].idx;
    if (frozen) {
      frozen[info_pos[i]] = 0U;
    }
  }
}

static void nr_polar_encode_n(const uint8_t *u, uint32_t N, uint8_t *x)
{
  if (!u || !x) {
    return;
  }
  memcpy(x, u, N * sizeof(*x));
  for (uint32_t step = 1U; step < N; step <<= 1U) {
    for (uint32_t i = 0; i < N; i += (step << 1U)) {
      for (uint32_t j = 0; j < step; j++) {
        x[i + j] ^= x[i + step + j];
      }
    }
  }
}

static float nr_sc_f(float a, float b)
{
  const float sa = (a >= 0.0f) ? 1.0f : -1.0f;
  const float sb = (b >= 0.0f) ? 1.0f : -1.0f;
  const float aa = fabsf(a);
  const float bb = fabsf(b);
  return sa * sb * ((aa < bb) ? aa : bb);
}

static float nr_sc_g(float a, float b, uint8_t u)
{
  return b + ((u == 0U) ? a : -a);
}

static int nr_polar_should_flip(uint32_t offset, const uint16_t *flip_pos, uint32_t flip_count)
{
  for (uint32_t i = 0U; i < flip_count; i++) {
    if (flip_pos[i] == offset) {
      return 1;
    }
  }
  return 0;
}

static int nr_polar_sc_decode_rec(const float *llr, uint32_t N, uint32_t offset,
                                  const uint8_t *frozen, const uint16_t *flip_pos,
                                  uint32_t flip_count, float *leaf_llr,
                                  uint8_t *leaf_bits, uint8_t *partial_sums)
{
  if (N == 1U) {
    uint8_t bit = frozen[offset] ? 0U : (uint8_t)(llr[0] < 0.0f);
    if (!frozen[offset] && nr_polar_should_flip(offset, flip_pos, flip_count)) {
      bit ^= 1U;
    }
    if (leaf_llr) {
      leaf_llr[offset] = llr[0];
    }
    leaf_bits[offset] = bit;
    partial_sums[0] = bit;
    return 0;
  }

  const uint32_t half = N >> 1U;
  float *left = (float *)malloc(sizeof(*left) * (size_t)half);
  float *right = (float *)malloc(sizeof(*right) * (size_t)half);
  uint8_t *ps_left = (uint8_t *)malloc(sizeof(*ps_left) * (size_t)half);
  uint8_t *ps_right = (uint8_t *)malloc(sizeof(*ps_right) * (size_t)half);
  if (!left || !right || !ps_left || !ps_right) {
    free(left);
    free(right);
    free(ps_left);
    free(ps_right);
    return -1;
  }

  for (uint32_t i = 0; i < half; i++) {
    left[i] = nr_sc_f(llr[i], llr[half + i]);
  }
  if (nr_polar_sc_decode_rec(left, half, offset, frozen, flip_pos, flip_count,
                             leaf_llr, leaf_bits, ps_left) != 0) {
    free(left);
    free(right);
    free(ps_left);
    free(ps_right);
    return -1;
  }
  for (uint32_t i = 0; i < half; i++) {
    right[i] = nr_sc_g(llr[i], llr[half + i], ps_left[i]);
  }
  if (nr_polar_sc_decode_rec(right, half, offset + half, frozen, flip_pos, flip_count,
                             leaf_llr, leaf_bits, ps_right) != 0) {
    free(left);
    free(right);
    free(ps_left);
    free(ps_right);
    return -1;
  }
  for (uint32_t i = 0; i < half; i++) {
    partial_sums[i] = (uint8_t)(ps_left[i] ^ ps_right[i]);
    partial_sums[half + i] = ps_right[i];
  }

  free(left);
  free(right);
  free(ps_left);
  free(ps_right);
  return 0;
}

static int nr_pbch_extract_candidate(const uint8_t *uhat,
                                     const uint16_t *info_pos,
                                     uint8_t *payload_crc,
                                     uint8_t *payload_plain,
                                     uint16_t pci)
{
  uint8_t payload_il[NR_PBCH_K];
  if (!uhat || !info_pos || !payload_crc || !payload_plain) {
    return -1;
  }
  for (uint32_t i = 0; i < NR_PBCH_K; i++) {
    payload_il[i] = uhat[info_pos[i]];
  }
  nr_polar_input_deinterleave(payload_il, NR_PBCH_K, payload_crc);
  if (!nr_pbch_crc_ok(payload_crc)) {
    return -1;
  }
  if (nr_pbch_payload_descramble(pci, payload_crc, payload_plain) != 0) {
    return -1;
  }
  return 0;
}

static void nr_pbch_subblock_interleave(const uint8_t *in, uint8_t *out)
{
  const uint32_t blk = NR_PBCH_N / 32U;
  for (uint32_t n = 0; n < NR_PBCH_N; n++) {
    const uint32_t j = g_subblock_perm[n / blk] * blk + (n % blk);
    out[n] = in[j];
  }
}

static void nr_pbch_subblock_deinterleave_llr(const float *in, float *out)
{
  const uint32_t blk = NR_PBCH_N / 32U;
  memset(out, 0, NR_PBCH_N * sizeof(*out));
  for (uint32_t n = 0; n < NR_PBCH_N; n++) {
    const uint32_t j = g_subblock_perm[n / blk] * blk + (n % blk);
    out[j] = in[n];
  }
}

static void nr_pbch_rate_match_encode(const uint8_t *in, uint8_t *out)
{
  for (uint32_t i = 0; i < NR_PBCH_E; i++) {
    out[i] = in[i % NR_PBCH_N];
  }
}

static void nr_pbch_rate_match_decode(const float *in, float *out)
{
  memset(out, 0, NR_PBCH_N * sizeof(*out));
  for (uint32_t i = 0; i < NR_PBCH_E; i++) {
    out[i % NR_PBCH_N] += in[i];
  }
}

static void nr_pbch_codeword_scramble(uint16_t pci, uint8_t ssb_idx, uint8_t *bits)
{
  const uint32_t nu = (uint32_t)(ssb_idx & 7U);
  uint8_t *c = (uint8_t *)malloc(sizeof(*c) * (size_t)((nu + 1U) * NR_PBCH_E));
  if (!c || !bits) {
    free(c);
    return;
  }
  nr_pbch_gen_prbs((uint32_t)pci, c, (nu + 1U) * NR_PBCH_E);
  for (uint32_t i = 0; i < NR_PBCH_E; i++) {
    bits[i] ^= c[nu * NR_PBCH_E + i];
  }
  free(c);
}

static void nr_pbch_codeword_descramble_llr(uint16_t pci, uint8_t ssb_idx, float *llr)
{
  const uint32_t nu = (uint32_t)(ssb_idx & 7U);
  uint8_t *c = (uint8_t *)malloc(sizeof(*c) * (size_t)((nu + 1U) * NR_PBCH_E));
  if (!c || !llr) {
    free(c);
    return;
  }
  nr_pbch_gen_prbs((uint32_t)pci, c, (nu + 1U) * NR_PBCH_E);
  for (uint32_t i = 0; i < NR_PBCH_E; i++) {
    if (c[nu * NR_PBCH_E + i]) {
      llr[i] = -llr[i];
    }
  }
  free(c);
}

uint32_t nr_pbch_dmrs_re_positions(uint8_t v, uint16_t *rel_idx, uint8_t *sym_idx,
                                   uint32_t max_len)
{
  uint32_t pos = 0U;
  if (max_len < NR_PBCH_DMRS_RE) {
    return 0U;
  }
  for (uint32_t rel = v; rel < 240U; rel += 4U) {
    rel_idx[pos] = (uint16_t)rel;
    sym_idx[pos++] = 1U;
    if (rel < 48U || rel >= 192U) {
      rel_idx[pos] = (uint16_t)rel;
      sym_idx[pos++] = 2U;
    }
    rel_idx[pos] = (uint16_t)rel;
    sym_idx[pos++] = 3U;
  }
  return pos;
}

uint32_t nr_pbch_data_re_positions(uint8_t v, uint16_t *rel_idx, uint8_t *sym_idx,
                                   uint32_t max_len)
{
  uint32_t pos = 0U;
  if (max_len < NR_PBCH_QPSK_RE) {
    return 0U;
  }
  for (uint32_t rel = 0U; rel < 240U; rel++) {
    const uint8_t dmrs = (uint8_t)((rel & 3U) == v);
    if (!dmrs) {
      rel_idx[pos] = (uint16_t)rel;
      sym_idx[pos++] = 1U;
    }
    if ((rel < 48U || rel >= 192U) && !dmrs) {
      rel_idx[pos] = (uint16_t)rel;
      sym_idx[pos++] = 2U;
    }
    if (!dmrs) {
      rel_idx[pos] = (uint16_t)rel;
      sym_idx[pos++] = 3U;
    }
  }
  return pos;
}

int nr_pbch_bch_encode(uint16_t pci, uint8_t ssb_idx, uint16_t sfn, uint8_t hrf,
                       uint32_t mib_payload, uint8_t *coded_bits, uint32_t max_bits)
{
  uint8_t payload_plain[NR_PBCH_A];
  uint8_t payload_scr[NR_PBCH_A];
  uint8_t payload_crc[NR_PBCH_K];
  uint8_t payload_il[NR_PBCH_K];
  uint16_t info_pos[NR_PBCH_K];
  uint8_t frozen[NR_PBCH_N];
  uint8_t u[NR_PBCH_N];
  uint8_t d[NR_PBCH_N];
  uint8_t y[NR_PBCH_N];

  if (!coded_bits || max_bits < NR_PBCH_E) {
    return -1;
  }

  nr_pbch_build_payload_bits(sfn, hrf, mib_payload, payload_plain);
  memcpy(payload_scr, payload_plain, sizeof(payload_scr));
  nr_pbch_payload_scramble(pci, sfn, payload_scr);
  nr_pbch_append_crc24c(payload_scr, payload_crc);
  nr_polar_input_interleave(payload_crc, NR_PBCH_K, payload_il);

  nr_pbch_build_info_set(info_pos, frozen);
  memset(u, 0, sizeof(u));
  for (uint32_t i = 0; i < NR_PBCH_K; i++) {
    u[info_pos[i]] = payload_il[i];
  }
  nr_polar_encode_n(u, NR_PBCH_N, d);
  nr_pbch_subblock_interleave(d, y);
  nr_pbch_rate_match_encode(y, coded_bits);
  nr_pbch_codeword_scramble(pci, ssb_idx, coded_bits);
  return (int)NR_PBCH_E;
}

int nr_pbch_bch_decode(const float *llr, uint32_t llr_len, nr_sync_state_t *sync)
{
  enum { NR_PBCH_FLIP_TRIES = 6 };
  float llr_scr[NR_PBCH_E];
  float llr_rm[NR_PBCH_N];
  float llr_sub[NR_PBCH_N];
  float leaf_llr[NR_PBCH_N];
  uint8_t uhat[NR_PBCH_N];
  uint8_t partial[NR_PBCH_N];
  uint16_t info_pos[NR_PBCH_K];
  uint16_t flip_pos[2];
  uint8_t frozen[NR_PBCH_N];
  uint8_t payload_crc[NR_PBCH_K];
  uint8_t payload_plain[NR_PBCH_A];
  nr_pbch_weak_bit_t weak_bits[NR_PBCH_K];
  uint16_t sfn = 0U;
  uint32_t mib = 0U;

  if (!llr || llr_len < NR_PBCH_E || !sync) {
    return -1;
  }

  memcpy(llr_scr, llr, sizeof(llr_scr));
  nr_pbch_codeword_descramble_llr(sync->pci, sync->ssb_index, llr_scr);
  nr_pbch_rate_match_decode(llr_scr, llr_rm);
  nr_pbch_subblock_deinterleave_llr(llr_rm, llr_sub);

  nr_pbch_build_info_set(info_pos, frozen);
  memset(uhat, 0, sizeof(uhat));
  memset(leaf_llr, 0, sizeof(leaf_llr));
  if (nr_polar_sc_decode_rec(llr_sub, NR_PBCH_N, 0U, frozen, NULL, 0U,
                             leaf_llr, uhat, partial) != 0) {
    return -1;
  }
  if (nr_pbch_extract_candidate(uhat, info_pos, payload_crc, payload_plain, sync->pci) != 0) {
    uint32_t weak_count = 0U;
    for (uint32_t i = 0; i < NR_PBCH_K; i++) {
      weak_bits[i].pos = info_pos[i];
      weak_bits[i].abs_llr = fabsf(leaf_llr[info_pos[i]]);
    }
    qsort(weak_bits, NR_PBCH_K, sizeof(weak_bits[0]), nr_pbch_weak_bit_cmp);
    weak_count = (NR_PBCH_K < NR_PBCH_FLIP_TRIES) ? NR_PBCH_K : NR_PBCH_FLIP_TRIES;

    for (uint32_t i = 0U; i < weak_count; i++) {
      memset(uhat, 0, sizeof(uhat));
      flip_pos[0] = weak_bits[i].pos;
      if (nr_polar_sc_decode_rec(llr_sub, NR_PBCH_N, 0U, frozen, flip_pos, 1U,
                                 NULL, uhat, partial) == 0 &&
          nr_pbch_extract_candidate(uhat, info_pos, payload_crc, payload_plain, sync->pci) == 0) {
        goto pbch_crc_ok;
      }
    }

    for (uint32_t i = 0U; i < weak_count; i++) {
      for (uint32_t j = i + 1U; j < weak_count; j++) {
        memset(uhat, 0, sizeof(uhat));
        flip_pos[0] = weak_bits[i].pos;
        flip_pos[1] = weak_bits[j].pos;
        if (nr_polar_sc_decode_rec(llr_sub, NR_PBCH_N, 0U, frozen, flip_pos, 2U,
                                   NULL, uhat, partial) == 0 &&
            nr_pbch_extract_candidate(uhat, info_pos, payload_crc, payload_plain, sync->pci) == 0) {
          goto pbch_crc_ok;
        }
      }
    }
    return -1;
  }

pbch_crc_ok:

  for (uint32_t i = 0; i < 24U; i++) {
    mib = (mib << 1U) | (uint32_t)payload_plain[i];
  }
  for (uint32_t i = 0; i < 6U; i++) {
    sfn = (uint16_t)((sfn << 1U) | payload_plain[i]);
  }
  for (uint32_t i = 24U; i < 28U; i++) {
    sfn = (uint16_t)((sfn << 1U) | payload_plain[i]);
  }

  sync->sfn = sfn;
  sync->slot = payload_plain[28] ? 10U : 0U;
  sync->mib_payload = mib;
  sync->mib_ok = 1U;
  return 0;
}
