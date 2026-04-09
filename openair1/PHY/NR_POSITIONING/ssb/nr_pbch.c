#include "openair1/PHY/NR_POSITIONING/nr_pos_api.h"

#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#define NR_PBCH_A 32U
#define NR_PBCH_CRC 24U
#define NR_PBCH_K (NR_PBCH_A + NR_PBCH_CRC)
#define NR_PBCH_N 512U
#define NR_PBCH_E 864U
#define NR_PBCH_QPSK_RE (NR_PBCH_E / 2U)
#define NR_PBCH_DMRS_RE 144U
#define NR_PBCH_SCL_LIST 8U
#define NR_PBCH_MAX_PATHS (2U * NR_PBCH_SCL_LIST)
#define NR_PBCH_POLAR_LEVELS 10U

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
  const uint32_t poly = 0x1B2B117U;
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

/* 3GPP TS 38.212 Table 5.3.1.2-1: Reliability sequence Q_0^{N-1} for N=512.
 * Channels are ordered from least reliable to most reliable. */
static const uint16_t g_Q_0_Nminus1_512[512] = {
    0, 1, 2, 4, 8, 16, 32, 3, 5, 64, 9, 6, 17, 10, 18, 128, 12, 33, 65, 20, 256, 34, 24, 36, 7,
    129, 66, 11, 40, 68, 130, 19, 13, 48, 14, 72, 257, 21, 132, 35, 258, 26, 80, 37, 25, 22, 136, 260, 264, 38,
    96, 67, 41, 144, 28, 69, 42, 49, 74, 272, 160, 288, 192, 70, 44, 131, 81, 50, 73, 15, 320, 133, 52, 23, 134,
    384, 76, 137, 82, 56, 27, 97, 39, 259, 84, 138, 145, 261, 29, 43, 98, 88, 140, 30, 146, 71, 262, 265, 161, 45,
    100, 51, 148, 46, 75, 266, 273, 104, 162, 53, 193, 152, 77, 164, 268, 274, 54, 83, 57, 112, 135, 78, 289, 194, 85,
    276, 58, 168, 139, 99, 86, 60, 280, 89, 290, 196, 141, 101, 147, 176, 142, 321, 31, 200, 90, 292, 322, 263, 149, 102,
    105, 304, 296, 163, 92, 47, 267, 385, 324, 208, 386, 150, 153, 165, 106, 55, 328, 113, 154, 79, 269, 108, 224, 166, 195,
    270, 275, 291, 59, 169, 114, 277, 156, 87, 197, 116, 170, 61, 281, 278, 177, 293, 388, 91, 198, 172, 120, 201, 336, 62,
    282, 143, 103, 178, 294, 93, 202, 323, 392, 297, 107, 180, 151, 209, 284, 94, 204, 298, 400, 352, 325, 155, 210, 305, 300,
    109, 184, 115, 167, 225, 326, 306, 157, 329, 110, 117, 212, 171, 330, 226, 387, 308, 216, 416, 271, 279, 158, 337, 118, 332,
    389, 173, 121, 199, 179, 228, 338, 312, 390, 174, 393, 283, 122, 448, 353, 203, 63, 340, 394, 181, 295, 285, 232, 124, 205,
    182, 286, 299, 354, 211, 401, 185, 396, 344, 240, 206, 95, 327, 402, 356, 307, 301, 417, 213, 186, 404, 227, 418, 302, 360,
    111, 331, 214, 309, 188, 449, 217, 408, 229, 159, 420, 310, 333, 119, 339, 218, 368, 230, 391, 313, 450, 334, 233, 175, 123,
    341, 220, 314, 424, 395, 355, 287, 183, 234, 125, 342, 316, 241, 345, 452, 397, 403, 207, 432, 357, 187, 236, 126, 242, 398,
    346, 456, 358, 405, 303, 244, 189, 361, 215, 348, 419, 406, 464, 362, 409, 219, 311, 421, 410, 231, 248, 369, 190, 364, 335,
    480, 315, 221, 370, 422, 425, 451, 235, 412, 343, 372, 317, 222, 426, 453, 237, 433, 347, 243, 454, 318, 376, 428, 238, 359,
    457, 399, 434, 349, 245, 458, 363, 127, 191, 407, 436, 465, 246, 350, 460, 249, 411, 365, 440, 374, 423, 466, 250, 371, 481,
    413, 366, 468, 429, 252, 373, 482, 427, 414, 223, 472, 455, 377, 435, 319, 484, 430, 488, 239, 378, 459, 437, 380, 461, 496,
    351, 467, 438, 251, 462, 442, 441, 469, 247, 367, 253, 375, 444, 470, 483, 415, 485, 473, 474, 254, 379, 431, 489, 486, 476,
    439, 490, 463, 381, 497, 492, 443, 382, 498, 445, 471, 500, 446, 475, 487, 504, 255, 477, 491, 478, 383, 493, 499, 502, 494,
    501, 447, 505, 506, 479, 508, 495, 503, 507, 509, 510, 511};

static int nr_pbch_u16_cmp(const void *a, const void *b)
{
  const uint16_t va = *(const uint16_t *)a;
  const uint16_t vb = *(const uint16_t *)b;
  return (va > vb) - (va < vb);
}

static int nr_pbch_extract_candidate(const uint8_t *uhat,
                                     const uint16_t *info_pos,
                                     uint8_t *payload_crc,
                                     uint8_t *payload_plain,
                                     uint16_t pci);

static void nr_pbch_build_info_set(uint16_t *info_pos, uint8_t *frozen)
{
  uint16_t tmp[NR_PBCH_K];

  if (frozen) {
    memset(frozen, 1, NR_PBCH_N * sizeof(*frozen));
  }
  /* The last K entries of Q_0 are the selected info channels. OAI then sorts
   * Q_I_N in ascending bit-index order before mapping C' into u. */
  for (uint32_t i = 0; i < NR_PBCH_K; i++) {
    tmp[i] = g_Q_0_Nminus1_512[NR_PBCH_N - NR_PBCH_K + i];
    if (frozen) {
      frozen[tmp[i]] = 0U;
    }
  }
  if (info_pos) {
    memcpy(info_pos, tmp, sizeof(tmp));
    qsort(info_pos, NR_PBCH_K, sizeof(*info_pos), nr_pbch_u16_cmp);
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

static int16_t nr_pbch_sat_add16(int16_t a, int16_t b)
{
  const int32_t s = (int32_t)a + (int32_t)b;
  if (s > 32767) {
    return 32767;
  }
  if (s < -32767) {
    return -32767;
  }
  return (int16_t)s;
}

static int16_t nr_pbch_sat_sub16(int16_t a, int16_t b)
{
  const int32_t s = (int32_t)a - (int32_t)b;
  if (s > 32767) {
    return 32767;
  }
  if (s < -32767) {
    return -32767;
  }
  return (int16_t)s;
}

static int16_t nr_pbch_oai_f_int16(int16_t a, int16_t b)
{
  const int16_t absa = (int16_t)abs((int)a);
  const int16_t absb = (int16_t)abs((int)b);
  const int16_t minab = (absa < absb) ? absa : absb;
  return ((a < 0) ^ (b < 0)) ? (int16_t)-minab : minab;
}

static void nr_pbch_quantize_llr_int16(const float *in, int16_t *out, uint32_t len)
{
  if (!in || !out) {
    return;
  }
  for (uint32_t i = 0U; i < len; i++) {
    int32_t q = (int32_t)lroundf(in[i]);
    if (q > 32) {
      q = 32;
    } else if (q < -32) {
      q = -32;
    }
    out[i] = (int16_t)q;
  }
}

typedef struct {
  uint16_t first_leaf_index;
  uint16_t nv;
  int16_t left;
  int16_t right;
  uint8_t level;
  uint8_t leaf;
  uint8_t all_frozen;
  uint8_t beta_init;
} nr_pbch_decoder_node_t;

typedef struct {
  uint8_t op_code;
  uint16_t node_idx;
} nr_pbch_decoder_op_t;

typedef struct {
  uint8_t initialized;
  uint16_t root_idx;
  uint16_t node_count;
  uint16_t op_count;
  nr_pbch_decoder_node_t nodes[2U * NR_PBCH_N];
  nr_pbch_decoder_op_t ops[3U * NR_PBCH_N];
  int16_t alpha[2U * NR_PBCH_N][NR_PBCH_N];
  uint8_t beta[2U * NR_PBCH_N][NR_PBCH_N];
} nr_pbch_decoder_ctx_t;

enum {
  NR_PBCH_OP_LEFT = 0,
  NR_PBCH_OP_RIGHT = 1,
  NR_PBCH_OP_BETA = 2,
};

static uint8_t nr_pbch_all_frozen_range(const uint8_t *frozen,
                                        uint16_t first_leaf_index,
                                        uint16_t nv)
{
  if (!frozen) {
    return 0U;
  }
  for (uint16_t i = 0U; i < nv; i++) {
    if (!frozen[first_leaf_index + i]) {
      return 0U;
    }
  }
  return 1U;
}

static int16_t nr_pbch_build_decoder_tree(nr_pbch_decoder_ctx_t *ctx,
                                          uint8_t level,
                                          uint16_t first_leaf_index,
                                          const uint8_t *frozen)
{
  nr_pbch_decoder_node_t *node;
  const uint16_t nv = (uint16_t)(1U << level);
  const uint16_t idx = ctx->node_count++;
  const uint8_t all_frozen = nr_pbch_all_frozen_range(frozen, first_leaf_index, nv);

  node = &ctx->nodes[idx];
  memset(node, 0, sizeof(*node));
  node->first_leaf_index = first_leaf_index;
  node->nv = nv;
  node->level = level;
  node->leaf = (uint8_t)((level == 0U) || all_frozen);
  node->all_frozen = all_frozen;
  node->left = -1;
  node->right = -1;

  if (!node->leaf) {
    const uint16_t half = (uint16_t)(nv >> 1U);
    node->left = nr_pbch_build_decoder_tree(ctx, (uint8_t)(level - 1U), first_leaf_index, frozen);
    node->right = nr_pbch_build_decoder_tree(ctx, (uint8_t)(level - 1U),
                                             (uint16_t)(first_leaf_index + half), frozen);
  }
  return (int16_t)idx;
}

static void nr_pbch_linearize_decoder_tree(nr_pbch_decoder_ctx_t *ctx, uint16_t node_idx)
{
  const nr_pbch_decoder_node_t *node = &ctx->nodes[node_idx];
  if (node->leaf) {
    return;
  }

  ctx->ops[ctx->op_count++] = (nr_pbch_decoder_op_t){ .op_code = NR_PBCH_OP_LEFT, .node_idx = node_idx };
  nr_pbch_linearize_decoder_tree(ctx, (uint16_t)node->left);
  ctx->ops[ctx->op_count++] = (nr_pbch_decoder_op_t){ .op_code = NR_PBCH_OP_RIGHT, .node_idx = node_idx };
  nr_pbch_linearize_decoder_tree(ctx, (uint16_t)node->right);
  ctx->ops[ctx->op_count++] = (nr_pbch_decoder_op_t){ .op_code = NR_PBCH_OP_BETA, .node_idx = node_idx };
}

static void nr_pbch_init_generic_decoder(nr_pbch_decoder_ctx_t *ctx, const uint8_t *frozen)
{
  memset(ctx, 0, sizeof(*ctx));
  ctx->root_idx = (uint16_t)nr_pbch_build_decoder_tree(ctx, 9U, 0U, frozen);
  nr_pbch_linearize_decoder_tree(ctx, ctx->root_idx);
  ctx->initialized = 1U;
}

static void nr_pbch_apply_f_to_left(nr_pbch_decoder_ctx_t *ctx,
                                    uint16_t node_idx,
                                    uint8_t *output_bits)
{
  nr_pbch_decoder_node_t *node = &ctx->nodes[node_idx];
  nr_pbch_decoder_node_t *left = &ctx->nodes[(uint16_t)node->left];
  int16_t *alpha_v = ctx->alpha[node_idx];
  int16_t *alpha_l = ctx->alpha[(uint16_t)node->left];

  if (!left->all_frozen) {
    for (uint16_t i = 0U; i < left->nv; i++) {
      alpha_l[i] = nr_pbch_oai_f_int16(alpha_v[i], alpha_v[i + left->nv]);
    }

    if (left->leaf) {
      const uint8_t bit = (uint8_t)(alpha_l[0] <= 0);
      ctx->beta[(uint16_t)node->left][0] = bit;
      left->beta_init = 1U;
      output_bits[left->first_leaf_index] = bit;
    }
  }
}

static void nr_pbch_apply_g_to_right(nr_pbch_decoder_ctx_t *ctx,
                                     uint16_t node_idx,
                                     uint8_t *output_bits)
{
  nr_pbch_decoder_node_t *node = &ctx->nodes[node_idx];
  nr_pbch_decoder_node_t *left = &ctx->nodes[(uint16_t)node->left];
  nr_pbch_decoder_node_t *right = &ctx->nodes[(uint16_t)node->right];
  int16_t *alpha_v = ctx->alpha[node_idx];
  int16_t *alpha_r = ctx->alpha[(uint16_t)node->right];
  uint8_t *beta_l = ctx->beta[(uint16_t)node->left];

  if (!right->all_frozen) {
    for (uint16_t i = 0U; i < right->nv; i++) {
      alpha_r[i] = left->beta_init
        ? (beta_l[i]
            ? nr_pbch_sat_sub16(alpha_v[i + right->nv], alpha_v[i])
            : nr_pbch_sat_add16(alpha_v[i + right->nv], alpha_v[i]))
        : nr_pbch_sat_add16(alpha_v[i + right->nv], alpha_v[i]);
    }

    if (right->leaf) {
      const uint8_t bit = (uint8_t)(alpha_r[0] <= 0);
      ctx->beta[(uint16_t)node->right][0] = bit;
      right->beta_init = 1U;
      output_bits[right->first_leaf_index] = bit;
    }
  }
}

static void nr_pbch_compute_beta(nr_pbch_decoder_ctx_t *ctx, uint16_t node_idx)
{
  nr_pbch_decoder_node_t *node = &ctx->nodes[node_idx];
  nr_pbch_decoder_node_t *left = &ctx->nodes[(uint16_t)node->left];
  nr_pbch_decoder_node_t *right = &ctx->nodes[(uint16_t)node->right];
  uint8_t *beta_v = ctx->beta[node_idx];
  uint8_t *beta_l = ctx->beta[(uint16_t)node->left];
  uint8_t *beta_r = ctx->beta[(uint16_t)node->right];

  if (left->all_frozen) {
    for (uint16_t i = 0U; i < right->nv; i++) {
      beta_v[i] = beta_r[i];
      beta_v[i + right->nv] = beta_r[i];
    }
  } else {
    for (uint16_t i = 0U; i < right->nv; i++) {
      beta_v[i] = (uint8_t)((beta_l[i] + beta_r[i]) & 1U);
      beta_v[i + right->nv] = beta_r[i];
    }
  }
  node->beta_init = 1U;
}

static void nr_pbch_extract_u_from_tree_output(const nr_pbch_decoder_ctx_t *ctx,
                                               uint8_t *uhat)
{
  if (!ctx || !uhat) {
    return;
  }
  memset(uhat, 0, NR_PBCH_N * sizeof(*uhat));
  for (uint16_t i = 0U; i < ctx->node_count; i++) {
    const nr_pbch_decoder_node_t *node = &ctx->nodes[i];
    if (!node->leaf || node->all_frozen) {
      continue;
    }
    uhat[node->first_leaf_index] = ctx->beta[i][0];
  }
}

static int nr_pbch_oai_generic_decoder_int16(const float *llr_sub,
                                             const uint8_t *frozen,
                                             const uint16_t *info_pos,
                                             uint8_t *payload_crc,
                                             uint8_t *payload_plain,
                                             uint16_t pci)
{
  static __thread nr_pbch_decoder_ctx_t ctx;
  uint8_t uhat[NR_PBCH_N];

  if (!llr_sub || !frozen || !info_pos || !payload_crc || !payload_plain) {
    return -1;
  }

  if (!ctx.initialized) {
    nr_pbch_init_generic_decoder(&ctx, frozen);
  }

  nr_pbch_quantize_llr_int16(llr_sub, ctx.alpha[ctx.root_idx], NR_PBCH_N);
  for (uint16_t i = 0U; i < ctx.node_count; i++) {
    ctx.nodes[i].beta_init = 0U;
    memset(ctx.beta[i], 0, NR_PBCH_N * sizeof(ctx.beta[i][0]));
  }

  for (uint16_t i = 0U; i < ctx.op_count; i++) {
    const nr_pbch_decoder_op_t *op = &ctx.ops[i];
    switch (op->op_code) {
      case NR_PBCH_OP_LEFT:
        nr_pbch_apply_f_to_left(&ctx, op->node_idx, uhat);
        break;
      case NR_PBCH_OP_RIGHT:
        nr_pbch_apply_g_to_right(&ctx, op->node_idx, uhat);
        break;
      case NR_PBCH_OP_BETA:
        nr_pbch_compute_beta(&ctx, op->node_idx);
        break;
      default:
        return -1;
    }
  }

  nr_pbch_extract_u_from_tree_output(&ctx, uhat);
  return nr_pbch_extract_candidate(uhat, info_pos, payload_crc, payload_plain, pci);
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

typedef struct {
  double metric;
  uint8_t parent;
  uint8_t bit;
  uint8_t crc_state;
  uint8_t crc_checksum[NR_PBCH_CRC];
} nr_pbch_oai_candidate_t;

static int nr_pbch_oai_candidate_cmp(const void *a, const void *b)
{
  const nr_pbch_oai_candidate_t *ca = (const nr_pbch_oai_candidate_t *)a;
  const nr_pbch_oai_candidate_t *cb = (const nr_pbch_oai_candidate_t *)b;
  if (ca->metric < cb->metric) {
    return -1;
  }
  if (ca->metric > cb->metric) {
    return 1;
  }
  return (int)ca->parent - (int)cb->parent;
}

static double nr_pbch_softplus(double x)
{
  if (x > 40.0) {
    return x;
  }
  if (x < -40.0) {
    return exp(x);
  }
  return log1p(exp(x));
}

static double nr_pbch_oai_compute_llr(double a, double b)
{
  const double s = ((a >= 0.0) == (b >= 0.0)) ? 1.0 : -1.0;
  const double x = fabs(a + b);
  const double y = fabs(a - b);
  const double minab = (fabs(a) < fabs(b)) ? fabs(a) : fabs(b);
  return s * (minab + log1p(exp(-x)) - log1p(exp(-y)));
}

static void nr_pbch_oai_update_bit(uint8_t list_size,
                                   uint16_t row,
                                   uint16_t col,
                                   uint8_t bit[NR_PBCH_N][NR_PBCH_POLAR_LEVELS][NR_PBCH_MAX_PATHS],
                                   uint8_t bit_updated[NR_PBCH_N][NR_PBCH_POLAR_LEVELS])
{
  const uint32_t offset = NR_PBCH_N / (1U << (NR_PBCH_POLAR_LEVELS - col));

  for (uint32_t p = 0U; p < list_size; p++) {
    if ((row % (2U * offset)) >= offset) {
      if (!bit_updated[row][col - 1U]) {
        nr_pbch_oai_update_bit(list_size, row, (uint16_t)(col - 1U), bit, bit_updated);
      }
      bit[row][col][p] = bit[row][col - 1U][p];
    } else {
      if (!bit_updated[row][col - 1U]) {
        nr_pbch_oai_update_bit(list_size, row, (uint16_t)(col - 1U), bit, bit_updated);
      }
      if (!bit_updated[row + offset][col - 1U]) {
        nr_pbch_oai_update_bit(list_size, (uint16_t)(row + offset), (uint16_t)(col - 1U),
                               bit, bit_updated);
      }
      bit[row][col][p] =
          (uint8_t)((bit[row][col - 1U][p] + bit[row + offset][col - 1U][p]) & 1U);
    }
  }

  bit_updated[row][col] = 1U;
}

static void nr_pbch_oai_update_llr(uint8_t list_size,
                                   uint16_t row,
                                   uint16_t col,
                                   double llr[NR_PBCH_N][NR_PBCH_POLAR_LEVELS][NR_PBCH_MAX_PATHS],
                                   uint8_t llr_updated[NR_PBCH_N][NR_PBCH_POLAR_LEVELS],
                                   uint8_t bit[NR_PBCH_N][NR_PBCH_POLAR_LEVELS][NR_PBCH_MAX_PATHS],
                                   uint8_t bit_updated[NR_PBCH_N][NR_PBCH_POLAR_LEVELS])
{
  const uint32_t offset = NR_PBCH_N / (1U << (NR_PBCH_POLAR_LEVELS - col - 1U));

  for (uint32_t p = 0U; p < list_size; p++) {
    if ((row % (2U * offset)) >= offset) {
      if (!bit_updated[row - offset][col]) {
        nr_pbch_oai_update_bit(list_size, (uint16_t)(row - offset), col, bit, bit_updated);
      }
      if (!llr_updated[row - offset][col + 1U]) {
        nr_pbch_oai_update_llr(list_size, (uint16_t)(row - offset), (uint16_t)(col + 1U),
                               llr, llr_updated, bit, bit_updated);
      }
      if (!llr_updated[row][col + 1U]) {
        nr_pbch_oai_update_llr(list_size, row, (uint16_t)(col + 1U),
                               llr, llr_updated, bit, bit_updated);
      }
      llr[row][col][p] =
          (((bit[row - offset][col][p] != 0U) ? -1.0 : 1.0) * llr[row - offset][col + 1U][p]) +
          llr[row][col + 1U][p];
    } else {
      if (!llr_updated[row][col + 1U]) {
        nr_pbch_oai_update_llr(list_size, row, (uint16_t)(col + 1U),
                               llr, llr_updated, bit, bit_updated);
      }
      if (!llr_updated[row + offset][col + 1U]) {
        nr_pbch_oai_update_llr(list_size, (uint16_t)(row + offset), (uint16_t)(col + 1U),
                               llr, llr_updated, bit, bit_updated);
      }
      llr[row][col][p] =
          nr_pbch_oai_compute_llr(llr[row][col + 1U][p], llr[row + offset][col + 1U][p]);
    }
  }

  llr_updated[row][col] = 1U;
}

static void nr_pbch_oai_update_path_metric(double *path_metric,
                                           uint8_t list_size,
                                           uint8_t bit_value,
                                           uint16_t row,
                                           double llr[NR_PBCH_N][NR_PBCH_POLAR_LEVELS][NR_PBCH_MAX_PATHS])
{
  const double sign = bit_value ? 1.0 : -1.0;
  for (uint32_t p = 0U; p < list_size; p++) {
    path_metric[p] += nr_pbch_softplus(sign * llr[row][0][p]);
  }
}

static void nr_pbch_oai_copy_path_double(double dst[NR_PBCH_N][NR_PBCH_POLAR_LEVELS][NR_PBCH_MAX_PATHS],
                                         uint8_t dst_path,
                                         double src[NR_PBCH_N][NR_PBCH_POLAR_LEVELS][NR_PBCH_MAX_PATHS],
                                         uint8_t src_path)
{
  for (uint32_t n = 0U; n < NR_PBCH_N; n++) {
    for (uint32_t c = 0U; c < NR_PBCH_POLAR_LEVELS; c++) {
      dst[n][c][dst_path] = src[n][c][src_path];
    }
  }
}

static void nr_pbch_oai_copy_path_u8(uint8_t dst[NR_PBCH_N][NR_PBCH_POLAR_LEVELS][NR_PBCH_MAX_PATHS],
                                     uint8_t dst_path,
                                     uint8_t src[NR_PBCH_N][NR_PBCH_POLAR_LEVELS][NR_PBCH_MAX_PATHS],
                                     uint8_t src_path)
{
  for (uint32_t n = 0U; n < NR_PBCH_N; n++) {
    for (uint32_t c = 0U; c < NR_PBCH_POLAR_LEVELS; c++) {
      dst[n][c][dst_path] = src[n][c][src_path];
    }
  }
}

static __attribute__((unused))
int nr_pbch_oai_ca_scl_decode(const float *llr_sub,
                              const uint8_t *frozen,
                              const uint16_t *info_pos,
                              uint8_t *payload_crc,
                              uint8_t *payload_plain,
                              uint16_t pci)
{
  double llr[NR_PBCH_N][NR_PBCH_POLAR_LEVELS][NR_PBCH_MAX_PATHS];
  double old_llr[NR_PBCH_N][NR_PBCH_POLAR_LEVELS][NR_PBCH_MAX_PATHS];
  uint8_t bit[NR_PBCH_N][NR_PBCH_POLAR_LEVELS][NR_PBCH_MAX_PATHS];
  uint8_t old_bit[NR_PBCH_N][NR_PBCH_POLAR_LEVELS][NR_PBCH_MAX_PATHS];
  uint8_t bit_updated[NR_PBCH_N][NR_PBCH_POLAR_LEVELS];
  uint8_t llr_updated[NR_PBCH_N][NR_PBCH_POLAR_LEVELS];
  uint8_t crc_checksum[NR_PBCH_CRC][NR_PBCH_SCL_LIST];
  uint8_t old_crc_checksum[NR_PBCH_CRC][NR_PBCH_SCL_LIST];
  uint8_t crc_state[NR_PBCH_SCL_LIST];
  uint8_t old_crc_state[NR_PBCH_SCL_LIST];
  uint8_t info_pattern[NR_PBCH_N];
  uint8_t uhat[NR_PBCH_N];
  nr_pbch_oai_candidate_t cand[NR_PBCH_MAX_PATHS];
  double path_metric[NR_PBCH_SCL_LIST];
  double old_path_metric[NR_PBCH_SCL_LIST];
  uint32_t non_frozen_bit = 0U;
  uint8_t current_list_size = 1U;

  if (!llr_sub || !frozen || !info_pos || !payload_crc || !payload_plain) {
    return -1;
  }

  memset(llr, 0, sizeof(llr));
  memset(old_llr, 0, sizeof(old_llr));
  memset(bit, 0, sizeof(bit));
  memset(old_bit, 0, sizeof(old_bit));
  memset(bit_updated, 0, sizeof(bit_updated));
  memset(llr_updated, 0, sizeof(llr_updated));
  memset(crc_checksum, 0, sizeof(crc_checksum));
  memset(old_crc_checksum, 0, sizeof(old_crc_checksum));
  memset(path_metric, 0, sizeof(path_metric));
  memset(old_path_metric, 0, sizeof(old_path_metric));
  memset(info_pattern, 0, sizeof(info_pattern));
  memset(uhat, 0, sizeof(uhat));

  for (uint32_t n = 0U; n < NR_PBCH_N; n++) {
    llr[n][NR_PBCH_POLAR_LEVELS - 1U][0] = (double)llr_sub[n];
    llr_updated[n][NR_PBCH_POLAR_LEVELS - 1U] = 1U;
    info_pattern[n] = frozen[n] ? 0U : 1U;
    bit_updated[n][0] = (uint8_t)(frozen[n] ? 1U : 0U);
  }
  for (uint32_t p = 0U; p < NR_PBCH_SCL_LIST; p++) {
    crc_state[p] = 1U;
  }

  for (uint32_t current_bit = 0U; current_bit < NR_PBCH_N; current_bit++) {
    nr_pbch_oai_update_llr(current_list_size, (uint16_t)current_bit, 0U,
                           llr, llr_updated, bit, bit_updated);

    if (!info_pattern[current_bit]) {
      nr_pbch_oai_update_path_metric(path_metric, current_list_size, 0U,
                                     (uint16_t)current_bit, llr);
      continue;
    }

    {
      uint32_t cand_count = 0U;

      for (uint32_t p = 0U; p < current_list_size; p++) {
        nr_pbch_oai_copy_path_double(old_llr, (uint8_t)p, llr, (uint8_t)p);
        nr_pbch_oai_copy_path_u8(old_bit, (uint8_t)p, bit, (uint8_t)p);
        old_path_metric[p] = path_metric[p];
        old_crc_state[p] = crc_state[p];
      }
      memcpy(old_crc_checksum, crc_checksum, sizeof(old_crc_checksum));

      for (uint32_t p = 0U; p < current_list_size; p++) {
        cand[cand_count].metric =
            old_path_metric[p] + nr_pbch_softplus(-old_llr[current_bit][0][p]);
        cand[cand_count].parent = (uint8_t)p;
        cand[cand_count].bit = 0U;
        cand[cand_count].crc_state = old_crc_state[p];
        for (uint32_t j = 0U; j < NR_PBCH_CRC; j++) {
          cand[cand_count].crc_checksum[j] = old_crc_checksum[j][p];
        }
        cand_count++;

        cand[cand_count].metric =
            old_path_metric[p] + nr_pbch_softplus(old_llr[current_bit][0][p]);
        cand[cand_count].parent = (uint8_t)p;
        cand[cand_count].bit = 1U;
        cand[cand_count].crc_state = old_crc_state[p];
        for (uint32_t j = 0U; j < NR_PBCH_CRC; j++) {
          cand[cand_count].crc_checksum[j] = old_crc_checksum[j][p];
        }
        cand_count++;
      }

      qsort(cand, cand_count, sizeof(cand[0]), nr_pbch_oai_candidate_cmp);
      current_list_size = (uint8_t)((cand_count < NR_PBCH_SCL_LIST) ? cand_count : NR_PBCH_SCL_LIST);

      for (uint32_t p = 0U; p < current_list_size; p++) {
        const uint8_t parent = cand[p].parent;
        nr_pbch_oai_copy_path_double(llr, (uint8_t)p, old_llr, parent);
        nr_pbch_oai_copy_path_u8(bit, (uint8_t)p, old_bit, parent);
        bit[current_bit][0][p] = cand[p].bit;
        path_metric[p] = cand[p].metric;
        crc_state[p] = cand[p].crc_state;
        for (uint32_t j = 0U; j < NR_PBCH_CRC; j++) {
          crc_checksum[j][p] = cand[p].crc_checksum[j];
        }
      }
      bit_updated[current_bit][0] = 1U;
      non_frozen_bit++;
    }
  }

  {
    uint8_t order[NR_PBCH_SCL_LIST];

    for (uint32_t p = 0U; p < current_list_size; p++) {
      order[p] = (uint8_t)p;
    }
    for (uint32_t i = 0U; i < current_list_size; i++) {
      for (uint32_t j = i + 1U; j < current_list_size; j++) {
        if (path_metric[order[j]] < path_metric[order[i]]) {
          const uint8_t tmp = order[i];
          order[i] = order[j];
          order[j] = tmp;
        }
      }
    }

    for (uint32_t r = 0U; r < current_list_size; r++) {
      const uint8_t path = order[r];
      if (!crc_state[path]) {
        continue;
      }
      for (uint32_t n = 0U; n < NR_PBCH_N; n++) {
        uhat[n] = bit[n][0][path];
      }
      if (nr_pbch_extract_candidate(uhat, info_pos, payload_crc, payload_plain, pci) == 0) {
        return 0;
      }
    }
  }

  return -1;
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
  /* 3GPP TS 38.211 Section 7.4.3.1: map in increasing order of first l
   * (symbol) then k (subcarrier).  Symbol 1 (60), symbol 2 PBCH edges (24),
   * symbol 3 (60) = 144 total. */
  for (uint32_t rel = v; rel < 240U; rel += 4U) {
    rel_idx[pos] = (uint16_t)rel;
    sym_idx[pos++] = 1U;
  }
  for (uint32_t rel = v; rel < 240U; rel += 4U) {
    if (rel < 48U || rel >= 192U) {
      rel_idx[pos] = (uint16_t)rel;
      sym_idx[pos++] = 2U;
    }
  }
  for (uint32_t rel = v; rel < 240U; rel += 4U) {
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
  /* OAI extracts PBCH LLRs symbol-by-symbol (1, then 2 edge REs, then 3),
   * which is the ordering we must follow to match live PBCH decoding. */
  for (uint32_t rel = 0U; rel < 240U; rel++) {
    const uint8_t dmrs = (uint8_t)((rel & 3U) == v);
    if (!dmrs) {
      rel_idx[pos] = (uint16_t)rel;
      sym_idx[pos++] = 1U;
    }
  }
  for (uint32_t rel = 0U; rel < 240U; rel++) {
    const uint8_t dmrs = (uint8_t)((rel & 3U) == v);
    if ((rel < 48U || rel >= 192U) && !dmrs) {
      rel_idx[pos] = (uint16_t)rel;
      sym_idx[pos++] = 2U;
    }
  }
  for (uint32_t rel = 0U; rel < 240U; rel++) {
    const uint8_t dmrs = (uint8_t)((rel & 3U) == v);
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
  float llr_scr[NR_PBCH_E];
  float llr_rm[NR_PBCH_N];
  float llr_sub[NR_PBCH_N];
  uint16_t info_pos[NR_PBCH_K];
  uint8_t frozen[NR_PBCH_N];
  uint8_t payload_crc[NR_PBCH_K];
  uint8_t payload_plain[NR_PBCH_A];
  uint16_t sfn = 0U;
  uint32_t mib = 0U;

  if (!llr || llr_len < NR_PBCH_E || !sync) {
    return -1;
  }

  memcpy(llr_scr, llr, sizeof(llr_scr));

  {
    static uint32_t llr_dbg = 0U;
    if ((llr_dbg++ % 20U) == 0U) {
      float sum_abs = 0.0f, mx = 0.0f;
      uint32_t npos = 0U;
      for (uint32_t i = 0; i < NR_PBCH_E; i++) {
        float a = llr_scr[i] < 0 ? -llr_scr[i] : llr_scr[i];
        sum_abs += a;
        if (a > mx) mx = a;
        if (llr_scr[i] > 0) npos++;
      }
      printf("  LLR_PRE_SCR: pci=%u ssb=%u mean_abs=%.2f max=%.2f pos_frac=%.3f\n",
             (unsigned)sync->pci, (unsigned)sync->ssb_index,
             sum_abs / NR_PBCH_E, mx, (float)npos / NR_PBCH_E);
    }
  }

  nr_pbch_codeword_descramble_llr(sync->pci, sync->ssb_index, llr_scr);
  nr_pbch_rate_match_decode(llr_scr, llr_rm);
  nr_pbch_subblock_deinterleave_llr(llr_rm, llr_sub);

  nr_pbch_build_info_set(info_pos, frozen);
  if (nr_pbch_oai_generic_decoder_int16(llr_sub, frozen, info_pos,
                                        payload_crc, payload_plain, sync->pci) != 0) {
    return -1;
  }

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
