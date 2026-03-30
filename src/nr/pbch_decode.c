#include "nr/pbch_decode.h"
#include "nr/pbch_re.h"
#include "nr/polar.h"
#include "common/error.h"

#include <string.h>

void nr_pbch_decode_cfg_default(nr_pbch_decode_cfg_t *cfg)
{
  if (!cfg)
    return;
  cfg->enable_polar_bitrev = 0;
  cfg->enable_pbch_descramble = 1;
  cfg->enable_polar_info_bitrev = 0;
  cfg->crc_bit_order = 0;
}

static uint32_t xorshift32(uint32_t *state)
{
  uint32_t x = *state;
  x ^= x << 13;
  x ^= x >> 17;
  x ^= x << 5;
  *state = x;
  return x;
}

static uint32_t crc24c_bits(const uint8_t *bits, uint32_t nbits)
{
  uint32_t reg = 0;
  const uint32_t poly = 0x1864CFBu; /* CRC24C polynomial */

  for (uint32_t i = 0; i < nbits; ++i) {
    uint32_t inb = (bits[i] & 1u);
    uint32_t msb = (reg >> 23) & 1u;
    reg = ((reg << 1) & 0xFFFFFFu) | inb;
    if (msb)
      reg ^= (poly & 0xFFFFFFu);
  }
  return reg & 0xFFFFFFu;
}

static void bits_to_bytes(const uint8_t *bits, uint32_t nbits, uint8_t *out)
{
  uint32_t nbytes = (nbits + 7u) / 8u;
  memset(out, 0, nbytes);
  for (uint32_t i = 0; i < nbits; ++i) {
    if (bits[i])
      out[i / 8u] |= (uint8_t)(1u << (7u - (i % 8u)));
  }
}

static void pbch_rate_recover_simple(const float *in, uint32_t e, float *out, uint32_t n)
{
  for (uint32_t i = 0; i < n; ++i)
    out[i] = 0.0f;
  for (uint32_t j = 0; j < e; ++j)
    out[j % n] += in[j];
}

static uint32_t bit_reverse_u32(uint32_t x, uint32_t nbits)
{
  uint32_t r = 0;
  for (uint32_t i = 0; i < nbits; ++i) {
    r = (r << 1) | (x & 1u);
    x >>= 1u;
  }
  return r;
}

int nr_pbch_decode_from_dmrs(const nr_ssb_result_t *ssb_res,
                             const cf32_t *fft_syms,
                             uint32_t n_sym,
                             uint32_t fft_size,
                             float dmrs_metric,
                             float dmrs_peak_ratio,
                             uint32_t dmrs_re_count,
                             uint32_t i_ssb,
                             const nr_pbch_decode_cfg_t *cfg,
                             nr_pbch_decode_result_t *out)
{
  nr_pbch_decode_cfg_t cfg_stack;
  const nr_pbch_decode_cfg_t *c;
  float llr_pool[2048];
  float llr_n[512];
  float llr_n_br[512];
  uint8_t frozen[512];
  uint8_t frozen_br[512];
  uint8_t u_hat[512];
  uint8_t info_bits[80];
  uint32_t llr_num = 0;
  const uint32_t n = 512;
  const uint32_t k = 56;   /* 32 payload + 24 CRC */
  const uint32_t info_n = 56;
  uint32_t scramble;
  uint32_t crc_rx, crc_calc;

  if (!ssb_res || !out)
    return TOA_ERR_INVALID_ARG;

  if (cfg)
    c = cfg;
  else {
    nr_pbch_decode_cfg_default(&cfg_stack);
    c = &cfg_stack;
  }

  memset(out, 0, sizeof(*out));
  out->valid_input = (ssb_res->found && dmrs_re_count > 0 && fft_syms && n_sym > 0 && fft_size >= 64);
  out->payload_bits = 32;
  out->quality_metric = dmrs_metric * dmrs_peak_ratio;
  if (!out->valid_input)
    return TOA_OK;

  if (nr_pbch_extract_llr(fft_syms, n_sym, fft_size,
                          (ssb_res->pci >= 0) ? (uint32_t)ssb_res->pci : 0u,
                          i_ssb, llr_pool, 2048, &llr_num) != TOA_OK)
    return TOA_OK;
  if (llr_num < 64)
    return TOA_OK;

  pbch_rate_recover_simple(llr_pool, llr_num, llr_n, n);

  if (c->enable_pbch_descramble) {
    /* Proto descrambling on LLR sign (cinit-like seed) */
    scramble = ((ssb_res->pci >= 0) ? (uint32_t)ssb_res->pci : 0u) ^ (i_ssb * 0x9E3779B1u) ^ 0xA5A5A5A5u;
    if (scramble == 0)
      scramble = 1u;
    for (uint32_t i = 0; i < n; ++i)
      if (xorshift32(&scramble) & 1u)
        llr_n[i] = -llr_n[i];
  }

  nr_polar_build_frozen_set(n, k, frozen);
  if (c->enable_polar_bitrev) {
    /* Align bit-channel indexing between rate-recovered LLR order and decoder order. */
    const uint32_t nbits = 9; /* log2(512) */
    for (uint32_t i = 0; i < n; ++i) {
      uint32_t ir = bit_reverse_u32(i, nbits);
      llr_n_br[i] = llr_n[ir];
      frozen_br[i] = frozen[ir];
    }
    nr_polar_sc_decode(llr_n_br, n, frozen_br, u_hat);
  } else {
    nr_polar_sc_decode(llr_n, n, frozen, u_hat);
  }

  {
    uint32_t j = 0;
    memset(info_bits, 0, sizeof(info_bits));
    if (c->enable_polar_bitrev && c->enable_polar_info_bitrev) {
      const uint32_t nbits = 9;
      for (uint32_t i = 0; i < n && j < info_n; ++i) {
        uint32_t ir = bit_reverse_u32(i, nbits);
        if (!frozen[ir])
          info_bits[j++] = u_hat[i] & 1u;
      }
    } else {
      for (uint32_t i = 0; i < n && j < info_n; ++i) {
        if (!frozen[i])
          info_bits[j++] = u_hat[i] & 1u;
      }
    }
    if (j < info_n)
      return TOA_OK;
  }

  if (c->crc_bit_order == 1) {
    uint8_t d32[32];
    memset(d32, 0, sizeof(d32));
    for (uint32_t i = 0; i < 32; ++i)
      d32[i] = info_bits[31u - i] & 1u;
    crc_calc = crc24c_bits(d32, 32);
    crc_rx = 0;
    for (uint32_t i = 0; i < 24; ++i)
      crc_rx = (crc_rx << 1) | (info_bits[55u - i] & 1u);
  } else {
    crc_calc = crc24c_bits(info_bits, 32);
    crc_rx = 0;
    for (uint32_t i = 0; i < 24; ++i)
      crc_rx = (crc_rx << 1) | (info_bits[32 + i] & 1u);
  }

  out->crc_ok = (crc_calc == crc_rx);
  bits_to_bytes(info_bits, 32, out->payload);
  return TOA_OK;
}
