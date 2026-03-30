#include "nr/polar.h"
#include "common/error.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

typedef struct {
  uint32_t idx;
  float w;
} ch_w_t;

static int cmp_desc(const void *a, const void *b)
{
  const ch_w_t *x = (const ch_w_t *)a;
  const ch_w_t *y = (const ch_w_t *)b;
  if (x->w > y->w)
    return -1;
  if (x->w < y->w)
    return 1;
  return (x->idx < y->idx) ? -1 : 1;
}

int nr_polar_build_frozen_set(uint32_t n, uint32_t k, uint8_t *frozen)
{
  ch_w_t *arr = NULL;
  uint32_t m = 0;

  if (!frozen || n == 0 || (n & (n - 1)) != 0 || k >= n)
    return TOA_ERR_INVALID_ARG;

  while ((1u << m) < n)
    m++;

  arr = (ch_w_t *)calloc(n, sizeof(*arr));
  if (!arr)
    return TOA_ERR_NO_MEMORY;

  for (uint32_t i = 0; i < n; ++i) {
    float pw = 0.0f;
    for (uint32_t b = 0; b < m; ++b) {
      if ((i >> b) & 1u)
        pw += powf(2.0f, 0.25f * (float)b);
    }
    arr[i].idx = i;
    arr[i].w = pw;
    frozen[i] = 1; /* frozen by default */
  }

  qsort(arr, n, sizeof(*arr), cmp_desc);
  for (uint32_t i = 0; i < k; ++i)
    frozen[arr[i].idx] = 0; /* information bit */

  free(arr);
  return TOA_OK;
}

static float f_node(float a, float b)
{
  float sa = (a >= 0.0f) ? 1.0f : -1.0f;
  float sb = (b >= 0.0f) ? 1.0f : -1.0f;
  float aa = fabsf(a), bb = fabsf(b);
  return sa * sb * ((aa < bb) ? aa : bb);
}

static float g_node(float a, float b, uint8_t u)
{
  return b + ((u == 0u) ? a : -a);
}

static void sc_decode_rec(const float *llr, uint32_t n, const uint8_t *frozen, uint8_t *u)
{
  if (n == 1) {
    if (frozen[0])
      u[0] = 0;
    else
      u[0] = (llr[0] < 0.0f) ? 1u : 0u;
    return;
  }

  uint32_t h = n / 2;
  float *l_left = (float *)calloc(h, sizeof(float));
  float *l_right = (float *)calloc(h, sizeof(float));
  uint8_t *u_left = (uint8_t *)calloc(h, sizeof(uint8_t));
  uint8_t *u_right = (uint8_t *)calloc(h, sizeof(uint8_t));

  if (!l_left || !l_right || !u_left || !u_right) {
    free(l_left); free(l_right); free(u_left); free(u_right);
    return;
  }

  for (uint32_t i = 0; i < h; ++i)
    l_left[i] = f_node(llr[i], llr[i + h]);
  sc_decode_rec(l_left, h, frozen, u_left);

  for (uint32_t i = 0; i < h; ++i)
    l_right[i] = g_node(llr[i], llr[i + h], u_left[i]);
  sc_decode_rec(l_right, h, frozen + h, u_right);

  for (uint32_t i = 0; i < h; ++i) {
    u[i] = u_left[i] ^ u_right[i];
    u[i + h] = u_right[i];
  }

  free(l_left); free(l_right); free(u_left); free(u_right);
}

int nr_polar_sc_decode(const float *llr, uint32_t n, const uint8_t *frozen, uint8_t *u_hat)
{
  if (!llr || !frozen || !u_hat || n == 0 || (n & (n - 1)) != 0)
    return TOA_ERR_INVALID_ARG;
  memset(u_hat, 0, n);
  sc_decode_rec(llr, n, frozen, u_hat);
  return TOA_OK;
}
