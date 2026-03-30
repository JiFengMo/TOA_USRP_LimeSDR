#include "io/iq_meta.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

void iq_meta_init_default(iq_meta_t *m)
{
  if (!m)
    return;
  memset(m, 0, sizeof(*m));
  m->format_version = 1u;
}

int iq_meta_has_sample_rate(const iq_meta_t *m)
{
  if (!m)
    return 0;
  return m->sample_rate_hz > 0.0;
}

static const char *skip_ws(const char *p)
{
  while (p && (*p == ' ' || *p == '\t' || *p == '\n' || *p == '\r'))
    ++p;
  return p;
}

static int parse_json_number_after_key(const char *buf, const char *key, double *out)
{
  const char *p = strstr(buf, key);
  char *endp = NULL;
  double v;
  if (!p || !out)
    return -1;
  p = strchr(p, ':');
  if (!p)
    return -1;
  p = skip_ws(p + 1);
  v = strtod(p, &endp);
  if (endp == p)
    return -1;
  *out = v;
  return 0;
}

static int parse_json_u64_after_key(const char *buf, const char *key, uint64_t *out)
{
  const char *p = strstr(buf, key);
  char *endp = NULL;
  unsigned long long v;
  if (!p || !out)
    return -1;
  p = strchr(p, ':');
  if (!p)
    return -1;
  p = skip_ws(p + 1);
  v = strtoull(p, &endp, 10);
  if (endp == p)
    return -1;
  *out = (uint64_t)v;
  return 0;
}

static int parse_json_i64_after_key(const char *buf, const char *key, int64_t *out)
{
  const char *p = strstr(buf, key);
  char *endp = NULL;
  long long v;
  if (!p || !out)
    return -1;
  p = strchr(p, ':');
  if (!p)
    return -1;
  p = skip_ws(p + 1);
  v = strtoll(p, &endp, 10);
  if (endp == p)
    return -1;
  *out = (int64_t)v;
  return 0;
}

static int parse_json_u32_after_key(const char *buf, const char *key, uint32_t *out)
{
  uint64_t v = 0;
  if (parse_json_u64_after_key(buf, key, &v) != 0)
    return -1;
  *out = (uint32_t)v;
  return 0;
}

static int parse_json_string_after_key(const char *buf, const char *key, char *out, size_t outsz)
{
  const char *p = strstr(buf, key);
  const char *q;
  size_t n = 0;
  if (!p || !out || outsz == 0)
    return -1;
  p = strchr(p, ':');
  if (!p)
    return -1;
  p = skip_ws(p + 1);
  if (*p != '"')
    return -1;
  ++p;
  q = p;
  while (*q && *q != '"' && n + 1 < outsz) {
    if (*q == '\\' && q[1])
      ++q;
    out[n++] = *q++;
  }
  out[n] = '\0';
  return 0;
}

toa_error_t iq_meta_load_json(const char *path, iq_meta_t *m)
{
  FILE *fp = NULL;
  char *buf = NULL;
  long sz;
  size_t nread;
  toa_error_t err = TOA_OK;

  if (!path || !m)
    return TOA_ERR_INVALID_ARG;

  iq_meta_init_default(m);

  fp = fopen(path, "rb");
  if (!fp)
    return TOA_ERR_IO;

  if (fseek(fp, 0, SEEK_END) != 0) {
    err = TOA_ERR_IO;
    goto out;
  }
  sz = ftell(fp);
  if (sz < 0 || sz > 65536) {
    err = TOA_ERR_UNSUPPORTED;
    goto out;
  }
  if (fseek(fp, 0, SEEK_SET) != 0) {
    err = TOA_ERR_IO;
    goto out;
  }
  buf = (char *)malloc((size_t)sz + 1u);
  if (!buf) {
    err = TOA_ERR_NO_MEMORY;
    goto out;
  }
  nread = fread(buf, 1, (size_t)sz, fp);
  if (nread != (size_t)sz) {
    err = TOA_ERR_IO;
    goto out;
  }
  buf[sz] = '\0';

  (void)parse_json_u32_after_key(buf, "\"format_version\"", &m->format_version);
  (void)parse_json_number_after_key(buf, "\"sample_rate_hz\"", &m->sample_rate_hz);
  (void)parse_json_number_after_key(buf, "\"center_freq_hz\"", &m->center_freq_hz);
  (void)parse_json_number_after_key(buf, "\"bandwidth_hz\"", &m->bandwidth_hz);
  (void)parse_json_number_after_key(buf, "\"gain_db\"", &m->gain_db);
  (void)parse_json_u32_after_key(buf, "\"block_size\"", &m->block_size);
  (void)parse_json_i64_after_key(buf, "\"capture_start_time_ns\"", &m->capture_start_time_ns);
  (void)parse_json_string_after_key(buf, "\"device_args\"", m->device_args, sizeof(m->device_args));
  (void)parse_json_u64_after_key(buf, "\"total_samples\"", &m->total_samples);

out:
  free(buf);
  if (fp)
    fclose(fp);
  return err;
}

static void fputs_json_escaped(FILE *fp, const char *s)
{
  const unsigned char *p = (const unsigned char *)s;
  if (!fp)
    return;
  if (!p) {
    return;
  }
  while (*p) {
    if (*p == '"' || *p == '\\')
      fputc('\\', fp);
    fputc((int)*p, fp);
    ++p;
  }
}

toa_error_t iq_meta_save_json(const char *path, const iq_meta_t *m)
{
  FILE *fp;

  if (!path || !m)
    return TOA_ERR_INVALID_ARG;

  fp = fopen(path, "wb");
  if (!fp)
    return TOA_ERR_IO;

  fprintf(fp,
          "{\n"
          "  \"format_version\": %u,\n"
          "  \"sample_rate_hz\": %.15g,\n"
          "  \"center_freq_hz\": %.15g,\n"
          "  \"bandwidth_hz\": %.15g,\n"
          "  \"gain_db\": %.15g,\n"
          "  \"block_size\": %u,\n"
          "  \"capture_start_time_ns\": %lld,\n"
          "  \"device_args\": \"",
          m->format_version,
          m->sample_rate_hz,
          m->center_freq_hz,
          m->bandwidth_hz,
          m->gain_db,
          m->block_size,
          (long long)m->capture_start_time_ns);
  fputs_json_escaped(fp, m->device_args);
  fprintf(fp,
          "\",\n"
          "  \"total_samples\": %llu\n"
          "}\n",
          (unsigned long long)m->total_samples);

  fclose(fp);
  return TOA_OK;
}
