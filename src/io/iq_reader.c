#include "io/iq_reader.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

typedef struct {
  int valid;
  uint64_t sample_offset;
  int64_t hw_time_ns;
  uint32_t nsamps;
  uint32_t flags;
} iq_blocklog_row_t;

static void make_default_meta_path(const char *iq_path, char *out, size_t outsz)
{
  size_t n;

  if (!iq_path || !out || outsz == 0) {
    if (out && outsz)
      out[0] = '\0';
    return;
  }
  n = strlen(iq_path);
  if (n > 5 && strcmp(iq_path + n - 5, ".cf32") == 0) {
    snprintf(out, outsz, "%.*s.meta.json", (int)(n - 5), iq_path);
  } else {
    snprintf(out, outsz, "%s.meta.json", iq_path);
  }
}

static void make_default_blocklog_path(const char *iq_path, char *out, size_t outsz)
{
  size_t n;

  if (!iq_path || !out || outsz == 0) {
    if (out && outsz)
      out[0] = '\0';
    return;
  }
  n = strlen(iq_path);
  if (n > 5 && strcmp(iq_path + n - 5, ".cf32") == 0) {
    snprintf(out, outsz, "%.*s.blocklog.csv", (int)(n - 5), iq_path);
  } else {
    snprintf(out, outsz, "%s.blocklog.csv", iq_path);
  }
}

static toa_error_t iq_reader_load_blocklog(iq_reader_t *r,
                                           const char *iq_path,
                                           const char *blocklog_path)
{
  char auto_path[512];
  const char *path;
  int required;
  FILE *fp;
  char line[384];
  uint32_t cap = 0;
  iq_blocklog_row_t *blog = NULL;

  if (!r || !iq_path)
    return TOA_ERR_INVALID_ARG;

  if (blocklog_path && blocklog_path[0]) {
    path = blocklog_path;
    required = 1;
  } else if (blocklog_path == NULL) {
    make_default_blocklog_path(iq_path, auto_path, sizeof(auto_path));
    path = auto_path;
    required = 0;
  } else {
    return TOA_OK; /* "" => disabled */
  }

  fp = fopen(path, "r");
  if (!fp)
    return required ? TOA_ERR_IO : TOA_OK;

  while (fgets(line, (int)sizeof(line), fp)) {
    uint64_t bidx, off;
    int64_t tns;
    unsigned ns_u = 0, fl_u = 0;
    int nscan;

    if (strncmp(line, "block_idx", 9) == 0)
      continue;

    nscan = sscanf(line,
                   "%" SCNu64 ",%" SCNu64 ",%" SCNd64 ",%u,%u",
                   &bidx,
                   &off,
                   &tns,
                   &ns_u,
                   &fl_u);
    if (nscan < 4)
      continue;

    if (bidx >= 10000000u)
      continue;

    if ((uint32_t)(bidx + 1u) > cap) {
      size_t nc = (size_t)(bidx + 1u);
      iq_blocklog_row_t *nb = (iq_blocklog_row_t *)realloc(blog, nc * sizeof(*nb));
      if (!nb) {
        fclose(fp);
        free(blog);
        return TOA_ERR_NO_MEMORY;
      }
      memset(nb + cap, 0, (nc - (size_t)cap) * sizeof(*nb));
      blog = nb;
      cap = (uint32_t)nc;
    }

    blog[bidx].valid = 1;
    blog[bidx].sample_offset = off;
    blog[bidx].hw_time_ns = tns;
    blog[bidx].nsamps = (uint32_t)ns_u;
    blog[bidx].flags = (nscan >= 5) ? (uint32_t)fl_u : 0u;
  }

  fclose(fp);
  r->blog_priv = blog;
  r->blog_cap = cap;
  return TOA_OK;
}

toa_error_t iq_reader_open(iq_reader_t *r,
                           const char *iq_path,
                           const char *meta_path,
                           const char *blocklog_path)
{
  char auto_meta[512];
  toa_error_t me;
  toa_error_t ble;

  if (!r || !iq_path)
    return TOA_ERR_INVALID_ARG;

  memset(r, 0, sizeof(*r));
  iq_meta_init_default(&r->meta);

  r->fp = fopen(iq_path, "rb");
  if (!r->fp)
    return TOA_ERR_IO;

  if (meta_path && meta_path[0]) {
    me = iq_meta_load_json(meta_path, &r->meta);
    if (me != TOA_OK) {
      fclose(r->fp);
      r->fp = NULL;
      return me;
    }
    r->meta_loaded = 1;
  } else {
    make_default_meta_path(iq_path, auto_meta, sizeof(auto_meta));
    me = iq_meta_load_json(auto_meta, &r->meta);
    if (me == TOA_OK && (r->meta.sample_rate_hz > 0.0 || r->meta.format_version > 0u))
      r->meta_loaded = 1;
  }

  if (r->meta_loaded && iq_meta_has_sample_rate(&r->meta))
    r->time_fs_hz = r->meta.sample_rate_hz;
  r->synthetic_time_ns = r->meta.capture_start_time_ns;

  ble = iq_reader_load_blocklog(r, iq_path, blocklog_path);
  if (ble != TOA_OK) {
    fclose(r->fp);
    r->fp = NULL;
    free(r->blog_priv);
    r->blog_priv = NULL;
    r->blog_cap = 0;
    return ble;
  }

  return TOA_OK;
}

void iq_reader_close(iq_reader_t *r)
{
  if (!r)
    return;
  if (r->fp) {
    fclose(r->fp);
    r->fp = NULL;
  }
  free(r->blog_priv);
  r->blog_priv = NULL;
  r->blog_cap = 0;
}

void iq_reader_set_time_fs_hz(iq_reader_t *r, double fs_hz)
{
  if (!r)
    return;
  if (fs_hz > 0.0)
    r->time_fs_hz = fs_hz;
}

toa_error_t iq_reader_next_block(iq_reader_t *r,
                                 cf32_t *buf,
                                 uint32_t max_samples,
                                 uint32_t *nread_out,
                                 iq_block_info_t *info_out)
{
  size_t nread;
  uint64_t sample_index0;
  int64_t hw_from_log = 0;
  int use_log = 0;
  uint32_t log_flags = 0;

  if (!r || !r->fp || !buf || max_samples == 0 || !nread_out)
    return TOA_ERR_INVALID_ARG;

  sample_index0 = r->sample_cursor;

  nread = fread(buf, sizeof(cf32_t), max_samples, r->fp);
  *nread_out = (uint32_t)nread;

  if (nread == 0)
    return TOA_OK;

  {
    iq_blocklog_row_t *blog = (iq_blocklog_row_t *)r->blog_priv;
    if (blog && r->block_index < r->blog_cap && blog[r->block_index].valid) {
      iq_blocklog_row_t *row = &blog[r->block_index];
      if (row->sample_offset == sample_index0) {
        hw_from_log = row->hw_time_ns;
        log_flags = row->flags;
        use_log = 1;
      }
    }
  }

  if (info_out) {
    info_out->block_index = r->block_index;
    info_out->sample_index0 = sample_index0;
    info_out->hw_time_ns0 = use_log ? hw_from_log : r->synthetic_time_ns;
    info_out->nsamps = (uint32_t)nread;
    info_out->flags = use_log ? log_flags : 0u;
  }

  r->block_index++;
  r->sample_cursor += (uint64_t)nread;
  if (r->time_fs_hz > 0.0)
    r->synthetic_time_ns += (int64_t)((1e9 * (double)nread) / r->time_fs_hz);

  return TOA_OK;
}

toa_error_t iq_replay_fill_fs_from_meta(const char *iq_path,
                                        const char *meta_path_opt,
                                        int fs_from_env,
                                        double *fs_hz_io)
{
  char auto_meta[512];
  iq_meta_t m;
  toa_error_t e;

  if (!iq_path || !fs_hz_io)
    return TOA_ERR_INVALID_ARG;
  if (fs_from_env)
    return TOA_OK;
  if (meta_path_opt && meta_path_opt[0])
    e = iq_meta_load_json(meta_path_opt, &m);
  else {
    make_default_meta_path(iq_path, auto_meta, sizeof(auto_meta));
    e = iq_meta_load_json(auto_meta, &m);
  }
  if (e == TOA_OK && iq_meta_has_sample_rate(&m))
    *fs_hz_io = m.sample_rate_hz;
  return TOA_OK;
}
