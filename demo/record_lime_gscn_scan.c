#include "radio/radio_factory.h"
#include "common/cli.h"
#include "common/demo_rx_defaults.h"
#include "common/error.h"
#include "common/log.h"
#include "io/capture_session.h"
#include "io/iq_meta.h"
#include "nr/gscn.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>

static void clamp_rx_to_caps(radio_channel_config_t *rx_cfg, const radio_device_caps_t *caps)
{
  if (rx_cfg->sample_rate < caps->sample_rate_min)
    rx_cfg->sample_rate = caps->sample_rate_min;
  if (rx_cfg->sample_rate > caps->sample_rate_max)
    rx_cfg->sample_rate = caps->sample_rate_max;
  if (rx_cfg->bandwidth < caps->bandwidth_min)
    rx_cfg->bandwidth = caps->bandwidth_min;
  if (rx_cfg->bandwidth > caps->bandwidth_max)
    rx_cfg->bandwidth = caps->bandwidth_max;
  if (rx_cfg->center_freq < caps->center_freq_min)
    rx_cfg->center_freq = caps->center_freq_min;
  if (rx_cfg->center_freq > caps->center_freq_max)
    rx_cfg->center_freq = caps->center_freq_max;
  if (rx_cfg->gain < caps->gain_min)
    rx_cfg->gain = caps->gain_min;
  if (rx_cfg->gain > caps->gain_max)
    rx_cfg->gain = caps->gain_max;
}

static int discard_blocks(radio_device_t *device,
                          void *buffs[1],
                          uint32_t buffer_size,
                          uint32_t n_blocks)
{
  for (uint32_t b = 0; b < n_blocks; b++) {
    radio_rx_result_t rx_result;
    int ret;

    memset(&rx_result, 0, sizeof(rx_result));
    ret = device->trx_read_func(device, buffs, buffer_size, &rx_result, 100000);
    if (ret != 0)
      return ret;
  }
  return 0;
}

static nr_gscn_profile_t parse_profile(const char *s)
{
  if (!s || s[0] == '\0')
    return NR_GSCN_PROFILE_TS38104_N41_SCS30K;
  if (strcmp(s, "fr1_hz") == 0 || strcmp(s, "fr1") == 0)
    return NR_GSCN_PROFILE_FR1_FREQ_WINDOW;
  if (strcmp(s, "n41_15k") == 0 || strcmp(s, "n41_scs15") == 0)
    return NR_GSCN_PROFILE_TS38104_N41_SCS15K;
  if (strcmp(s, "n41_30k") == 0 || strcmp(s, "n41_scs30") == 0)
    return NR_GSCN_PROFILE_TS38104_N41_SCS30K;
  return NR_GSCN_PROFILE_TS38104_N41_SCS30K;
}

/* 1 = OAI: one RX LO, multiple GSCN in passband; 0 = tune LO per GSCN (SS_REF). */
static int parse_record_tune(const char *s)
{
  if (!s || s[0] == '\0')
    return 0;
  if (strcmp(s, "oai_center") == 0 || strcmp(s, "center") == 0 || strcmp(s, "oai") == 0)
    return 1;
  return 0;
}

static int write_visible_gscn_sidecar(const char *base, const uint32_t *g, size_t n)
{
  char path[576];
  FILE *fp;
  size_t i;

  if (snprintf(path, sizeof(path), "%s.visible_gscn.txt", base) >= (int)sizeof(path))
    return -1;
  fp = fopen(path, "w");
  if (!fp)
    return -1;
  fprintf(fp,
          "# OAI-style: GSCN on configured raster with SS_REF inside RX passband "
          "(offline search / cell detect per entry)\n");
  for (i = 0; i < n; i++) {
    if (i)
      fputc(',', fp);
    fprintf(fp, "%u", g[i]);
  }
  fprintf(fp, "\n");
  fclose(fp);
  return 0;
}

static uint32_t gscn_min_arr(const uint32_t *v, size_t n)
{
  uint32_t m = UINT32_MAX;
  size_t i;

  for (i = 0; i < n; i++) {
    if (v[i] < m)
      m = v[i];
  }
  return m;
}

static uint32_t gscn_max_arr(const uint32_t *v, size_t n)
{
  uint32_t m = 0;
  size_t i;

  for (i = 0; i < n; i++) {
    if (v[i] > m)
      m = v[i];
  }
  return m;
}

int main(void)
{
  radio_device_t device;
  radio_device_caps_t caps;
  radio_channel_config_t rx_cfg;
  radio_stream_config_t rx_stream_cfg;
  capture_session_t session;
  iq_meta_t meta;
  cf32_t *rx_buffer = NULL;
  void *buffs[1];
  uint32_t cand[8192];
  uint32_t vis[4096];
  double centers[128];
  size_t n_cand = 0;
  size_t n_cent = 0;
  int caps_ok = 0;
  int ret;
  int rc = -1;
  int session_opened = 0;
  FILE *idx_fp = NULL;

  const char *prefix = "rx_gscn";
  const char *out_dir = NULL;
  const char *index_path = "gscn_record_index.csv";
  const char *preset = NULL;
  int blocks_env = 0;
  int overlap_env = 0;
  int warmup_env = 0;
  int settle_env = 0;
  int tune_env = 0;
  int num_blocks = 150;
  uint32_t stride = 1u;
  uint32_t warmup_blocks = 2u;
  uint32_t settle_ms = 0u;
  uint32_t rounds = 1u;
  int infinite_rounds = 0;
  double fmin_mhz = 2496.0;
  double fmax_mhz = 2690.0;
  int use_gscn_range = 0;
  uint32_t gscn_first = 0u;
  uint32_t gscn_last = 0u;
  nr_gscn_profile_t profile = NR_GSCN_PROFILE_TS38104_N41_SCS30K;
  int tune_oai_center = 0;
  double overlap_frac = 0.15;
  int idx_per_ssb = 1;

  memset(&device, 0, sizeof(device));
  memset(&session, 0, sizeof(session));
  log_set_level(LOG_LEVEL_INFO);

  preset = getenv("RECORD_GSCN_PRESET");
  tune_env = (getenv("RECORD_GSCN_TUNE") != NULL);
  blocks_env = (getenv("RECORD_GSCN_BLOCKS") != NULL);
  overlap_env = (getenv("RECORD_GSCN_OVERLAP") != NULL);
  warmup_env = (getenv("RECORD_GSCN_WARMUP_BLOCKS") != NULL);
  settle_env = (getenv("RECORD_GSCN_SETTLE_MS") != NULL);

  profile = parse_profile(getenv("RECORD_GSCN_PROFILE"));

  if (getenv("RECORD_GSCN_PREFIX"))
    prefix = getenv("RECORD_GSCN_PREFIX");
  if (getenv("RECORD_GSCN_OUT_DIR"))
    out_dir = getenv("RECORD_GSCN_OUT_DIR");
  if (getenv("RECORD_GSCN_INDEX"))
    index_path = getenv("RECORD_GSCN_INDEX");

  if (preset != NULL &&
      (strcmp(preset, "oai") == 0 || strcmp(preset, "OAI") == 0)) {
    if (!blocks_env)
      num_blocks = 200;
    if (!warmup_env)
      warmup_blocks = 3u;
    if (!settle_env)
      settle_ms = 5u;
    if (!overlap_env)
      overlap_frac = 0.15;
  }

  if (blocks_env) {
    uint32_t nb = 0;
    if (cli_env_u32("RECORD_GSCN_BLOCKS", &nb) == 0 && nb > 0u && nb < 1000000u)
      num_blocks = (int)nb;
  }

  if (warmup_env)
    (void)cli_env_u32("RECORD_GSCN_WARMUP_BLOCKS", &warmup_blocks);
  if (settle_env) {
    (void)cli_env_u32("RECORD_GSCN_SETTLE_MS", &settle_ms);
    if (settle_ms > 30000u)
      settle_ms = 30000u;
  }

  if (overlap_env) {
    double ov = 0.0;
    if (cli_env_f64("RECORD_GSCN_OVERLAP", &ov) == 0 && ov >= 0.0 && ov < 0.95)
      overlap_frac = ov;
  }

  if (getenv("RECORD_GSCN_ROUNDS") != NULL) {
    uint32_t r = 1u;
    if (cli_env_u32("RECORD_GSCN_ROUNDS", &r) == 0) {
      if (r == 0u)
        infinite_rounds = 1;
      else
        rounds = r;
    }
  }

  if (getenv("RECORD_GSCN_FIRST") != NULL && getenv("RECORD_GSCN_LAST") != NULL &&
      cli_env_u32("RECORD_GSCN_FIRST", &gscn_first) == 0 &&
      cli_env_u32("RECORD_GSCN_LAST", &gscn_last) == 0) {
    use_gscn_range = 1;
  } else {
    (void)cli_env_f64("RECORD_GSCN_FMIN_MHZ", &fmin_mhz);
    (void)cli_env_f64("RECORD_GSCN_FMAX_MHZ", &fmax_mhz);
  }

  {
    int stride_env = (getenv("RECORD_GSCN_STRIDE") != NULL);

    if (stride_env) {
      (void)cli_env_u32("RECORD_GSCN_STRIDE", &stride);
      if (stride == 0u)
        stride = 1u;
    } else if (use_gscn_range) {
      stride = NR_GSCN_SYNC_RASTER_STEP;
    } else {
      stride = 1u;
    }

    if (use_gscn_range && stride == 1u && stride_env)
      log_msg(LOG_LEVEL_WARN,
              "GSCN_REC",
              "GSCN range with RECORD_GSCN_STRIDE=1 includes off-raster points; use 3 for n41 or omit STRIDE");
    else if (use_gscn_range && !stride_env)
      log_msg(LOG_LEVEL_INFO,
              "GSCN_REC",
              "RECORD_GSCN_STRIDE unset + FIRST/LAST: using stride=%u (n41 sync raster step)",
              stride);
  }

  if (tune_env)
    tune_oai_center = parse_record_tune(getenv("RECORD_GSCN_TUNE"));
  else if (preset != NULL && (strcmp(preset, "oai") == 0 || strcmp(preset, "OAI") == 0))
    tune_oai_center = 1;
  else if (!use_gscn_range && profile != NR_GSCN_PROFILE_FR1_FREQ_WINDOW)
    tune_oai_center = 1;
  else
    tune_oai_center = 0;

  if (use_gscn_range) {
    if (tune_oai_center) {
      log_msg(LOG_LEVEL_INFO,
              "GSCN_REC",
              "RECORD_GSCN_FIRST/LAST set: OAI center mode disabled, tuning per GSCN");
      tune_oai_center = 0;
    }
    ret = (int)nr_gscn_fr1_collect_range(gscn_first,
                                         gscn_last,
                                         stride,
                                         cand,
                                         sizeof(cand) / sizeof(cand[0]),
                                         &n_cand);
  } else {
    double lo_hz = fmin_mhz * 1e6;
    double hi_hz = fmax_mhz * 1e6;

    if (lo_hz >= hi_hz) {
      log_msg(LOG_LEVEL_ERROR, "GSCN_REC", "FMIN_MHZ must be < FMAX_MHZ");
      goto out;
    }
    ret = (int)nr_gscn_collect_for_profile(profile, lo_hz, hi_hz, stride, cand, sizeof(cand) / sizeof(cand[0]), &n_cand);
  }

  if (ret != TOA_OK || n_cand == 0) {
    log_msg(LOG_LEVEL_ERROR,
            "GSCN_REC",
            "no GSCN candidates (ret=%d n=%zu); check RECORD_GSCN_* / PROFILE",
            ret,
            n_cand);
    goto out;
  }

  idx_per_ssb = !tune_oai_center;

  log_msg(LOG_LEVEL_INFO,
          "GSCN_REC",
          "candidates=%zu profile=%d tune=%s (OAI: passband-visible GSCN per RX LO) stride=%u blocks=%d "
          "warmup=%u overlap=%.2f preset=%s rounds=%s",
          n_cand,
          (int)profile,
          tune_oai_center ? "oai_center" : "per_ssb",
          stride,
          num_blocks,
          warmup_blocks,
          overlap_frac,
          (preset != NULL) ? preset : "-",
          infinite_rounds ? "inf" : "finite");

  ret = radio_device_init(&device, RADIO_DEVICE_LIME);
  if (ret != TOA_OK) {
    log_msg(LOG_LEVEL_ERROR, "GSCN_REC", "radio_device_init failed, ret=%d", ret);
    goto out;
  }

  ret = device.dev_open(&device, "driver=lime");
  if (ret != TOA_OK) {
    log_msg(LOG_LEVEL_ERROR, "GSCN_REC", "dev_open failed, ret=%d", ret);
    goto out;
  }

  caps_ok = (device.dev_get_caps && device.dev_get_caps(&device, &caps) == TOA_OK);
  if (caps_ok) {
    log_msg(LOG_LEVEL_INFO,
            "GSCN_REC",
            "caps fs=[%.0f, %.0f] fc=[%.0f, %.0f] gain=[%.1f, %.1f]",
            caps.sample_rate_min,
            caps.sample_rate_max,
            caps.center_freq_min,
            caps.center_freq_max,
            caps.gain_min,
            caps.gain_max);
  }

  memset(&rx_cfg, 0, sizeof(rx_cfg));
  rx_cfg.channel_id = 0;
  rx_cfg.sample_rate = DEMO_RX_SAMPLE_RATE_HZ;
  rx_cfg.center_freq = DEMO_RX_CENTER_FREQ_HZ;
  rx_cfg.bandwidth = DEMO_RX_BANDWIDTH_HZ;
  rx_cfg.gain = DEMO_RX_GAIN_DB;
  (void)cli_env_f64("RECORD_FS", &rx_cfg.sample_rate);
  (void)cli_env_f64("RECORD_BW", &rx_cfg.bandwidth);
  (void)cli_env_f64("RECORD_GAIN_DB", &rx_cfg.gain);
  if (caps_ok)
    clamp_rx_to_caps(&rx_cfg, &caps);

  snprintf(rx_cfg.antenna, sizeof(rx_cfg.antenna), "LNAH");
  if (getenv("RECORD_ANTENNA"))
    snprintf(rx_cfg.antenna, sizeof(rx_cfg.antenna), "%s", getenv("RECORD_ANTENNA"));

  memset(&rx_stream_cfg, 0, sizeof(rx_stream_cfg));
  rx_stream_cfg.channel_id = 0;
  rx_stream_cfg.buffer_size = DEMO_RX_BUFFER_SAMPLES;

  if (tune_oai_center) {
    double cover_lo = fmin_mhz * 1e6;
    double cover_hi = fmax_mhz * 1e6;

    if (use_gscn_range) {
      /* unreachable */
      cover_lo = cover_hi;
    }
    ret = (int)nr_gscn_tile_centers_hz(cover_lo,
                                      cover_hi,
                                      rx_cfg.bandwidth,
                                      overlap_frac,
                                      centers,
                                      sizeof(centers) / sizeof(centers[0]),
                                      &n_cent);
    if (ret != TOA_OK || n_cent == 0) {
      log_msg(LOG_LEVEL_ERROR, "GSCN_REC", "tile centers failed ret=%d n=%zu", ret, n_cent);
      goto out;
    }
    rx_cfg.center_freq = centers[0];
    log_msg(LOG_LEVEL_INFO,
            "GSCN_REC",
            "OAI-style: %zu RX center tiles over %.3f - %.3f MHz",
            n_cent,
            cover_lo * 1e-6,
            cover_hi * 1e-6);
  }

  ret = device.trx_configure_rx(&device, &rx_cfg);
  if (ret != TOA_OK) {
    log_msg(LOG_LEVEL_ERROR, "GSCN_REC", "trx_configure_rx (initial) failed, ret=%d", ret);
    goto out;
  }

  ret = device.trx_setup_rx_stream(&device, &rx_stream_cfg);
  if (ret != TOA_OK) {
    log_msg(LOG_LEVEL_ERROR, "GSCN_REC", "trx_setup_rx_stream failed, ret=%d", ret);
    goto out;
  }

  ret = device.trx_start_rx_stream(&device);
  if (ret != TOA_OK) {
    log_msg(LOG_LEVEL_ERROR, "GSCN_REC", "trx_start_rx_stream failed, ret=%d", ret);
    goto out;
  }

  rx_buffer = (cf32_t *)malloc(sizeof(cf32_t) * rx_stream_cfg.buffer_size);
  if (!rx_buffer) {
    log_msg(LOG_LEVEL_ERROR, "GSCN_REC", "malloc rx_buffer failed");
    goto out;
  }
  buffs[0] = rx_buffer;

  idx_fp = fopen(index_path, "w");
  if (!idx_fp) {
    log_msg(LOG_LEVEL_WARN, "GSCN_REC", "cannot open index %s (continuing)", index_path);
  } else if (idx_per_ssb) {
    fprintf(idx_fp, "mode,gscn,ss_ref_mhz,fc_mhz,samples,base_path\n");
  } else {
    fprintf(idx_fp, "mode,center_mhz,n_visible,gscn_min,gscn_max,samples,base_path\n");
  }

  for (uint32_t round = 0; infinite_rounds || round < rounds; round++) {
    if (tune_oai_center) {
      size_t ci;

      for (ci = 0; ci < n_cent; ci++) {
        double fc_hz = centers[ci];
        size_t nv = 0;
        char base[512];
        radio_rx_result_t rx_result;

        rx_cfg.center_freq = fc_hz;
        if (caps_ok)
          clamp_rx_to_caps(&rx_cfg, &caps);

        ret = (int)nr_gscn_visible_in_passband(fc_hz,
                                              rx_cfg.bandwidth,
                                              cand,
                                              n_cand,
                                              vis,
                                              sizeof(vis) / sizeof(vis[0]),
                                              &nv);
        if (ret != TOA_OK || nv == 0) {
          log_msg(LOG_LEVEL_WARN,
                  "GSCN_REC",
                  "center %.3f MHz: no GSCN in passband (skip)",
                  fc_hz * 1e-6);
          continue;
        }

        ret = device.trx_configure_rx(&device, &rx_cfg);
        if (ret != TOA_OK) {
          log_msg(LOG_LEVEL_WARN, "GSCN_REC", "configure center %.3f MHz failed ret=%d", fc_hz * 1e-6, ret);
          continue;
        }

        if (settle_ms > 0u)
          usleep((useconds_t)(settle_ms * 1000u));

        if (warmup_blocks > 0u) {
          ret = discard_blocks(&device, buffs, rx_stream_cfg.buffer_size, warmup_blocks);
          if (ret != 0) {
            log_msg(LOG_LEVEL_WARN, "GSCN_REC", "warmup failed at center %.3f MHz", fc_hz * 1e-6);
            continue;
          }
        }

        if (out_dir && out_dir[0] != '\0')
          snprintf(base,
                   sizeof(base),
                   "%s/%s_oai_%.3fMHz",
                   out_dir,
                   prefix,
                   fc_hz * 1e-6);
        else
          snprintf(base, sizeof(base), "%s_oai_%.3fMHz", prefix, fc_hz * 1e-6);

        if (capture_session_open(&session, base) != TOA_OK) {
          log_msg(LOG_LEVEL_ERROR, "GSCN_REC", "capture_session_open failed: %s", base);
          goto out;
        }
        session_opened = 1;

        iq_meta_init_default(&meta);
        meta.sample_rate_hz = rx_cfg.sample_rate;
        meta.center_freq_hz = rx_cfg.center_freq;
        meta.bandwidth_hz = rx_cfg.bandwidth;
        meta.gain_db = rx_cfg.gain;
        meta.block_size = rx_stream_cfg.buffer_size;
        if (device.get_hw_time_ns)
          meta.capture_start_time_ns = device.get_hw_time_ns(&device);
        snprintf(meta.device_args, sizeof(meta.device_args), "%s", "driver=lime");
        capture_session_set_meta(&session, &meta);

        if (write_visible_gscn_sidecar(base, vis, nv) != 0)
          log_msg(LOG_LEVEL_WARN, "GSCN_REC", "write %s.visible_gscn.txt failed", base);

        log_msg(LOG_LEVEL_INFO,
                "GSCN_REC",
                "round=%u OAI center=%.3f MHz visible_gscn=%zu (min=%u max=%u) -> %s",
                round,
                fc_hz * 1e-6,
                nv,
                gscn_min_arr(vis, nv),
                gscn_max_arr(vis, nv),
                base);

        for (int i = 0; i < num_blocks; i++) {
          memset(&rx_result, 0, sizeof(rx_result));
          ret = device.trx_read_func(&device,
                                     buffs,
                                     rx_stream_cfg.buffer_size,
                                     &rx_result,
                                     100000);
          if (ret != 0) {
            log_msg(LOG_LEVEL_WARN, "GSCN_REC", "read failed center tile block=%d ret=%d", i, ret);
            break;
          }
          if (rx_result.samples_read > 0) {
            if (capture_session_write_block(&session,
                                            rx_buffer,
                                            (uint32_t)rx_result.samples_read,
                                            rx_result.timestamp_ns,
                                            0) != TOA_OK) {
              log_msg(LOG_LEVEL_WARN, "GSCN_REC", "write failed block=%d", i);
              break;
            }
          }
        }

        if (idx_fp) {
          fprintf(idx_fp,
                  "oai_center,%.6f,%zu,%u,%u,%llu,%s\n",
                  fc_hz * 1e-6,
                  nv,
                  gscn_min_arr(vis, nv),
                  gscn_max_arr(vis, nv),
                  (unsigned long long)session.iq.total_samples,
                  base);
        }

        capture_session_close(&session);
        session_opened = 0;
      }
    } else {
      size_t k;

      for (k = 0; k < n_cand; k++) {
        uint32_t g = cand[k];
        double ss_hz = 0.0;
        char base[512];
        radio_rx_result_t rx_result;

        if (nr_gscn_fr1_lt_3ghz_to_ss_ref_hz(g, &ss_hz) != TOA_OK)
          continue;

        rx_cfg.center_freq = ss_hz;
        if (caps_ok)
          clamp_rx_to_caps(&rx_cfg, &caps);

        ret = device.trx_configure_rx(&device, &rx_cfg);
        if (ret != TOA_OK) {
          log_msg(LOG_LEVEL_WARN, "GSCN_REC", "GSCN %u configure failed ret=%d (skip)", g, ret);
          continue;
        }

        if (settle_ms > 0u)
          usleep((useconds_t)(settle_ms * 1000u));

        if (warmup_blocks > 0u) {
          ret = discard_blocks(&device, buffs, rx_stream_cfg.buffer_size, warmup_blocks);
          if (ret != 0) {
            log_msg(LOG_LEVEL_WARN, "GSCN_REC", "GSCN %u warmup read failed ret=%d", g, ret);
            continue;
          }
        }

        if (out_dir && out_dir[0] != '\0')
          snprintf(base, sizeof(base), "%s/%s_%u", out_dir, prefix, g);
        else
          snprintf(base, sizeof(base), "%s_%u", prefix, g);

        if (capture_session_open(&session, base) != TOA_OK) {
          log_msg(LOG_LEVEL_ERROR, "GSCN_REC", "capture_session_open failed: %s", base);
          goto out;
        }
        session_opened = 1;

        iq_meta_init_default(&meta);
        meta.sample_rate_hz = rx_cfg.sample_rate;
        meta.center_freq_hz = rx_cfg.center_freq;
        meta.bandwidth_hz = rx_cfg.bandwidth;
        meta.gain_db = rx_cfg.gain;
        meta.block_size = rx_stream_cfg.buffer_size;
        if (device.get_hw_time_ns)
          meta.capture_start_time_ns = device.get_hw_time_ns(&device);
        snprintf(meta.device_args, sizeof(meta.device_args), "%s", "driver=lime");
        capture_session_set_meta(&session, &meta);

        log_msg(LOG_LEVEL_INFO,
                "GSCN_REC",
                "round=%u GSCN=%u ss_ref=%.3f MHz -> fc=%.3f MHz session=%s",
                round,
                g,
                ss_hz * 1e-6,
                rx_cfg.center_freq * 1e-6,
                base);

        for (int i = 0; i < num_blocks; i++) {
          memset(&rx_result, 0, sizeof(rx_result));
          ret = device.trx_read_func(&device,
                                     buffs,
                                     rx_stream_cfg.buffer_size,
                                     &rx_result,
                                     100000);
          if (ret != 0) {
            log_msg(LOG_LEVEL_WARN, "GSCN_REC", "read failed GSCN=%u block=%d ret=%d", g, i, ret);
            break;
          }
          if (rx_result.samples_read > 0) {
            if (capture_session_write_block(&session,
                                            rx_buffer,
                                            (uint32_t)rx_result.samples_read,
                                            rx_result.timestamp_ns,
                                            0) != TOA_OK) {
              log_msg(LOG_LEVEL_WARN, "GSCN_REC", "write failed GSCN=%u block=%d", g, i);
              break;
            }
          }
        }

        if (idx_fp)
          fprintf(idx_fp,
                  "ssb,%u,%.6f,%.6f,%llu,%s\n",
                  g,
                  ss_hz * 1e-6,
                  rx_cfg.center_freq * 1e-6,
                  (unsigned long long)session.iq.total_samples,
                  base);

        capture_session_close(&session);
        session_opened = 0;
      }
    }
  }

  rc = 0;

out:
  if (session_opened)
    capture_session_close(&session);
  if (idx_fp)
    fclose(idx_fp);
  free(rx_buffer);
  if (device.rx_running)
    (void)device.trx_stop_rx_stream(&device);
  if (device.is_open)
    device.dev_close(&device);
  return rc;
}
