#include "openair1/PHY/NR_POSITIONING/nr_pos_types.h"
#include "radio/COMMON/common_lib.h"

#include <stdio.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>

static void nr_trim(char *s)
{
  if (!s) {
    return;
  }
  /* Trim leading spaces */
  char *p = s;
  while (*p && isspace((unsigned char)*p)) {
    p++;
  }
  if (p != s) {
    memmove(s, p, strlen(p) + 1);
  }
  /* Trim trailing spaces */
  size_t n = strlen(s);
  while (n > 0 && isspace((unsigned char)s[n - 1])) {
    s[n - 1] = '\0';
    n--;
  }
}

static int nr_streq_icase(const char *a, const char *b)
{
  if (!a || !b) {
    return 0;
  }
  while (*a && *b) {
    if (tolower((unsigned char)*a) != tolower((unsigned char)*b)) {
      return 0;
    }
    a++;
    b++;
  }
  return (*a == '\0' && *b == '\0');
}

static int nr_parse_u32(const char *s, uint32_t *out)
{
  if (!s || !out) {
    return -1;
  }
  char *end = NULL;
  unsigned long v = strtoul(s, &end, 0);
  if (!end || end == s) {
    return -1;
  }
  *out = (uint32_t)v;
  return 0;
}

int nr_toa_load_config(const char *path, nr_toa_app_cfg_t *cfg)
{
  if (!path || !cfg) {
    return -1;
  }
  memset(cfg, 0, sizeof(*cfg));

  /* Defaults (override by conf). */
  (void)snprintf(cfg->sdr, sizeof(cfg->sdr), "lime");
  cfg->mode = NR_TOA_MODE_SSB_TOA;
  cfg->meas_mode = NR_MEAS_MODE_MEAS_ONLY;
  (void)snprintf(cfg->clock_source, sizeof(cfg->clock_source), "internal");
  (void)snprintf(cfg->time_source, sizeof(cfg->time_source), "internal");
  cfg->rx_gain_db = 0.0;
  cfg->tx_gain_db = 0.0;
  cfg->center_freq_hz = 0.0;
  cfg->sample_rate_hz = 0.0;
  cfg->ssb_period_ms = 20;
  cfg->rx_ant = 1;
  cfg->tx_ant = 1;
  cfg->trace_enable = 0;
  cfg->iq_dump_enable = 0;
  cfg->full_band_sweep = 1;
  cfg->strict_center_freq = 0;
  cfg->gain_sweep_enable = 1;
  cfg->target_pci = -1;
  cfg->ssb_scs_khz = 0U;

  FILE *fp = fopen(path, "r");
  if (!fp) {
    return -1;
  }

  char line[1024];
  while (fgets(line, sizeof(line), fp)) {
    /* Strip newline */
    line[strcspn(line, "\r\n")] = '\0';

    /* Skip leading whitespace */
    nr_trim(line);
    if (line[0] == '\0') {
      continue;
    }
    if (line[0] == '#') {
      continue;
    }

    char *eq = strchr(line, '=');
    if (!eq) {
      continue;
    }
    *eq = '\0';
    char *key = line;
    char *val = eq + 1;
    nr_trim(key);
    nr_trim(val);
    if (key[0] == '\0' || val[0] == '\0') {
      continue;
    }

    if (nr_streq_icase(key, "sdr")) {
      (void)snprintf(cfg->sdr, sizeof(cfg->sdr), "%s", val);
    } else if (nr_streq_icase(key, "sdr_addrs")) {
      (void)snprintf(cfg->sdr_addrs, sizeof(cfg->sdr_addrs), "%s", val);
    } else if (nr_streq_icase(key, "clock_source")) {
      (void)snprintf(cfg->clock_source, sizeof(cfg->clock_source), "%s", val);
    } else if (nr_streq_icase(key, "time_source")) {
      (void)snprintf(cfg->time_source, sizeof(cfg->time_source), "%s", val);
    } else if (nr_streq_icase(key, "center_freq_hz")) {
      cfg->center_freq_hz = strtod(val, NULL);
    } else if (nr_streq_icase(key, "sample_rate_hz")) {
      cfg->sample_rate_hz = strtod(val, NULL);
    } else if (nr_streq_icase(key, "rx_gain_db")) {
      cfg->rx_gain_db = strtod(val, NULL);
    } else if (nr_streq_icase(key, "tx_gain_db")) {
      cfg->tx_gain_db = strtod(val, NULL);
    } else if (nr_streq_icase(key, "mode")) {
      if (nr_streq_icase(val, "PRS_TOA")) {
        cfg->mode = NR_TOA_MODE_PRS_TOA;
      } else {
        cfg->mode = NR_TOA_MODE_SSB_TOA;
      }
    } else if (nr_streq_icase(key, "meas_mode")) {
      if (nr_streq_icase(val, "PSEUDORANGE_ONLY")) {
        cfg->meas_mode = NR_MEAS_MODE_PSEUDORANGE_ONLY;
      } else if (nr_streq_icase(val, "POSITION_SOLVE")) {
        cfg->meas_mode = NR_MEAS_MODE_POSITION_SOLVE;
      } else {
        cfg->meas_mode = NR_MEAS_MODE_MEAS_ONLY;
      }
    } else if (nr_streq_icase(key, "anchor_db_path")) {
      (void)snprintf(cfg->anchor_db_path, sizeof(cfg->anchor_db_path), "%s", val);
    } else if (nr_streq_icase(key, "trace_enable")) {
      uint32_t v = 0;
      if (nr_parse_u32(val, &v) == 0) {
        cfg->trace_enable = (uint8_t)v;
      }
    } else if (nr_streq_icase(key, "iq_dump_enable")) {
      uint32_t v = 0;
      if (nr_parse_u32(val, &v) == 0) {
        cfg->iq_dump_enable = (uint8_t)v;
      }
    } else if (nr_streq_icase(key, "full_band_sweep")) {
      uint32_t v = 0;
      if (nr_parse_u32(val, &v) == 0) {
        cfg->full_band_sweep = (uint8_t)(v ? 1U : 0U);
      }
    } else if (nr_streq_icase(key, "strict_center_freq")) {
      uint32_t v = 0;
      if (nr_parse_u32(val, &v) == 0) {
        cfg->strict_center_freq = (uint8_t)(v ? 1U : 0U);
      }
    } else if (nr_streq_icase(key, "gain_sweep_enable")) {
      uint32_t v = 0;
      if (nr_parse_u32(val, &v) == 0) {
        cfg->gain_sweep_enable = (uint8_t)(v ? 1U : 0U);
      }
    } else if (nr_streq_icase(key, "target_pci")) {
      uint32_t v = 0;
      if (nr_parse_u32(val, &v) == 0 && v < 1008U) {
        cfg->target_pci = (int32_t)v;
      }
    } else if (nr_streq_icase(key, "ssb_scs_khz")) {
      uint32_t v = 0;
      if (nr_parse_u32(val, &v) == 0) {
        cfg->ssb_scs_khz = v;
      }
    } else if (nr_streq_icase(key, "ssb_period_ms")) {
      uint32_t v = 0;
      if (nr_parse_u32(val, &v) == 0) {
        cfg->ssb_period_ms = v;
      }
    } else if (nr_streq_icase(key, "rx_ant")) {
      uint32_t v = 0;
      if (nr_parse_u32(val, &v) == 0) {
        cfg->rx_ant = v;
      }
    } else if (nr_streq_icase(key, "tx_ant")) {
      uint32_t v = 0;
      if (nr_parse_u32(val, &v) == 0) {
        cfg->tx_ant = v;
      }
    } else {
      /* Unknown key: ignore for Phase-0. */
    }
  }

  fclose(fp);
  return 0;
}

int nr_toa_build_rf_cfg(const nr_toa_app_cfg_t *cfg, openair0_config_t *rf_cfg)
{
  if (!cfg || !rf_cfg) {
    return -1;
  }
  memset(rf_cfg, 0, sizeof(*rf_cfg));
  if (strcmp(cfg->sdr, "usrp") == 0) {
    rf_cfg->device_type = USRP_B200_DEV;
  } else {
    rf_cfg->device_type = LMSSDR_DEV;
  }
  rf_cfg->sample_rate = cfg->sample_rate_hz;
  rf_cfg->rx_freq_hz = cfg->center_freq_hz;
  rf_cfg->tx_freq_hz = cfg->center_freq_hz;
  rf_cfg->rx_gain_db = cfg->rx_gain_db;
  rf_cfg->tx_gain_db = cfg->tx_gain_db;

  rf_cfg->sdr_addrs = cfg->sdr_addrs;
  rf_cfg->clock_source = cfg->clock_source;
  rf_cfg->time_source = cfg->time_source;
  return 0;
}
