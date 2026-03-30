#include "nr/ssb_sync.h"
#include "nr/nr_grid.h"
#include "nr/pbch_dmrs.h"
#include "nr/pbch_decode.h"
#include "nr/mib.h"
#include "common/cli.h"
#include "common/demo_rx_defaults.h"
#include "common/error.h"
#include "common/log.h"
#include "io/iq_reader.h"
#include "pipeline/offline_runner.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct {
  nr_ssb_sync_ctx_t *ssb_ctx;
  FILE *fp_csv;
  uint32_t fft_size;
  uint32_t cp_len;
  uint32_t num_sym;
  uint32_t i_ssb;
  uint32_t align_enable;
  uint32_t align_half_window;
  nr_pbch_decode_cfg_t pbch_cfg;
  cf32_t fft_syms[4 * 1024];
  cf32_t fft_syms_best[4 * 1024];
  uint64_t crc_ok_cnt;
  uint64_t total_frames;
} mib_ud_t;

static toa_error_t mib_decode_cb(void *userdata,
                                 const cf32_t *samples,
                                 uint32_t nsamps,
                                 const iq_block_info_t *blk)
{
  mib_ud_t *u = (mib_ud_t *)userdata;
  nr_ssb_result_t ssb_res;
  nr_pbch_dmrs_result_t dmrs_res;
  nr_pbch_decode_result_t pbch_res;
  nr_mib_t mib;

  memset(&ssb_res, 0, sizeof(ssb_res));
  memset(&dmrs_res, 0, sizeof(dmrs_res));
  memset(&pbch_res, 0, sizeof(pbch_res));
  memset(&mib, 0, sizeof(mib));

  nr_ssb_sync_reset(u->ssb_ctx);
  (void)nr_ssb_sync_scan_block(u->ssb_ctx, samples, nsamps, blk->hw_time_ns0, &ssb_res);
  if (ssb_res.found) {
    uint32_t stride = u->fft_size + u->cp_len;
    uint32_t pci_u = (ssb_res.pci >= 0) ? (uint32_t)ssb_res.pci : 0u;
    uint32_t ok_syms = 0;
    int best_delta = 0;
    float best_score = -1.0f;
    uint32_t best_ok_syms = 0;
    nr_pbch_dmrs_result_t best_dmrs_res;

    memset(&best_dmrs_res, 0, sizeof(best_dmrs_res));
    memset(u->fft_syms_best, 0, sizeof(u->fft_syms_best));

    if (u->align_enable && u->align_half_window > 0) {
      for (int d = -(int)u->align_half_window; d <= (int)u->align_half_window; ++d) {
        int64_t base_i = (int64_t)ssb_res.peak_pos + (int64_t)d;
        if (base_i < 0)
          continue;
        uint32_t base = (uint32_t)base_i;
        uint32_t ok_tmp = 0;

        for (uint32_t s = 0; s < u->num_sym; ++s) {
          uint32_t spos = base + s * stride;
          if ((spos + u->cp_len + u->fft_size) > nsamps)
            break;
          if (nr_extract_symbol_fft(samples,
                                    nsamps,
                                    spos,
                                    u->cp_len,
                                    u->fft_size,
                                    &u->fft_syms[s * u->fft_size]) == TOA_OK)
            ok_tmp++;
        }
        if (ok_tmp == 0)
          continue;

        nr_pbch_dmrs_result_t tmp_dmrs;
        memset(&tmp_dmrs, 0, sizeof(tmp_dmrs));
        (void)nr_pbch_dmrs_correlate(u->fft_syms,
                                     ok_tmp,
                                     u->fft_size,
                                     pci_u,
                                     u->i_ssb,
                                     &tmp_dmrs);
        if (!tmp_dmrs.valid)
          continue;

        {
          float score = tmp_dmrs.metric * tmp_dmrs.peak_ratio;
          if (score > best_score) {
            best_score = score;
            best_delta = d;
            best_ok_syms = ok_tmp;
            best_dmrs_res = tmp_dmrs;
            memcpy(u->fft_syms_best, u->fft_syms, sizeof(u->fft_syms_best));
          }
        }
      }

      if (best_ok_syms > 0) {
        ok_syms = best_ok_syms;
        dmrs_res = best_dmrs_res;
        memcpy(u->fft_syms, u->fft_syms_best, sizeof(u->fft_syms_best));
      }
      (void)best_delta;
    } else {
      for (uint32_t s = 0; s < u->num_sym; ++s) {
        uint32_t spos = ssb_res.peak_pos + s * stride;
        if ((spos + u->cp_len + u->fft_size) > nsamps)
          break;
        if (nr_extract_symbol_fft(samples,
                                  nsamps,
                                  spos,
                                  u->cp_len,
                                  u->fft_size,
                                  &u->fft_syms[s * u->fft_size]) == TOA_OK)
          ok_syms++;
      }
      if (ok_syms > 0) {
        (void)nr_pbch_dmrs_correlate(u->fft_syms,
                                     ok_syms,
                                     u->fft_size,
                                     pci_u,
                                     u->i_ssb,
                                     &dmrs_res);
      }
    }
  }

  (void)nr_pbch_decode_from_dmrs(&ssb_res,
                                 u->fft_syms,
                                 u->num_sym,
                                 u->fft_size,
                                 dmrs_res.metric,
                                 dmrs_res.peak_ratio,
                                 dmrs_res.re_count,
                                 u->i_ssb,
                                 &u->pbch_cfg,
                                 &pbch_res);
  (void)nr_mib_unpack(&pbch_res, &mib);

  if (pbch_res.crc_ok)
    u->crc_ok_cnt++;

  fprintf(u->fp_csv, "%llu,%d,%d,%.6f,%.6f,%d,%d,%u,%u\n",
          (unsigned long long)blk->block_index,
          ssb_res.found,
          ssb_res.pci,
          dmrs_res.metric,
          dmrs_res.peak_ratio,
          pbch_res.crc_ok,
          mib.valid,
          mib.sfn,
          mib.scs_common);

  u->total_frames = blk->block_index + 1u;
  if (((blk->block_index + 1u) % 100u) == 0u)
    log_msg(LOG_LEVEL_INFO, "MIB", "frame=%llu crc_ok=%llu",
            (unsigned long long)u->total_frames,
            (unsigned long long)u->crc_ok_cnt);

  return TOA_OK;
}

int main(void)
{
  const char *iq_path = "rx_iq.cf32";
  const char *csv_path = "nr_mib_decode.csv";
  const char *meta_path = NULL;
  const char *blocklog_path = NULL;
  FILE *fp_csv = NULL;
  nr_ssb_cfg_t ssb_cfg;
  nr_ssb_sync_ctx_t ssb_ctx;
  mib_ud_t ud;
  int fs_env = 0;
  toa_error_t tr;
  int rc = -1;

  memset(&ssb_cfg, 0, sizeof(ssb_cfg));
  ssb_cfg.sample_rate_hz = DEMO_RX_SAMPLE_RATE_HZ;
  ssb_cfg.search_step = 2;
  ssb_cfg.track_half_window = 256;
  ssb_cfg.min_metric = DEMO_NR_MIN_PSS_METRIC;
  ssb_cfg.min_pss_peak_ratio = DEMO_NR_MIN_PSS_PEAK_RATIO;
  ssb_cfg.sss_enable = 1;
  ssb_cfg.sss_lag_samples = DEMO_NR_SSS_LAG_SAMPLES;
  ssb_cfg.sss_search_half_window = DEMO_NR_SSS_SEARCH_HALF_WIN;
  ssb_cfg.min_sss_metric = DEMO_NR_MIN_SSS_METRIC;
  ssb_cfg.min_sss_peak_ratio = DEMO_NR_MIN_SSS_PEAK_RATIO;
  ssb_cfg.sss_derotate_cfo = 1;
  ssb_cfg.sss_derotate_min_cfo_hz = DEMO_NR_SSS_DEROTATE_MIN_CFO_HZ;

  if (getenv("NR_IQ_PATH"))
    iq_path = getenv("NR_IQ_PATH");
  if (getenv("NR_CSV_PATH"))
    csv_path = getenv("NR_CSV_PATH");
  if (getenv("NR_IQ_META_PATH"))
    meta_path = getenv("NR_IQ_META_PATH");
  if (getenv("NR_IQ_BLOCKLOG_PATH"))
    blocklog_path = getenv("NR_IQ_BLOCKLOG_PATH");

  fs_env = (cli_env_f64("NR_FS", &ssb_cfg.sample_rate_hz) == 0);
  (void)iq_replay_fill_fs_from_meta(iq_path, meta_path, fs_env, &ssb_cfg.sample_rate_hz);

  {
    uint32_t v = 0;
    if (cli_env_u32("NR_SSS_ENABLE", &v) == 0)
      ssb_cfg.sss_enable = (v != 0);
  }
  (void)cli_env_u32("NR_SSS_SEARCH_HALF_WINDOW", &ssb_cfg.sss_search_half_window);
  {
    double v = 0.0;
    if (cli_env_f64("NR_MIN_METRIC", &v) == 0)
      ssb_cfg.min_metric = (float)v;
  }
  {
    double v = 0.0;
    if (cli_env_f64("NR_MIN_SSS_METRIC", &v) == 0)
      ssb_cfg.min_sss_metric = (float)v;
  }
  {
    double v = 0.0;
    if (cli_env_f64("NR_MIN_PSS_PEAK_RATIO", &v) == 0)
      ssb_cfg.min_pss_peak_ratio = (float)v;
  }
  {
    double v = 0.0;
    if (cli_env_f64("NR_MIN_SSS_PEAK_RATIO", &v) == 0)
      ssb_cfg.min_sss_peak_ratio = (float)v;
  }
  {
    uint32_t v = 1;
    if (cli_env_u32("NR_SSS_DEROTATE", &v) == 0)
      ssb_cfg.sss_derotate_cfo = (v != 0);
  }
  {
    double v = 0.0;
    if (cli_env_f64("NR_SSS_DEROTATE_MIN_CFO_HZ", &v) == 0)
      ssb_cfg.sss_derotate_min_cfo_hz = (float)v;
  }

  memset(&ud, 0, sizeof(ud));
  ud.fft_size = DEMO_NR_FFT_SIZE;
  ud.cp_len = DEMO_NR_CP_LEN;
  ud.num_sym = 4;
  ud.i_ssb = 0;
  ud.align_enable = 0;
  ud.align_half_window = 4;

  (void)cli_env_u32("NR_FFT_SIZE", &ud.fft_size);
  (void)cli_env_u32("NR_CP_LEN", &ud.cp_len);
  (void)cli_env_u32("NR_NUM_SYM", &ud.num_sym);
  (void)cli_env_u32("NR_I_SSB", &ud.i_ssb);
  {
    uint32_t v = 0;
    if (cli_env_u32("NR_ALIGN_ENABLE", &v) == 0)
      ud.align_enable = v;
  }
  {
    uint32_t v = 0;
    if (cli_env_u32("NR_ALIGN_HALF_WINDOW", &v) == 0)
      ud.align_half_window = v;
  }

  nr_pbch_decode_cfg_default(&ud.pbch_cfg);
  {
    uint32_t v = 0;
    if (cli_env_u32("NR_POLAR_BITREV_ENABLE", &v) == 0)
      ud.pbch_cfg.enable_polar_bitrev = (int)v;
  }
  {
    uint32_t v = 0;
    if (cli_env_u32("NR_PBCH_DESCRAMBLE_ENABLE", &v) == 0)
      ud.pbch_cfg.enable_pbch_descramble = (int)v;
  }
  {
    uint32_t v = 0;
    if (cli_env_u32("NR_POLAR_INFO_BITREV_ENABLE", &v) == 0)
      ud.pbch_cfg.enable_polar_info_bitrev = (int)v;
  }
  {
    uint32_t v = 0;
    if (cli_env_u32("NR_CRC_BIT_ORDER", &v) == 0)
      ud.pbch_cfg.crc_bit_order = (int)v;
  }

  if (ud.fft_size > 1024)
    ud.fft_size = 1024;
  if (ud.num_sym == 0)
    ud.num_sym = 1;
  if (ud.num_sym > 4)
    ud.num_sym = 4;

  {
    double lag_v = 0.0;
    if (cli_env_f64("NR_SSS_LAG", &lag_v) == 0)
      ssb_cfg.sss_lag_samples = (int32_t)lag_v;
    else
      ssb_cfg.sss_lag_samples =
          (int32_t)(2 * ((int32_t)ud.fft_size + (int32_t)ud.cp_len));
  }

  log_set_level(LOG_LEVEL_INFO);
  log_msg(LOG_LEVEL_INFO,
          "MIB",
          "OTA 30kHz grid: fs=%.3f MHz scs=%.0f Hz N_FFT=%u CP=%u sss_lag=%d",
          ssb_cfg.sample_rate_hz * 1e-6,
          DEMO_NR_SCS_HZ,
          (unsigned)ud.fft_size,
          (unsigned)ud.cp_len,
          (int)ssb_cfg.sss_lag_samples);
  log_msg(LOG_LEVEL_INFO, "MIB", "Sprint3 proto: LLR + proto rate-recovery + Polar-SC + CRC24C");

  if (nr_ssb_sync_init(&ssb_ctx, &ssb_cfg) != TOA_OK) {
    log_msg(LOG_LEVEL_ERROR, "MIB", "ssb init failed");
    goto out;
  }

  fp_csv = fopen(csv_path, "w");
  if (!fp_csv) {
    log_msg(LOG_LEVEL_ERROR, "MIB", "open csv failed: %s", csv_path);
    goto out;
  }

  fprintf(fp_csv, "frame_id,ssb_found,pci,dmrs_metric,dmrs_peak_ratio,mib_crc_ok,mib_valid,sfn,scs_common\n");

  ud.ssb_ctx = &ssb_ctx;
  ud.fp_csv = fp_csv;

  tr = offline_run_cf32_file(iq_path,
                             meta_path,
                             blocklog_path,
                             DEMO_RX_BUFFER_SAMPLES,
                             ssb_cfg.sample_rate_hz,
                             &ud,
                             mib_decode_cb);
  if (tr != TOA_OK) {
    log_msg(LOG_LEVEL_ERROR, "MIB", "offline_run_cf32_file failed: %d", tr);
    goto out;
  }

  log_msg(LOG_LEVEL_INFO, "MIB", "done frames=%llu crc_ok=%llu ratio=%.2f%% csv=%s",
          (unsigned long long)ud.total_frames,
          (unsigned long long)ud.crc_ok_cnt,
          ud.total_frames ? (100.0 * (double)ud.crc_ok_cnt / (double)ud.total_frames) : 0.0,
          csv_path);
  rc = 0;
out:
  if (fp_csv)
    fclose(fp_csv);
  return rc;
}
