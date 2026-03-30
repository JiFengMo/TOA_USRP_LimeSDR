#include "toa/toa_context.h"
#include "toa/toa_pipeline.h"
#include "common/cli.h"
#include "common/error.h"
#include "common/log.h"
#include "io/iq_reader.h"
#include "pipeline/offline_runner.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct {
  toa_pipeline_t *pl;
  toa_context_t *ctx;
  uint64_t total_obs;
  uint64_t total_samples;
} toa_offline_ud_t;

static toa_error_t toa_offline_cb(void *userdata,
                                  const cf32_t *samples,
                                  uint32_t nsamps,
                                  const iq_block_info_t *blk)
{
  toa_offline_ud_t *u = (toa_offline_ud_t *)userdata;
  toa_frame_t frame;
  toa_obs_t obs[8];
  uint32_t obs_num = 0;
  int ret;

  obs_num = 0;
  frame.samples = samples;
  frame.frame_id = blk->block_index;
  frame.sample_count = nsamps;
  frame.sample_rate_hz = u->ctx->sample_rate_hz;
  frame.center_freq_hz = u->ctx->center_freq_hz;
  frame.rx_gain_db = 0.0;
  frame.hw_timestamp_ns = blk->hw_time_ns0;

  ret = toa_pipeline_process_frame(u->pl, &frame, obs, 8, &obs_num);
  if (ret != TOA_OK) {
    log_msg(LOG_LEVEL_ERROR, "TOA_OFFLINE", "process_frame failed frame=%llu ret=%d",
            (unsigned long long)blk->block_index, ret);
    return TOA_ERR_IO;
  }

  u->total_obs += obs_num;
  u->total_samples += nsamps;

  if (((blk->block_index + 1u) % 100u) == 0u) {
    log_msg(LOG_LEVEL_INFO,
            "TOA_OFFLINE",
            "processed frame=%llu total_obs=%llu",
            (unsigned long long)(blk->block_index + 1u),
            (unsigned long long)u->total_obs);
    if (obs_num > 0) {
      log_msg(LOG_LEVEL_INFO,
              "TOA_OFFLINE",
              "obs0 toa_ns=%lld snr_db=%.2f conf=%.3f",
              (long long)obs[0].toa_ns,
              obs[0].snr_db,
              obs[0].confidence);
    }
  }

  return TOA_OK;
}

int main(void)
{
  const char *iq_path = "rx_iq.cf32";
  const char *meta_path = NULL;
  const char *blocklog_path = NULL;
  toa_context_t ctx;
  toa_pipeline_t pl;
  toa_offline_ud_t ud;
  int fs_env = 0;
  toa_error_t tr;
  int ret;
  int rc = -1;

  if (getenv("TOA_IQ_PATH"))
    iq_path = getenv("TOA_IQ_PATH");
  if (getenv("TOA_IQ_META_PATH"))
    meta_path = getenv("TOA_IQ_META_PATH");
  if (getenv("TOA_IQ_BLOCKLOG_PATH"))
    blocklog_path = getenv("TOA_IQ_BLOCKLOG_PATH");

  memset(&pl, 0, sizeof(pl));
  log_set_level(LOG_LEVEL_INFO);

  ret = toa_context_init(&ctx);
  if (ret != TOA_OK) {
    log_msg(LOG_LEVEL_ERROR, "TOA_OFFLINE", "toa_context_init failed, ret=%d", ret);
    goto out;
  }

  fs_env = (cli_env_f64("TOA_FS", &ctx.sample_rate_hz) == 0);
  (void)iq_replay_fill_fs_from_meta(iq_path, meta_path, fs_env, &ctx.sample_rate_hz);

  (void)cli_env_f64("TOA_FC", &ctx.center_freq_hz);
  {
    uint32_t pci_u32 = 0;
    if (cli_env_u32("TOA_PCI", &pci_u32) == 0)
      ctx.pci = (uint16_t)pci_u32;
  }
  (void)cli_env_u32("TOA_REF_LEN", &ctx.prs_ref_len);
  {
    uint32_t v = 0;
    if (cli_env_u32("TOA_ENABLE_CFO", &v) == 0)
      ctx.enable_cfo = (v != 0);
  }
  (void)cli_env_u32("TOA_SEARCH_START", &ctx.search_start);
  (void)cli_env_u32("TOA_SEARCH_LEN", &ctx.search_len);
  (void)cli_env_u32("TOA_SEARCH_STEP", &ctx.search_step);
  (void)cli_env_u32("TOA_TRACK_HALF_WINDOW", &ctx.track_half_window);
  (void)cli_env_f64("TOA_MIN_CONF", &ctx.min_confidence);
  (void)cli_env_f64("TOA_MIN_SNR_DB", &ctx.min_snr_db);
  if (ctx.search_step == 0)
    ctx.search_step = 1;

  log_msg(LOG_LEVEL_INFO,
          "TOA_OFFLINE",
          "cfg fs=%.3f fc=%.3f pci=%u ref_len=%u",
          ctx.sample_rate_hz,
          ctx.center_freq_hz,
          (unsigned)ctx.pci,
          (unsigned)ctx.prs_ref_len);
  log_msg(LOG_LEVEL_INFO, "TOA_OFFLINE", "cfg enable_cfo=%d", ctx.enable_cfo);
  log_msg(LOG_LEVEL_INFO,
          "TOA_OFFLINE",
          "cfg search_start=%u search_len=%u search_step=%u",
          (unsigned)ctx.search_start,
          (unsigned)ctx.search_len,
          (unsigned)ctx.search_step);
  log_msg(LOG_LEVEL_INFO,
          "TOA_OFFLINE",
          "cfg track_half_window=%u min_conf=%.3f min_snr_db=%.3f",
          (unsigned)ctx.track_half_window,
          ctx.min_confidence,
          ctx.min_snr_db);

  ret = toa_pipeline_init(&pl, &ctx);
  if (ret != TOA_OK) {
    log_msg(LOG_LEVEL_ERROR, "TOA_OFFLINE", "toa_pipeline_init failed, ret=%d", ret);
    goto out;
  }

  memset(&ud, 0, sizeof(ud));
  ud.pl = &pl;
  ud.ctx = &ctx;

  tr = offline_run_cf32_file(iq_path,
                             meta_path,
                             blocklog_path,
                             4096u,
                             ctx.sample_rate_hz,
                             &ud,
                             toa_offline_cb);
  if (tr != TOA_OK) {
    log_msg(LOG_LEVEL_ERROR, "TOA_OFFLINE", "offline_run_cf32_file failed: %d", tr);
    goto out_free_pl;
  }

  log_msg(LOG_LEVEL_INFO,
          "TOA_OFFLINE",
          "done file=%s frames=%llu samples=%llu obs=%llu",
          iq_path,
          (unsigned long long)pl.frames_processed,
          (unsigned long long)ud.total_samples,
          (unsigned long long)ud.total_obs);

  rc = 0;
out_free_pl:
  toa_pipeline_free(&pl);
out:
  return rc;
}
