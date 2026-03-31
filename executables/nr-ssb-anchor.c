#include "executables/nr-toa-threads.h"
#include "openair1/PHY/NR_POSITIONING/nr_pos_api.h"
#include "radio/COMMON/common_lib.h"

#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>

volatile int oai_exit;

typedef struct {
  nr_toa_app_cfg_t app;
  openair0_config_t rf;
  openair0_device_t *dev;
  nr_ssb_tx_plan_t plan;
  c16_t *txbuf_mem;
  nr_tx_burst_t burst;
} nr_anchor_ctx_t;

static void nr_anchor_sig_handler(int signo)
{
  (void)signo;
  oai_exit = 1;
}

static void *ANCHOR_main_thread(void *arg)
{
  nr_anchor_ctx_t *ctx = (nr_anchor_ctx_t *)arg;
  nr_clock_status_t clk;
  memset(&clk, 0, sizeof(clk));

  while (!oai_exit && nr_toa_wait_clock_lock(ctx->dev, &clk) != 0) {
    usleep(10000);
  }
  if (oai_exit) {
    return NULL;
  }

  (void)nr_toa_set_epoch_at_next_pps(ctx->dev, 0);

  uint64_t epoch_id = 0;
  openair0_timestamp_t now = 0;
  const double fs = (ctx->rf.sample_rate > 0.0) ? ctx->rf.sample_rate : 30.72e6;
  const uint64_t period_samp =
      (uint64_t)((double)ctx->plan.period_ms * 1e-3 * fs);

  while (!oai_exit) {
    if (nr_toa_get_device_time(ctx->dev, &now) != 0) {
      usleep(10000);
      continue;
    }

    (void)nr_ssb_plan_next_epoch(&ctx->plan, epoch_id++);
    /* In simulator path, schedule slightly ahead to avoid long initial silence. */
    uint64_t lead_samp = (period_samp > 8192U) ? 8192U : (period_samp / 2U);
    if (lead_samp == 0U) {
      lead_samp = 1024U;
    }
    ctx->plan.tx_hw_timestamp = now + lead_samp;

    ctx->burst.ts_first = ctx->plan.tx_hw_timestamp;
    (void)nr_anchor_write_burst(ctx->dev, &ctx->plan, &ctx->burst);
    (void)nr_anchor_log_tx_event(&ctx->plan);

    usleep((useconds_t)(ctx->plan.period_ms * 1000));
  }
  return NULL;
}

int main(int argc, char **argv)
{
  oai_exit = 0;
  signal(SIGINT, nr_anchor_sig_handler);
  signal(SIGTERM, nr_anchor_sig_handler);

  nr_anchor_ctx_t ctx;
  memset(&ctx, 0, sizeof(ctx));

  if (nr_toa_load_config(
          (argc > 1) ? argv[1]
                     : "targets/PROJECTS/NR-TOA/CONF/anchor0.ssb.usrpb210.conf",
          &ctx.app) != 0) {
    return 1;
  }
  if (nr_toa_build_rf_cfg(&ctx.app, &ctx.rf) != 0) {
    return 1;
  }

  if (nr_ssb_build_tx_plan(&ctx.app, &ctx.plan) != 0) {
    return 1;
  }

  if (nr_toa_radio_init(&ctx.dev, &ctx.rf) != 0) {
    return 1;
  }
  if (nr_toa_radio_start(ctx.dev) != 0) {
    return 1;
  }

  nr_ssb_ref_t ref;
  nr_ssb_grid_t grid;
  if (nr_ssb_gen_ref(ctx.plan.pci, ctx.plan.ssb_index, &ref) != 0) {
    return 1;
  }
  if (nr_ssb_build_grid(&ref, &grid) != 0) {
    return 1;
  }
  ctx.txbuf_mem = (c16_t *)calloc(4096, sizeof(c16_t));
  if (!ctx.txbuf_mem) {
    return 1;
  }
  memset(&ctx.burst, 0, sizeof(ctx.burst));
  ctx.burst.tx[0] = ctx.txbuf_mem;
  if (nr_ssb_ofdm_mod(&grid, &ctx.burst) != 0) {
    return 1;
  }

  (void)TOA_THREAD_ANCHOR_MAIN;
  pthread_t tid;
  if (pthread_create(&tid, NULL, ANCHOR_main_thread, &ctx) != 0) {
    return 1;
  }
  pthread_join(tid, NULL);
  free(ctx.txbuf_mem);
  return 0;
}
