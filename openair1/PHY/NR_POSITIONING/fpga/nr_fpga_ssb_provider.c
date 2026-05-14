#include "openair1/PHY/NR_POSITIONING/nr_pos_provider_if.h"
#include "openair1/PHY/NR_POSITIONING/nr_pos_api.h"

#include <stdio.h>
#include <stdlib.h>

typedef struct {
  uint8_t emulate;
} nr_fpga_ssb_ctx_t;

static nr_fpga_ssb_ctx_t *nr_fpga_ctx(void *ctx)
{
  static nr_fpga_ssb_ctx_t fallback;
  nr_toa_ue_t *ue = (nr_toa_ue_t *)ctx;
  if (!ue) {
    return &fallback;
  }
  if (!ue->provider_ctx || ue->provider_ctx == ctx) {
    return &fallback;
  }
  return (nr_fpga_ssb_ctx_t *)ue->provider_ctx;
}

static int fpga_ssb_init(void *ctx)
{
  nr_fpga_ssb_ctx_t *fpga = nr_fpga_ctx(ctx);
  const char *backend = getenv("NR_TOA_FPGA_BACKEND");
  fpga->emulate = (!backend || backend[0] == '\0' ||
                   backend[0] == 'e' || backend[0] == 'E') ? 1U : 0U;

  if (fpga->emulate) {
    printf("fpga_ssb_provider: using software-emulated backend for golden-vector bring-up\n");
    return nr_ssb_provider.init ? nr_ssb_provider.init(ctx) : 0;
  }

  fprintf(stderr,
          "fpga_ssb_provider: hardware backend '%s' requested but UIO/DMA binding is not implemented yet\n",
          backend);
  return -1;
}

static int fpga_ssb_acquire(void *ctx, const nr_iq_block_t *blk, nr_sync_state_t *sync)
{
  nr_fpga_ssb_ctx_t *fpga = nr_fpga_ctx(ctx);
  if (!fpga->emulate) {
    return -1;
  }
  return nr_ssb_provider.acquire ? nr_ssb_provider.acquire(ctx, blk, sync) : -1;
}

static int fpga_ssb_track(void *ctx, const nr_iq_block_t *blk, nr_sync_state_t *sync)
{
  nr_fpga_ssb_ctx_t *fpga = nr_fpga_ctx(ctx);
  if (!fpga->emulate) {
    return -1;
  }
  return nr_ssb_provider.track ? nr_ssb_provider.track(ctx, blk, sync) : -1;
}

static int fpga_ssb_extract_meas(void *ctx, const nr_iq_block_t *blk,
                                 const nr_sync_state_t *sync, nr_toa_meas_t *meas)
{
  nr_fpga_ssb_ctx_t *fpga = nr_fpga_ctx(ctx);
  if (!fpga->emulate) {
    return -1;
  }
  return nr_ssb_provider.extract_meas ? nr_ssb_provider.extract_meas(ctx, blk, sync, meas) : -1;
}

static int fpga_ssb_dump_trace(void *ctx)
{
  nr_fpga_ssb_ctx_t *fpga = nr_fpga_ctx(ctx);
  if (!fpga->emulate) {
    return -1;
  }
  return nr_ssb_provider.dump_trace ? nr_ssb_provider.dump_trace(ctx) : 0;
}

const nr_pos_provider_if_t nr_fpga_ssb_provider = {
    .name = "fpga_ssb_provider",
    .init = fpga_ssb_init,
    .acquire = fpga_ssb_acquire,
    .track = fpga_ssb_track,
    .extract_meas = fpga_ssb_extract_meas,
    .dump_trace = fpga_ssb_dump_trace,
};
