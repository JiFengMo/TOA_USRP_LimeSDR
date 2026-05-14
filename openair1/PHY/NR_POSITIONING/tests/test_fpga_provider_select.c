#include "openair1/PHY/NR_POSITIONING/nr_pos_api.h"

#include <stdio.h>
#include <string.h>

static int expect_provider(const char *name, const nr_toa_app_cfg_t *cfg)
{
  const nr_pos_provider_if_t *provider = nr_toa_select_provider(cfg);
  if (!provider || !provider->name || strcmp(provider->name, name) != 0) {
    return 1;
  }
  return 0;
}

int main(void)
{
  nr_toa_app_cfg_t cfg;
  memset(&cfg, 0, sizeof(cfg));

  cfg.mode = NR_TOA_MODE_SSB_TOA;
  if (expect_provider("ssb_provider", &cfg) != 0) {
    return 1;
  }

  cfg.mode = NR_TOA_MODE_PRS_TOA;
  if (expect_provider("prs_provider", &cfg) != 0) {
    return 1;
  }

  cfg.mode = NR_TOA_MODE_SSB_TOA;
  (void)snprintf(cfg.provider, sizeof(cfg.provider), "%s", "fpga_ssb");
  if (expect_provider("fpga_ssb_provider", &cfg) != 0) {
    return 1;
  }

  return 0;
}
