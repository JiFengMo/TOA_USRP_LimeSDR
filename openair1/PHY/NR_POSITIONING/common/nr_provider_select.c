#include "openair1/PHY/NR_POSITIONING/nr_pos_api.h"

#include <ctype.h>
#include <string.h>

static int nr_provider_streq_icase(const char *a, const char *b)
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
  return *a == '\0' && *b == '\0';
}

const nr_pos_provider_if_t *nr_toa_select_provider(const nr_toa_app_cfg_t *cfg)
{
  if (!cfg) {
    return NULL;
  }

  if (cfg->provider[0] != '\0') {
    if (nr_provider_streq_icase(cfg->provider, "fpga_ssb") ||
        nr_provider_streq_icase(cfg->provider, "fpga") ||
        nr_provider_streq_icase(cfg->provider, "hw_ssb")) {
      return &nr_fpga_ssb_provider;
    }
    if (nr_provider_streq_icase(cfg->provider, "prs")) {
      return &nr_prs_provider;
    }
    if (nr_provider_streq_icase(cfg->provider, "ssb") ||
        nr_provider_streq_icase(cfg->provider, "software_ssb")) {
      return &nr_ssb_provider;
    }
  }

  return (cfg->mode == NR_TOA_MODE_PRS_TOA) ? &nr_prs_provider : &nr_ssb_provider;
}
