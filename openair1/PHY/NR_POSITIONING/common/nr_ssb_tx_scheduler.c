#include "openair1/PHY/NR_POSITIONING/nr_pos_types.h"

#include <string.h>

int nr_ssb_build_tx_plan(const nr_toa_app_cfg_t *cfg, nr_ssb_tx_plan_t *plan)
{
  if (!cfg || !plan) {
    return -1;
  }
  memset(plan, 0, sizeof(*plan));
  plan->anchor_id = 0;
  plan->pci = 0;
  plan->ssb_index = 0;
  plan->period_ms = cfg->ssb_period_ms ? cfg->ssb_period_ms : 20;
  return 0;
}

int nr_ssb_plan_next_epoch(nr_ssb_tx_plan_t *plan, uint64_t epoch_id)
{
  if (!plan) {
    return -1;
  }
  plan->epoch_id = epoch_id;
  return 0;
}
