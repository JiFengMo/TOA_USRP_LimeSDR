#include "openair1/PHY/NR_POSITIONING/nr_pos_types.h"

#include <string.h>

int nr_ssb_build_tx_plan(const nr_toa_app_cfg_t *cfg, nr_ssb_tx_plan_t *plan)
{
  if (!cfg || !plan) {
    return -1;
  }
  memset(plan, 0, sizeof(*plan));
  plan->period_ms = 20;
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
