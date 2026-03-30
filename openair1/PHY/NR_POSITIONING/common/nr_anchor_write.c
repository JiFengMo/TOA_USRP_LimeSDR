#include "openair1/PHY/NR_POSITIONING/nr_pos_types.h"
#include "radio/COMMON/common_lib.h"

int nr_anchor_write_burst(openair0_device_t *dev,
                          const nr_ssb_tx_plan_t *plan, c16_t **txbuf)
{
  if (!dev || !plan || !txbuf) {
    return -1;
  }
  (void)dev;
  (void)plan;
  (void)txbuf;
  return 0;
}

int nr_anchor_log_tx_event(const nr_ssb_tx_plan_t *plan)
{
  if (!plan) {
    return -1;
  }
  return 0;
}
