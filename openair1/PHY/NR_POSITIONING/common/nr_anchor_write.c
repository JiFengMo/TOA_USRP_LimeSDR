#include "openair1/PHY/NR_POSITIONING/nr_pos_types.h"
#include "radio/COMMON/common_lib.h"

#include <stdio.h>

static openair0_timestamp_t g_last_tx_timestamp;

int nr_anchor_write_burst(openair0_device_t *dev,
                          const nr_ssb_tx_plan_t *plan,
                          const nr_tx_burst_t *burst)
{
  if (!dev || !plan || !burst || !dev->trx_write_func) {
    return -1;
  }
  void *buf[NR_TOA_MAX_RX_ANT] = {0};
  buf[0] = burst->tx[0];
  openair0_timestamp_t ts = plan->tx_hw_timestamp;
  int rc = dev->trx_write_func(dev, &ts, buf, burst->nsamps, 0, 0);
  if (rc >= 0) {
    g_last_tx_timestamp = plan->tx_hw_timestamp;
  }
  return rc;
}

int nr_anchor_log_tx_event(const nr_ssb_tx_plan_t *plan)
{
  if (!plan) {
    return -1;
  }
  static int initialized = 0;
  static FILE *fp = NULL;
  if (!initialized) {
    fp = fopen("tx_event.csv", "w");
    if (!fp) {
      return -1;
    }
    fprintf(fp, "epoch_id,anchor_id,pci,ssb_index,tx_hw_timestamp,period_ms\n");
    initialized = 1;
  }
  fprintf(fp, "%llu,%u,%u,%u,%llu,%u\n",
          (unsigned long long)plan->epoch_id, (unsigned)plan->anchor_id,
          (unsigned)plan->pci, (unsigned)plan->ssb_index,
          (unsigned long long)plan->tx_hw_timestamp, (unsigned)plan->period_ms);
  fflush(fp);
  return 0;
}

openair0_timestamp_t nr_toa_get_last_tx_timestamp(void)
{
  return g_last_tx_timestamp;
}
