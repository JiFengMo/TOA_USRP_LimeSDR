#include "openair1/PHY/NR_POSITIONING/nr_pos_types.h"

#include <string.h>

int nr_epoch_mgr_push(nr_epoch_mgr_t *mgr, const nr_toa_meas_t *meas)
{
  if (!mgr || !meas) {
    return -1;
  }
  mgr->pending = 1;
  (void)meas;
  return 0;
}

int nr_epoch_mgr_pop_ready(nr_epoch_mgr_t *mgr, nr_toa_epoch_t *epoch)
{
  if (!mgr || !epoch) {
    return -1;
  }
  memset(epoch, 0, sizeof(*epoch));
  epoch->epoch_id = mgr->next_epoch_id++;
  mgr->pending = 0;
  return 0;
}
