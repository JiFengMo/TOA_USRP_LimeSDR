#include "openair1/PHY/NR_POSITIONING/nr_pos_types.h"

#include <string.h>

int nr_epoch_mgr_push(nr_epoch_mgr_t *mgr, const nr_toa_meas_t *meas)
{
  if (!mgr || !meas) {
    return -1;
  }
  if (mgr->meas_count < NR_TOA_MAX_MEAS_PER_EP) {
    mgr->meas_buf[mgr->meas_count++] = *meas;
    mgr->pending = 1;
  } else {
    /* Drop policy: silently drop when epoch buffer full for Phase-0. */
    mgr->pending = 1;
  }
  return 0;
}

int nr_epoch_mgr_pop_ready(nr_epoch_mgr_t *mgr, nr_toa_epoch_t *epoch)
{
  if (!mgr || !epoch) {
    return -1;
  }
  if (!mgr->pending || mgr->meas_count == 0) {
    return -1;
  }
  memset(epoch, 0, sizeof(*epoch));
  epoch->epoch_id = mgr->next_epoch_id++;
  epoch->num_meas = (uint8_t)mgr->meas_count;
  for (uint32_t i = 0; i < mgr->meas_count && i < NR_TOA_MAX_MEAS_PER_EP; i++) {
    epoch->meas[i] = mgr->meas_buf[i];
  }
  mgr->pending = 0;
  mgr->meas_count = 0;
  return 0;
}
