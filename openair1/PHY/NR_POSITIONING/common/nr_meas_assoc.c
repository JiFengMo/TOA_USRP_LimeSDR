#include "openair1/PHY/NR_POSITIONING/nr_pos_types.h"

int nr_meas_assoc_anchor(const nr_toa_meas_t *meas, const nr_anchor_desc_t *db,
                         int n)
{
  if (!meas || !db || n <= 0) {
    return -1;
  }
  (void)meas;
  (void)db;
  return 0;
}
