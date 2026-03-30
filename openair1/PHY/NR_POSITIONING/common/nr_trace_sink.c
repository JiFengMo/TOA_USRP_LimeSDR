#include "openair1/PHY/NR_POSITIONING/nr_pos_types.h"

#include <stdio.h>

int nr_trace_sync(const nr_sync_state_t *sync)
{
  if (!sync) {
    return -1;
  }
  return 0;
}

int nr_trace_cir(const nr_cir_t *cir)
{
  if (!cir) {
    return -1;
  }
  return 0;
}

int nr_trace_meas(const nr_toa_meas_t *meas)
{
  if (!meas) {
    return -1;
  }
  return 0;
}

int nr_trace_solution(const nr_loc_solution_t *sol)
{
  if (!sol) {
    return -1;
  }
  return 0;
}
