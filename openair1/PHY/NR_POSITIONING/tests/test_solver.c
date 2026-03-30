#include "openair1/PHY/NR_POSITIONING/nr_pos_api.h"

#include <string.h>

int main(void)
{
  nr_toa_epoch_t ep;
  nr_solver_input_t in;
  nr_loc_solution_t sol;
  memset(&ep, 0, sizeof(ep));
  if (nr_pos_build_equations(&ep, &in) != 0) {
    return 1;
  }
  if (nr_pos_solve_wls(&in, &sol) != 0) {
    return 1;
  }
  return 0;
}
