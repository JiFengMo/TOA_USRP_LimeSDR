#include "openair1/PHY/NR_POSITIONING/nr_pos_types.h"

#include <string.h>

int nr_pos_build_equations(const nr_toa_epoch_t *epoch, nr_solver_input_t *in)
{
  if (!epoch || !in) {
    return -1;
  }
  memset(in, 0, sizeof(*in));
  return 0;
}

int nr_pos_solve_wls(const nr_solver_input_t *in, nr_loc_solution_t *sol)
{
  if (!in || !sol) {
    return -1;
  }
  memset(sol, 0, sizeof(*sol));
  sol->valid = 0;
  return 0;
}

int nr_pos_validate_solution(const nr_loc_solution_t *sol)
{
  if (!sol) {
    return -1;
  }
  return sol->valid ? 0 : -1;
}
