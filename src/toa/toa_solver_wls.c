#include "toa/toa_solver.h"

int toa_solver_wls_2d(const toa_gnb_position_t *gnbs,
                      const toa_obs_t *obs,
                      uint32_t n,
                      toa_solution_t *sol)
{
  (void)gnbs;
  (void)obs;

  if (!sol || n < 3)
    return -1;

  sol->x_m = 0.0;
  sol->y_m = 0.0;
  sol->z_m = 0.0;
  sol->valid = 0;
  return 0;
}
