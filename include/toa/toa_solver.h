#ifndef TOA_SOLVER_H
#define TOA_SOLVER_H

#include <stdint.h>
#include "toa/toa_types.h"

typedef struct {
  double x_m;
  double y_m;
  double z_m;
  int valid;
} toa_solution_t;

int toa_solver_wls_2d(const toa_gnb_position_t *gnbs,
                      const toa_obs_t *obs,
                      uint32_t n,
                      toa_solution_t *sol);

#endif
