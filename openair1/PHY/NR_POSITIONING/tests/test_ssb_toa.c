#include "openair1/PHY/NR_POSITIONING/nr_pos_api.h"

#include <stdlib.h>
#include <string.h>

int main(void)
{
  nr_cir_t cir;
  int peak = 0;
  double frac = 0.0;
  memset(&cir, 0, sizeof(cir));
  if (nr_toa_find_integer_peak(&cir, &peak) != 0) {
    return 1;
  }
  if (nr_toa_refine_fractional(&cir, peak, &frac) != 0) {
    return 1;
  }
  return 0;
}
