#include "openair1/PHY/NR_POSITIONING/nr_pos_api.h"

#include <stdio.h>
#include <stdlib.h>

int main(int argc, char **argv)
{
  (void)argc;
  (void)argv;
  nr_iq_ring_t ring;
  if (nr_iq_ring_init(&ring, 1) != 0) {
    return 1;
  }
  return 0;
}
