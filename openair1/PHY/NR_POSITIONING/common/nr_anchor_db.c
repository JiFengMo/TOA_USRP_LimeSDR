#include "openair1/PHY/NR_POSITIONING/nr_pos_types.h"

#include <stdio.h>
#include <string.h>

int nr_toa_load_anchor_db(const char *path, nr_anchor_desc_t *db, int *n)
{
  if (!path || !db || !n) {
    return -1;
  }
  (void)path;
  *n = 0;
  memset(db, 0, sizeof(nr_anchor_desc_t) * NR_TOA_MAX_ANCHORS);
  return 0;
}
