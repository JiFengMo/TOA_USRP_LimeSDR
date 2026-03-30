#include "openair1/PHY/NR_POSITIONING/nr_pos_types.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int nr_toa_load_anchor_db(const char *path, nr_anchor_desc_t *db, int *n)
{
  if (!path || !db || !n) {
    return -1;
  }

  FILE *fp = fopen(path, "r");
  if (!fp) {
    return -1;
  }

  memset(db, 0, sizeof(nr_anchor_desc_t) * NR_TOA_MAX_ANCHORS);
  *n = 0;

  char line[1024];
  while (fgets(line, sizeof(line), fp)) {
    line[strcspn(line, "\r\n")] = '\0';

    /* Skip empty / comment lines */
    char *p = line;
    while (*p && (*p == ' ' || *p == '\t')) {
      p++;
    }
    if (*p == '\0' || *p == '#') {
      continue;
    }

    /* Header line heuristic: contains 'anchor_id' */
    if (strstr(p, "anchor_id") != NULL) {
      continue;
    }

    if (*n >= NR_TOA_MAX_ANCHORS) {
      break;
    }

    /* CSV: anchor_id,pci,ssb_index,x_m,y_m,z_m,absolute_time_valid,hw_cal_delay_ns */
    char *save = NULL;
    char *tok = strtok_r(p, ",", &save);
    if (!tok) {
      continue;
    }
    db[*n].anchor_id = (uint8_t)strtoul(tok, NULL, 0);

    tok = strtok_r(NULL, ",", &save);
    if (!tok) {
      continue;
    }
    db[*n].pci = (uint16_t)strtoul(tok, NULL, 0);

    tok = strtok_r(NULL, ",", &save);
    if (!tok) {
      continue;
    }
    db[*n].ssb_index = (uint8_t)strtoul(tok, NULL, 0);

    tok = strtok_r(NULL, ",", &save);
    if (!tok) {
      continue;
    }
    db[*n].x_m = strtod(tok, NULL);

    tok = strtok_r(NULL, ",", &save);
    if (!tok) {
      continue;
    }
    db[*n].y_m = strtod(tok, NULL);

    tok = strtok_r(NULL, ",", &save);
    if (!tok) {
      continue;
    }
    db[*n].z_m = strtod(tok, NULL);

    tok = strtok_r(NULL, ",", &save);
    if (!tok) {
      continue;
    }
    db[*n].absolute_time_valid = (uint8_t)strtoul(tok, NULL, 0);

    tok = strtok_r(NULL, ",", &save);
    if (!tok) {
      db[*n].hw_cal_delay_ns = 0.0;
    } else {
      db[*n].hw_cal_delay_ns = strtod(tok, NULL);
    }

    (*n)++;
  }

  fclose(fp);
  return 0;
}
