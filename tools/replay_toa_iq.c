#include "openair1/PHY/NR_POSITIONING/nr_pos_api.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int load_fs_from_meta(const char *iq_path, double *fs_hz)
{
  char meta_path[1024];
  size_t n = 0U;
  FILE *fp = NULL;
  char line[512];

  if (!iq_path || !fs_hz) {
    return -1;
  }
  snprintf(meta_path, sizeof(meta_path), "%s", iq_path);
  n = strlen(meta_path);
  if (n >= 4U && strcmp(meta_path + n - 4U, ".c16") == 0) {
    meta_path[n - 4U] = '\0';
  }
  snprintf(meta_path + strlen(meta_path),
           sizeof(meta_path) - strlen(meta_path),
           ".meta.txt");
  fp = fopen(meta_path, "r");
  if (!fp) {
    return -1;
  }
  while (fgets(line, sizeof(line), fp)) {
    if (strncmp(line, "fs_hz=", 6) == 0) {
      *fs_hz = strtod(line + 6, NULL);
      fclose(fp);
      return (*fs_hz > 0.0) ? 0 : -1;
    }
  }
  fclose(fp);
  return -1;
}

int main(int argc, char **argv)
{
  nr_iq_ring_t ring;
  nr_iq_block_t *blk = NULL;
  FILE *fp = NULL;
  long sz = 0;
  size_t got = 0;
  double fs_hz = 0.0;
  nr_pss_hit_t hits[4];
  nr_sync_state_t best_sync;
  int best_valid = 0;
  float best_score = -1.0f;

  if (argc < 2 || argc > 3) {
    fprintf(stderr, "usage: %s <capture.c16> [fs_hz]\n", argv[0]);
    return 1;
  }

  if (argc >= 3) {
    fs_hz = strtod(argv[2], NULL);
  } else if (load_fs_from_meta(argv[1], &fs_hz) != 0) {
    fprintf(stderr, "failed to infer fs_hz from %s.meta.txt; pass fs_hz explicitly\n",
            argv[1]);
    return 1;
  }
  if (!(fs_hz > 0.0)) {
    fprintf(stderr, "invalid fs_hz\n");
    return 1;
  }

  if (nr_iq_ring_init(&ring, 1) != 0) {
    return 1;
  }

  fp = fopen(argv[1], "rb");
  if (!fp) {
    perror("fopen");
    nr_iq_ring_free(&ring);
    return 1;
  }
  if (fseek(fp, 0, SEEK_END) != 0) {
    perror("fseek");
    fclose(fp);
    nr_iq_ring_free(&ring);
    return 1;
  }
  sz = ftell(fp);
  if (sz <= 0 || (sz % (long)sizeof(c16_t)) != 0) {
    fprintf(stderr, "invalid IQ file size: %ld bytes\n", sz);
    fclose(fp);
    nr_iq_ring_free(&ring);
    return 1;
  }
  rewind(fp);

  blk = nr_iq_ring_alloc_ex(&ring, (uint32_t)(sz / (long)sizeof(c16_t)), 1U);
  if (!blk) {
    fclose(fp);
    nr_iq_ring_free(&ring);
    return 1;
  }
  blk->fs_hz = fs_hz;
  got = fread(blk->rx[0], sizeof(c16_t), blk->nsamps, fp);
  fclose(fp);
  if (got != blk->nsamps) {
    fprintf(stderr, "short read: got %zu samples, expected %u\n", got, blk->nsamps);
    nr_iq_block_put(blk);
    nr_iq_ring_free(&ring);
    return 1;
  }

  if (nr_ssb_pss_search(blk, hits, 4) != 0) {
    printf("replay: no PSS detected\n");
    nr_iq_block_put(blk);
    nr_iq_ring_free(&ring);
    return 2;
  }

  printf("replay: hits [0] nid2=%u off=%d m=%.3f [1] nid2=%u off=%d m=%.3f [2] nid2=%u off=%d m=%.3f\n",
         (unsigned)hits[0].nid2, hits[0].peak_samp, hits[0].metric,
         (unsigned)hits[1].nid2, hits[1].peak_samp, hits[1].metric,
         (unsigned)hits[2].nid2, hits[2].peak_samp, hits[2].metric);

  memset(&best_sync, 0, sizeof(best_sync));
  for (int i = 0; i < 4; i++) {
    nr_sync_state_t cand;
    nr_sync_state_t probe;
    float score = 0.0f;
    int pbch_rc = -1;
    if (hits[i].metric <= 0.0f) {
      continue;
    }
    if (nr_ssb_refine_sync(blk, &hits[i], &cand) != 0) {
      continue;
    }
    printf("replay: cand[%d] pci=%u off=%d cfo=%.2f snr=%.2f pss=%.3f\n",
           i, (unsigned)cand.pci, cand.coarse_offset_samp, cand.cfo_hz,
           cand.snr_db, cand.pss_metric);
    probe = cand;
    pbch_rc = nr_ssb_pbch_decode(blk, &probe);
    printf("replay: cand[%d] pbch=%s ssb=%u stage=%u dmrs=%.3f second=%.3f mib=%u sfn=%u payload=0x%06x\n",
           i, (pbch_rc == 0 && probe.pbch_ok) ? "ok" : "fail",
           (unsigned)probe.ssb_index, (unsigned)probe.pbch_fail_stage,
           probe.pbch_metric, probe.pbch_metric_second,
           (unsigned)probe.mib_ok, (unsigned)probe.sfn,
           (unsigned)probe.mib_payload);
    score = (pbch_rc == 0 && probe.pbch_ok)
                ? (1000.0f + probe.pbch_metric)
                : (cand.snr_db + 0.75f * probe.pbch_metric + 0.10f * cand.pss_metric);
    if (!best_valid || score > best_score) {
      best_score = score;
      best_sync = probe;
      best_valid = 1;
    }
  }

  if (!best_valid) {
    printf("replay: refine_sync failed for all candidates\n");
    nr_iq_block_put(blk);
    nr_iq_ring_free(&ring);
    return 3;
  }

  if (best_sync.pbch_ok) {
    printf("replay: PBCH OK pci=%u ssb=%u dmrs=%.3f second=%.3f mib=%u sfn=%u mib_payload=0x%06x\n",
           (unsigned)best_sync.pci, (unsigned)best_sync.ssb_index,
           best_sync.pbch_metric, best_sync.pbch_metric_second,
           (unsigned)best_sync.mib_ok, (unsigned)best_sync.sfn,
           (unsigned)best_sync.mib_payload);
  } else {
    printf("replay: PBCH FAIL pci=%u stage=%u dmrs=%.3f second=%.3f pss=%.3f snr=%.2f\n",
           (unsigned)best_sync.pci, (unsigned)best_sync.pbch_fail_stage,
           best_sync.pbch_metric, best_sync.pbch_metric_second,
           best_sync.pss_metric, best_sync.snr_db);
  }

  nr_iq_block_put(blk);
  nr_iq_ring_free(&ring);
  return 0;
}
