#include "nr/nr_sequence.h"
#include "common/demo_rx_defaults.h"
#include "common/error.h"
#include "io/iq_meta.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/*
 * 写出与自检程序同结构的�? IQ�?.cf32 + .meta.json），�? nr_ssb_scan 回放�?
 */
int main(int argc, char **argv)
{
  enum { NSAMP = 8192 };
  const char *base = (argc > 1) ? argv[1] : "nr_synth_ssb";
  char path_cf32[512];
  char path_meta[512];
  cf32_t *samples = NULL;
  cf32_t pss[NR_PSS_LEN];
  cf32_t sss[NR_SSS_LEN];
  iq_meta_t meta;
  FILE *fp = NULL;
  const uint32_t nid1 = 42u;
  const uint32_t nid2 = 2u;
  const uint32_t p0 = 800u;
  uint32_t s0;
  int rc = 1;

  snprintf(path_cf32, sizeof(path_cf32), "%s.cf32", base);
  snprintf(path_meta, sizeof(path_meta), "%s.meta.json", base);

  samples = (cf32_t *)calloc(NSAMP, sizeof(cf32_t));
  if (!samples) {
    fprintf(stderr, "nr_synth_ssb: oom\n");
    return 1;
  }

  if (nr_generate_pss(nid2, pss, NR_PSS_LEN) != TOA_OK ||
      nr_generate_sss(nid1, nid2, sss, NR_SSS_LEN) != TOA_OK) {
    fprintf(stderr, "nr_synth_ssb: sequence gen failed\n");
    goto out;
  }

  for (uint32_t i = 0; i < NR_PSS_LEN; ++i) {
    samples[p0 + i].r = pss[i].r * 0.6f;
    samples[p0 + i].i = pss[i].i * 0.6f;
  }
  s0 = p0 + (uint32_t)DEMO_NR_SSS_LAG_SAMPLES;
  for (uint32_t i = 0; i < NR_SSS_LEN; ++i) {
    samples[s0 + i].r = sss[i].r * 0.5f;
    samples[s0 + i].i = sss[i].i * 0.5f;
  }

  fp = fopen(path_cf32, "wb");
  if (!fp || fwrite(samples, sizeof(cf32_t), (size_t)NSAMP, fp) != (size_t)NSAMP) {
    fprintf(stderr, "nr_synth_ssb: write %s failed\n", path_cf32);
    goto out;
  }
  fclose(fp);
  fp = NULL;

  iq_meta_init_default(&meta);
  meta.sample_rate_hz = DEMO_RX_SAMPLE_RATE_HZ;
  meta.center_freq_hz = DEMO_RX_CENTER_FREQ_HZ;
  meta.bandwidth_hz = DEMO_RX_BANDWIDTH_HZ;
  meta.gain_db = DEMO_RX_GAIN_DB;
  meta.block_size = DEMO_RX_BUFFER_SAMPLES;
  meta.total_samples = (uint64_t)NSAMP;
  snprintf(meta.device_args, sizeof(meta.device_args), "synth");

  if (iq_meta_save_json(path_meta, &meta) != TOA_OK) {
    fprintf(stderr, "nr_synth_ssb: meta %s failed\n", path_meta);
    goto out;
  }

  printf("nr_synth_ssb: wrote %s + %s (%u samples)\n", path_cf32, path_meta, (unsigned)NSAMP);
  rc = 0;
out:
  if (fp)
    fclose(fp);
  free(samples);
  return rc;
}
