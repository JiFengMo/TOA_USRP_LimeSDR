/*
 * Lime 闂佽�查��?�﹢�?嶅磻閻愯�ф�?�閻�?彃鈧�鍧楀极瀹ュ拑鎷峰�楀牊瀚归柣搴ｏ拷澶�??�� TS 38.104 n41闂備焦瀵х粙鎴︽偋婵犲洤鍨傞柨鐕傛�?? FR1 缂傚倷鐒﹂崝鏍�鏁撻挊澶涙�?�閿熺晫�?堥崪绋�?�N 闂備礁鎲＄敮妤呮晸閽樺�婏拷銊╁焵閿熶粙宕橀敓浠�?蓟閸涙潙绠涢柨鐔剁矙瀹曟垿鏁愰崶銊ワ拷锟介梺鍛婂姧閹风兘鏌ｉ敓浠�?晸娴犲��鐓ユ繛鎴烇�?閹�?�兘�?堢粣锟� 闂備礁鎲￠懝楣冨嫉閿熶粙鏌ㄩ悢鍑ゆ�?�閹�?拷閸旂數绱掗幉瀣�瀚�
 * 闂備礁鎲￠懝鐐�绺介�?鍌滃崥闁跨噦鎷� GSCN 濠电偞鍨堕幐鎼佹晝閵堝��?櫢闁兼亽鍎存竟妯荤�?�鐏忔牕澧叉繛鐓�?拷�?��鍊烽柣銏犵仛閺嗕即�?�虹憴鍕�鑸�闁稿�?�鐓″��锟界憸鐗堝笒缁��?��?�煛閸�銏℃毄缂佺姳�?欓弻锝�?�晲�?囨柨锟斤�? nr_ssb_sync 闂佽崵濮惧▍锝�?�窗閺囥垺鐓傛繝濠傛噽閻滅粯淇婇妶鍌�?壕閻熸粎澧�?�惄锟介柡浣哥Ф娴狅箓鎸婃径灞藉Ф濠电偛鐡ㄧ划锝夊�??閿燂�? SSS/PCI闂備焦瀵ч崘濠氬�?閿燂�?
 * 闂佸�?锟藉棙娅�?柛瀣�鍠栭幃浠�?晸閿燂拷 LIVE_SSB_STABLE_BLOCKS 闂佷�?鎷烽柨鐔剁矙閺佹捇鏁撻敓锟� PCI 濠电偞鍨堕幐鎾�宕戦幘缁樼厾濡炲��娴�?�弧鈧�缂備焦锟戒胶澧电�?碉拷閸�锟藉�板�?鈧�锝忔嫹婵炲懎绻橀弻娑樜旈埀锟介柣銈�?喘閺佹挻鎱ㄩ幇锟介埀锟介幘鑸靛�婚柣鏃囨�ｉ惌娆撴煕閹帮拷鐏忔瑩�?�?导瀛樷拺濠㈣泛鎽滈敓鎴掔窔閹�鐐�銈ｉ崘銊庛儵鏌ㄩ悤鍌涘��?
 */
#include "radio/radio_factory.h"
#include "common/cli.h"
#include "common/demo_rx_defaults.h"
#include "common/error.h"
#include "common/log.h"
#include "nr/gscn.h"
#include "nr/ssb_sync.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>

static void clamp_rx_to_caps(radio_channel_config_t *rx_cfg, const radio_device_caps_t *caps)
{
  if (rx_cfg->sample_rate < caps->sample_rate_min)
    rx_cfg->sample_rate = caps->sample_rate_min;
  if (rx_cfg->sample_rate > caps->sample_rate_max)
    rx_cfg->sample_rate = caps->sample_rate_max;
  if (rx_cfg->bandwidth < caps->bandwidth_min)
    rx_cfg->bandwidth = caps->bandwidth_min;
  if (rx_cfg->bandwidth > caps->bandwidth_max)
    rx_cfg->bandwidth = caps->bandwidth_max;
  if (rx_cfg->center_freq < caps->center_freq_min)
    rx_cfg->center_freq = caps->center_freq_min;
  if (rx_cfg->center_freq > caps->center_freq_max)
    rx_cfg->center_freq = caps->center_freq_max;
  if (rx_cfg->gain < caps->gain_min)
    rx_cfg->gain = caps->gain_min;
  if (rx_cfg->gain > caps->gain_max)
    rx_cfg->gain = caps->gain_max;
}

static nr_gscn_profile_t parse_profile(const char *s)
{
  if (!s || s[0] == '\0')
    return NR_GSCN_PROFILE_TS38104_N41_SCS30K;
  if (strcmp(s, "fr1_hz") == 0 || strcmp(s, "fr1") == 0)
    return NR_GSCN_PROFILE_FR1_FREQ_WINDOW;
  if (strcmp(s, "n41_15k") == 0 || strcmp(s, "n41_scs15") == 0)
    return NR_GSCN_PROFILE_TS38104_N41_SCS15K;
  if (strcmp(s, "n41_30k") == 0 || strcmp(s, "n41_scs30") == 0)
    return NR_GSCN_PROFILE_TS38104_N41_SCS30K;
  return NR_GSCN_PROFILE_TS38104_N41_SCS30K;
}

static void apply_nr_env(nr_ssb_cfg_t *cfg)
{
  (void)cli_env_u32("NR_SEARCH_STEP", &cfg->search_step);
  (void)cli_env_u32("NR_TRACK_HALF_WINDOW", &cfg->track_half_window);
  {
    uint32_t v = 0;
    if (cli_env_u32("NR_SSS_ENABLE", &v) == 0)
      cfg->sss_enable = (v != 0);
  }
  {
    double v = 0.0;
    if (cli_env_f64("NR_SSS_LAG", &v) == 0)
      cfg->sss_lag_samples = (int32_t)v;
  }
  (void)cli_env_u32("NR_SSS_SEARCH_HALF_WINDOW", &cfg->sss_search_half_window);
  {
    double v = 0.0;
    if (cli_env_f64("NR_MIN_METRIC", &v) == 0)
      cfg->min_metric = (float)v;
  }
  {
    double v = 0.0;
    if (cli_env_f64("NR_MIN_SSS_METRIC", &v) == 0)
      cfg->min_sss_metric = (float)v;
  }
  {
    double v = 0.0;
    if (cli_env_f64("NR_MIN_PSS_PEAK_RATIO", &v) == 0)
      cfg->min_pss_peak_ratio = (float)v;
  }
  {
    double v = 0.0;
    if (cli_env_f64("NR_MIN_SSS_PEAK_RATIO", &v) == 0)
      cfg->min_sss_peak_ratio = (float)v;
  }
  {
    uint32_t v = 0;
    if (cli_env_u32("NR_LOCK_NID2", &v) == 0)
      cfg->lock_nid2 = (v != 0);
  }
  {
    uint32_t v = 0;
    if (cli_env_u32("NR_LOCK_NID1", &v) == 0)
      cfg->lock_nid1 = (v != 0);
  }
  {
    uint32_t v = 1;
    if (cli_env_u32("NR_SSS_DEROTATE", &v) == 0)
      cfg->sss_derotate_cfo = (v != 0);
  }
  {
    double v = 0.0;
    if (cli_env_f64("NR_SSS_DEROTATE_MIN_CFO_HZ", &v) == 0)
      cfg->sss_derotate_min_cfo_hz = (float)v;
  }
}

static int discard_rx(radio_device_t *dev,
                      void *buffs[1],
                      uint32_t buf_samps,
                      uint32_t n_blocks)
{
  uint32_t b;
  for (b = 0; b < n_blocks; b++) {
    radio_rx_result_t rr;
    int r;

    memset(&rr, 0, sizeof(rr));
    r = dev->trx_read_func(dev, buffs, buf_samps, &rr, 200000);
    if (r != 0)
      return r;
  }
  return 0;
}

int main(void)
{
  radio_device_t device;
  radio_device_caps_t caps;
  radio_channel_config_t rx_cfg;
  radio_stream_config_t rx_stream_cfg;
  nr_ssb_cfg_t nr_cfg;
  nr_ssb_sync_ctx_t ssb_ctx;
  cf32_t *rx_buf = NULL;
  void *buffs[1];
  uint32_t cand[8192];
  size_t n_cand = 0;
  FILE *fp_csv = NULL;
  int caps_ok = 0;
  int ret;
  int rc = 1;
  uint32_t blocks_per_gscn = 60u;
  uint32_t stable_need = 5u;
  uint32_t warmup_blocks = 2u;
  uint32_t settle_ms = 5u;
  uint32_t reset_each_block = 0u;
  uint32_t stop_on_lock = 1u;
  double fmin_mhz = 2496.0;
  double fmax_mhz = 2690.0;
  uint32_t list_stride = 1u;
  int use_range = 0;
  uint32_t g_first = 0u;
  uint32_t g_last = 0u;
  nr_gscn_profile_t profile = NR_GSCN_PROFILE_TS38104_N41_SCS30K;
  const char *csv_path = NULL;
  uint64_t frame_id = 0u;

  memset(&device, 0, sizeof(device));
  memset(&nr_cfg, 0, sizeof(nr_cfg));
  log_set_level(LOG_LEVEL_INFO);

  profile = parse_profile(getenv("LIVE_GSCN_PROFILE"));
  (void)cli_env_f64("LIVE_GSCN_FMIN_MHZ", &fmin_mhz);
  (void)cli_env_f64("LIVE_GSCN_FMAX_MHZ", &fmax_mhz);
  (void)cli_env_u32("LIVE_SSB_BLOCKS_PER_GSCN", &blocks_per_gscn);
  if (blocks_per_gscn == 0u)
    blocks_per_gscn = 60u;
  (void)cli_env_u32("LIVE_SSB_STABLE_BLOCKS", &stable_need);
  if (stable_need == 0u)
    stable_need = 1u;
  (void)cli_env_u32("LIVE_SSB_WARMUP_BLOCKS", &warmup_blocks);
  (void)cli_env_u32("LIVE_SSB_SETTLE_MS", &settle_ms);
  if (settle_ms > 30000u)
    settle_ms = 30000u;
  {
    uint32_t v = 0;
    if (cli_env_u32("LIVE_SSB_RESET_EACH_BLOCK", &v) == 0)
      reset_each_block = v;
  }
  {
    uint32_t v = 1;
    if (cli_env_u32("LIVE_SSB_STOP_ON_LOCK", &v) == 0)
      stop_on_lock = v;
  }

  if (getenv("LIVE_SSB_CSV"))
    csv_path = getenv("LIVE_SSB_CSV");

  if (getenv("LIVE_GSCN_FIRST") != NULL && getenv("LIVE_GSCN_LAST") != NULL &&
      cli_env_u32("LIVE_GSCN_FIRST", &g_first) == 0 &&
      cli_env_u32("LIVE_GSCN_LAST", &g_last) == 0) {
    use_range = 1;
  }

  {
    int stride_env = (getenv("LIVE_GSCN_STRIDE") != NULL);

    if (stride_env) {
      (void)cli_env_u32("LIVE_GSCN_STRIDE", &list_stride);
      if (list_stride == 0u)
        list_stride = 1u;
    } else if (use_range) {
      /* TS 38.104 n41 sync raster step; avoids scanning non-raster GSCN */
      list_stride = NR_GSCN_SYNC_RASTER_STEP;
    } else {
      list_stride = 1u;
    }

    if (use_range && list_stride == 1u && stride_env)
      log_msg(LOG_LEVEL_WARN,
              "LIVE_SSB",
              "GSCN range with LIVE_GSCN_STRIDE=1 hits off-raster indices; use 3 for n41 or omit STRIDE");
    else if (use_range && !stride_env)
      log_msg(LOG_LEVEL_INFO,
              "LIVE_SSB",
              "LIVE_GSCN_STRIDE unset + FIRST/LAST: using stride=%u (n41 sync raster step)",
              list_stride);
  }

  nr_cfg.sample_rate_hz = DEMO_RX_SAMPLE_RATE_HZ;
  nr_cfg.search_step = 2;
  nr_cfg.track_half_window = 256;
  nr_cfg.min_metric = DEMO_NR_MIN_PSS_METRIC;
  nr_cfg.min_pss_peak_ratio = DEMO_NR_MIN_PSS_PEAK_RATIO;
  nr_cfg.sss_enable = 1;
  nr_cfg.sss_lag_samples = DEMO_NR_SSS_LAG_SAMPLES;
  nr_cfg.sss_search_half_window = DEMO_NR_SSS_SEARCH_HALF_WIN;
  nr_cfg.min_sss_metric = DEMO_NR_MIN_SSS_METRIC;
  nr_cfg.min_sss_peak_ratio = DEMO_NR_MIN_SSS_PEAK_RATIO;
  nr_cfg.lock_nid2 = 0;
  nr_cfg.lock_nid1 = 0;
  nr_cfg.sss_derotate_cfo = 1;
  nr_cfg.sss_derotate_min_cfo_hz = DEMO_NR_SSS_DEROTATE_MIN_CFO_HZ;
  (void)cli_env_f64("NR_FS", &nr_cfg.sample_rate_hz);
  apply_nr_env(&nr_cfg);

  if (use_range) {
    ret = (int)nr_gscn_fr1_collect_range(g_first,
                                        g_last,
                                        list_stride,
                                        cand,
                                        sizeof(cand) / sizeof(cand[0]),
                                        &n_cand);
  } else {
    ret = (int)nr_gscn_collect_for_profile(profile,
                                          fmin_mhz * 1e6,
                                          fmax_mhz * 1e6,
                                          list_stride,
                                          cand,
                                          sizeof(cand) / sizeof(cand[0]),
                                          &n_cand);
  }

  if (ret != TOA_OK || n_cand == 0) {
    log_msg(LOG_LEVEL_ERROR, "LIVE_SSB", "no GSCN candidates (ret=%d n=%zu)", ret, n_cand);
    goto out;
  }

  if (nr_ssb_sync_init(&ssb_ctx, &nr_cfg) != TOA_OK) {
    log_msg(LOG_LEVEL_ERROR, "LIVE_SSB", "nr_ssb_sync_init failed");
    goto out;
  }

  if (csv_path) {
    fp_csv = fopen(csv_path, "w");
    if (!fp_csv) {
      log_msg(LOG_LEVEL_ERROR, "LIVE_SSB", "open csv failed: %s", csv_path);
      goto out;
    }
    fprintf(fp_csv,
            "frame,gscn,ss_ref_mhz,found,nid2,nid1,pci,peak_pos,sss_pos,toa_ns,cfo_hz,"
            "pss_metric,pss_peak_ratio,sss_metric,sss_peak_ratio\n");
  }

  ret = radio_device_init(&device, RADIO_DEVICE_LIME);
  if (ret != TOA_OK) {
    log_msg(LOG_LEVEL_ERROR, "LIVE_SSB", "radio_device_init failed ret=%d", ret);
    goto out;
  }

  ret = device.dev_open(&device, "driver=lime");
  if (ret != TOA_OK) {
    log_msg(LOG_LEVEL_ERROR, "LIVE_SSB", "dev_open failed ret=%d", ret);
    goto out;
  }

  caps_ok = (device.dev_get_caps && device.dev_get_caps(&device, &caps) == TOA_OK);

  memset(&rx_cfg, 0, sizeof(rx_cfg));
  rx_cfg.channel_id = 0;
  rx_cfg.sample_rate = DEMO_RX_SAMPLE_RATE_HZ;
  rx_cfg.center_freq = DEMO_RX_CENTER_FREQ_HZ;
  rx_cfg.bandwidth = DEMO_RX_BANDWIDTH_HZ;
  rx_cfg.gain = DEMO_RX_GAIN_DB;
  (void)cli_env_f64("RECORD_FS", &rx_cfg.sample_rate);
  (void)cli_env_f64("RECORD_BW", &rx_cfg.bandwidth);
  (void)cli_env_f64("RECORD_GAIN_DB", &rx_cfg.gain);
  if (caps_ok)
    clamp_rx_to_caps(&rx_cfg, &caps);
  snprintf(rx_cfg.antenna, sizeof(rx_cfg.antenna), "LNAH");
  if (getenv("RECORD_ANTENNA"))
    snprintf(rx_cfg.antenna, sizeof(rx_cfg.antenna), "%s", getenv("RECORD_ANTENNA"));

  memset(&rx_stream_cfg, 0, sizeof(rx_stream_cfg));
  rx_stream_cfg.channel_id = 0;
  rx_stream_cfg.buffer_size = DEMO_RX_BUFFER_SAMPLES;

  ret = device.trx_configure_rx(&device, &rx_cfg);
  if (ret != TOA_OK) {
    log_msg(LOG_LEVEL_ERROR, "LIVE_SSB", "trx_configure_rx failed ret=%d", ret);
    goto out;
  }

  ret = device.trx_setup_rx_stream(&device, &rx_stream_cfg);
  if (ret != TOA_OK) {
    log_msg(LOG_LEVEL_ERROR, "LIVE_SSB", "trx_setup_rx_stream failed ret=%d", ret);
    goto out;
  }

  ret = device.trx_start_rx_stream(&device);
  if (ret != TOA_OK) {
    log_msg(LOG_LEVEL_ERROR, "LIVE_SSB", "trx_start_rx_stream failed ret=%d", ret);
    goto out;
  }

  rx_buf = (cf32_t *)malloc(sizeof(cf32_t) * rx_stream_cfg.buffer_size);
  if (!rx_buf) {
    log_msg(LOG_LEVEL_ERROR, "LIVE_SSB", "malloc failed");
    goto out;
  }
  buffs[0] = rx_buf;

  log_msg(LOG_LEVEL_INFO,
          "LIVE_SSB",
          "GSCN search: n=%zu profile=%d fs=%.3f MHz blocks/gscn=%u stable_need=%u",
          n_cand,
          (int)profile,
          nr_cfg.sample_rate_hz * 1e-6,
          blocks_per_gscn,
          stable_need);

  for (size_t gi = 0; gi < n_cand; gi++) {
    uint32_t g = cand[gi];
    double ss_hz = 0.0;
    uint32_t bi;
    int stable_pci = -1;
    uint32_t stable_run = 0u;

    if (nr_gscn_fr1_lt_3ghz_to_ss_ref_hz(g, &ss_hz) != TOA_OK)
      continue;

    rx_cfg.center_freq = ss_hz;
    if (caps_ok)
      clamp_rx_to_caps(&rx_cfg, &caps);

    ret = device.trx_configure_rx(&device, &rx_cfg);
    if (ret != TOA_OK) {
      log_msg(LOG_LEVEL_WARN, "LIVE_SSB", "GSCN %u tune failed ret=%d", g, ret);
      continue;
    }

    if (settle_ms > 0u)
      usleep((useconds_t)(settle_ms * 1000u));

    if (warmup_blocks > 0u) {
      ret = discard_rx(&device, buffs, rx_stream_cfg.buffer_size, warmup_blocks);
      if (ret != 0) {
        log_msg(LOG_LEVEL_WARN, "LIVE_SSB", "GSCN %u warmup failed", g);
        continue;
      }
    }

    nr_ssb_sync_reset(&ssb_ctx);
    stable_pci = -1;
    stable_run = 0u;

    log_msg(LOG_LEVEL_INFO,
            "LIVE_SSB",
            "try GSCN=%u ss_ref=%.3f MHz (fc=%.3f MHz)",
            g,
            ss_hz * 1e-6,
            rx_cfg.center_freq * 1e-6);

    for (bi = 0; bi < blocks_per_gscn; bi++) {
      radio_rx_result_t rr;
      nr_ssb_result_t res;
      int64_t t0;

      if (reset_each_block)
        nr_ssb_sync_reset(&ssb_ctx);

      memset(&rr, 0, sizeof(rr));
      ret = device.trx_read_func(&device,
                                 buffs,
                                 rx_stream_cfg.buffer_size,
                                 &rr,
                                 200000);
      if (ret != 0 || rr.samples_read <= 0) {
        log_msg(LOG_LEVEL_WARN, "LIVE_SSB", "read fail gscn=%u block=%u", g, bi);
        nr_ssb_sync_reset(&ssb_ctx);
        stable_pci = -1;
        stable_run = 0u;
        continue;
      }

      t0 = rr.timestamp_ns;

      memset(&res, 0, sizeof(res));
      ret = nr_ssb_sync_scan_block(&ssb_ctx,
                                   rx_buf,
                                   (uint32_t)rr.samples_read,
                                   t0,
                                   &res);
      if (ret != TOA_OK) {
        log_msg(LOG_LEVEL_WARN, "LIVE_SSB", "scan_block ret=%d", ret);
        continue;
      }

      if (!res.found) {
        nr_ssb_sync_reset(&ssb_ctx);
        stable_pci = -1;
        stable_run = 0u;
      }

      if (fp_csv) {
        fprintf(fp_csv,
                "%llu,%u,%.6f,%d,%d,%d,%d,%u,%u,%lld,%.3f,%.6f,%.6f,%.6f,%.6f\n",
                (unsigned long long)frame_id,
                g,
                ss_hz * 1e-6,
                res.found,
                res.nid2,
                res.nid1,
                res.pci,
                res.peak_pos,
                res.sss_pos,
                (long long)res.toa_ns,
                res.cfo_hz,
                res.metric,
                res.pss_peak_ratio,
                res.sss_metric,
                res.sss_peak_ratio);
      }
      frame_id++;

      if (res.found && res.pci >= 0) {
        if (res.pci == stable_pci)
          stable_run++;
        else {
          stable_pci = res.pci;
          stable_run = 1u;
        }

        log_msg(LOG_LEVEL_INFO,
                "LIVE_SSB",
                "  pci=%d cfo=%.1f Hz pss=%.4f sss=%.4f (stable %u/%u)",
                res.pci,
                (double)res.cfo_hz,
                (double)res.metric,
                (double)res.sss_metric,
                stable_run,
                stable_need);

        if (stable_run >= stable_need) {
          log_msg(LOG_LEVEL_INFO,
                  "LIVE_SSB",
                  "LOCK gscn=%u pci=%d ss_ref=%.3f MHz after %u blocks this GSCN",
                  g,
                  res.pci,
                  ss_hz * 1e-6,
                  bi + 1u);
          rc = 0;
          if (stop_on_lock)
            goto out;
          break;
        }
      }
    }
  }

  if (rc != 0)
    log_msg(LOG_LEVEL_WARN, "LIVE_SSB", "scan finished without stable PCI lock");

out:
  if (fp_csv)
    fclose(fp_csv);
  free(rx_buf);
  if (device.rx_running)
    (void)device.trx_stop_rx_stream(&device);
  if (device.is_open)
    device.dev_close(&device);
  return rc;
}
