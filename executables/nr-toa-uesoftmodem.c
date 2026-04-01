#include "executables/nr-toa-softmodem.h"
#include "executables/nr-toa-threads.h"
#include "openair1/PHY/NR_POSITIONING/nr_pos_api.h"

#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>

volatile int oai_exit;
#define NR_SYNC_LOST_MISS_THRESH 24U
#define NR_FREQ_SWEEP_STEP_ITERS 20U
#define NR_FREQ_SWEEP_STEP_ITERS_WIDE 5U
#define NR_FREQ_SWEEP_BEST_DWELL 1U
#define NR_GAIN_STEP_FULL_SWEEPS 1U
#define NR_FREQ_SWEEP_NEARMISS_DWELL 3U
#define NR_FREQ_SWEEP_HOT_DECAY 0.82f
#define NR_FREQ_SWEEP_GAIN_MEMORY 0.72f
#define NR_UE_LOOP_LOG_EVERY 1000U

typedef struct {
  unsigned points;
  double *freqs_hz;
  long *gscn;
} nr_freq_sweep_plan_t;

typedef struct {
  unsigned count;
  double values_db[16];
} nr_gain_plan_t;

static int nr_toa_build_sweep_plan(double center_freq_hz,
                                   double sample_rate_hz,
                                   nr_freq_sweep_plan_t *plan)
{
  const double gscn_grid_hz = 1.44e6;
  const double ssref0_hz = 3000e6;
  const double n78_lo_hz = 3300e6;
  const double n78_hi_hz = 3800e6;
  if (!plan) {
    return -1;
  }
  memset(plan, 0, sizeof(*plan));

  if (center_freq_hz >= n78_lo_hz && center_freq_hz <= n78_hi_hz) {
    const double target_step_hz = fmin(fmax(sample_rate_hz * 0.5, gscn_grid_hz), 5.76e6);
    const long gscn_step = (long)fmax(1.0, llround(target_step_hz / gscn_grid_hz));
    const long coarse_mult = 4L;
    const long coarse_step = gscn_step * coarse_mult;
    const long n_lo = (long)ceil((n78_lo_hz - ssref0_hz) / gscn_grid_hz);
    const long n_hi = (long)floor((n78_hi_hz - ssref0_hz) / gscn_grid_hz);
    const long n0 = (long)llround((center_freq_hz - ssref0_hz) / gscn_grid_hz);
    const unsigned raw_points = (unsigned)((n_hi - n_lo) + 1L);
    long *ordered_n = NULL;
    unsigned fill = 0U;
    if (raw_points == 0U) {
      return -1;
    }
    plan->freqs_hz = (double *)calloc(raw_points, sizeof(*plan->freqs_hz));
    plan->gscn = (long *)calloc(raw_points, sizeof(*plan->gscn));
    ordered_n = (long *)calloc(raw_points, sizeof(*ordered_n));
    if (!plan->freqs_hz || !plan->gscn || !ordered_n) {
      free(plan->freqs_hz);
      free(plan->gscn);
      free(ordered_n);
      memset(plan, 0, sizeof(*plan));
      return -1;
    }

    for (int pass = 0; pass < 2 && fill < raw_points; pass++) {
      const long step_n = (pass == 0) ? coarse_step : gscn_step;
      for (long d = 0L; fill < raw_points; d++) {
        const long cand[2] = {n0 - d * step_n, n0 + d * step_n};
        for (unsigned s = 0U; s < ((d == 0L) ? 1U : 2U); s++) {
          const long n = cand[s];
          int dup = 0;
          if (n < n_lo || n > n_hi) {
            continue;
          }
          for (unsigned k = 0U; k < fill; k++) {
            if (ordered_n[k] == n) {
              dup = 1;
              break;
            }
          }
          if (!dup) {
            ordered_n[fill++] = n;
          }
        }
        if ((n0 - d * step_n) < n_lo && (n0 + d * step_n) > n_hi) {
          break;
        }
      }
    }

    plan->points = fill;
    for (unsigned i = 0U; i < fill; i++) {
      plan->gscn[i] = 7499L + ordered_n[i];
      plan->freqs_hz[i] = ssref0_hz + (double)ordered_n[i] * gscn_grid_hz;
    }
    free(ordered_n);
    return 0;
  }

  {
    const unsigned raw_points = 17U;
    const long n0 = (long)llround((center_freq_hz - ssref0_hz) / gscn_grid_hz);
    plan->points = raw_points;
    plan->freqs_hz = (double *)calloc(raw_points, sizeof(*plan->freqs_hz));
    plan->gscn = (long *)calloc(raw_points, sizeof(*plan->gscn));
    if (!plan->freqs_hz || !plan->gscn) {
      free(plan->freqs_hz);
      free(plan->gscn);
      memset(plan, 0, sizeof(*plan));
      return -1;
    }
    for (unsigned i = 0U; i < raw_points; i++) {
      long off = 0;
      if (i > 0U) {
        const long k = (long)((i + 1U) / 2U);
        off = (i & 1U) ? -k : k;
      }
      plan->gscn[i] = 7499L + n0 + off;
      plan->freqs_hz[i] = ssref0_hz + (double)(n0 + off) * gscn_grid_hz;
    }
    return 0;
  }
}

static void nr_toa_free_sweep_plan(nr_freq_sweep_plan_t *plan)
{
  if (!plan) {
    return;
  }
  free(plan->freqs_hz);
  free(plan->gscn);
  memset(plan, 0, sizeof(*plan));
}

static void nr_toa_build_gain_plan(double base_gain_db, nr_gain_plan_t *plan)
{
  double g = 0.0;
  if (!plan) {
    return;
  }
  memset(plan, 0, sizeof(*plan));
  g = (base_gain_db >= 0.0) ? base_gain_db : 35.0;
  if (g < 20.0) {
    g = 20.0;
  }
  for (; g <= 65.0 && plan->count < (sizeof(plan->values_db) / sizeof(plan->values_db[0])); g += 3.0) {
    plan->values_db[plan->count++] = g;
  }
  if (plan->count == 0U) {
    plan->values_db[plan->count++] = (base_gain_db >= 0.0) ? base_gain_db : 35.0;
  }
}

static float nr_toa_sync_evidence_score(const nr_sync_state_t *sync)
{
  float score = 0.0f;
  if (!sync) {
    return 0.0f;
  }
  score = 0.60f * sync->pss_metric + 0.10f * sync->snr_db;
  if (sync->locked) {
    score += 0.04f;
  }
  switch ((nr_pbch_fail_stage_t)sync->pbch_fail_stage) {
    case NR_PBCH_FAIL_BCH:
      score += 1.10f * sync->pbch_metric + 0.12f;
      break;
    case NR_PBCH_FAIL_DMRS_AMBIG:
      score += 0.75f * sync->pbch_metric + 0.05f;
      break;
    case NR_PBCH_FAIL_DMRS_WEAK:
      score += 0.45f * sync->pbch_metric;
      break;
    default:
      score += 0.20f * sync->pbch_metric;
      break;
  }
  if (sync->pbch_ok) {
    score += 2.0f;
  }
  if (sync->mib_ok) {
    score += 4.0f;
  }
  if (score < 0.0f) {
    score = 0.0f;
  }
  return score;
}

static int nr_toa_sync_is_nearmiss(const nr_sync_state_t *sync)
{
  if (!sync) {
    return 0;
  }
  if (sync->pbch_fail_stage == NR_PBCH_FAIL_BCH && sync->pbch_metric >= 0.17f) {
    return 1;
  }
  if (sync->pbch_fail_stage == NR_PBCH_FAIL_DMRS_AMBIG &&
      sync->pbch_metric >= 0.24f &&
      sync->pss_metric >= 0.20f &&
      sync->snr_db >= 1.0f) {
    return 1;
  }
  if (sync->pbch_ok || sync->mib_ok) {
    return 1;
  }
  return 0;
}

static unsigned nr_toa_select_focus_index(const float *metric,
                                          const float *heat,
                                          const float *pbch_best,
                                          const uint16_t *near_hits,
                                          unsigned points)
{
  unsigned best_i = 0U;
  float best_score = -1.0f;
  if (!metric || !heat || !pbch_best || !near_hits || points == 0U) {
    return 0U;
  }
  for (unsigned i = 0U; i < points; i++) {
    const float score = 0.50f * metric[i] +
                        0.95f * heat[i] +
                        0.55f * pbch_best[i] +
                        0.03f * (float)near_hits[i];
    if (score > best_score) {
      best_score = score;
      best_i = i;
    }
  }
  return best_i;
}

static unsigned nr_toa_find_gscn_index(const nr_freq_sweep_plan_t *plan,
                                       int32_t gscn,
                                       unsigned fallback_idx)
{
  if (!plan || plan->points == 0U || !plan->gscn) {
    return fallback_idx;
  }
  if (gscn < 0) {
    return fallback_idx;
  }
  for (unsigned i = 0U; i < plan->points; i++) {
    if ((int32_t)plan->gscn[i] == gscn) {
      return i;
    }
  }
  return fallback_idx;
}

static void nr_toa_sig_handler(int signo)
{
  (void)signo;
  oai_exit = 1;
}

static void *sync_actor_thread(void *arg)
{
  PHY_VARS_NR_TOA_UE *UE = (PHY_VARS_NR_TOA_UE *)arg;
  uint64_t sync_cnt = 0;
  uint64_t detect_total = 0;
  uint64_t nodetect_total = 0;
  uint32_t miss_streak = 0;
  uint8_t prev_locked = 0;

  while (!oai_exit) {
    nr_iq_block_t *blk = NULL;
    uint64_t sync_job_id = 0;

    pthread_mutex_lock(&UE->sync_mtx);
    while (!oai_exit && UE->sync_q_count == 0U) {
      pthread_cond_wait(&UE->sync_cv, &UE->sync_mtx);
    }
    if (oai_exit) {
      pthread_mutex_unlock(&UE->sync_mtx);
      break;
    }

    blk = UE->sync_q[UE->sync_q_head];
    sync_job_id = UE->sync_q_job_id[UE->sync_q_head];
    UE->sync_q[UE->sync_q_head] = NULL;
    UE->sync_q_head = (UE->sync_q_head + 1U) % (uint32_t)NR_SYNC_Q_DEPTH;
    UE->sync_q_count--;
    UE->sync_job_id = sync_job_id;
    UE->sync_job_blk = blk;
    UE->sync_job_pending = (UE->sync_q_count > 0U) ? 1 : 0;
    pthread_mutex_unlock(&UE->sync_mtx);

    nr_sync_state_t local = UE->sync;
    if (blk && blk->rx[0] && (sync_cnt % 50U) == 0U) {
      double p = 0.0;
      double peak = 0.0;
      for (uint32_t n = 0; n < blk->nsamps; n++) {
        double i = (double)blk->rx[0][n].r;
        double q = (double)blk->rx[0][n].i;
        double e = i * i + q * q;
        p += e;
        if (e > peak) {
          peak = e;
        }
      }
      p /= (double)(blk->nsamps ? blk->nsamps : 1U);
      printf("sync_actor: ts=%llu nsamps=%u avg_pwr=%.1f peak=%.1f\n",
             (unsigned long long)blk->ts_first, blk->nsamps, p, peak);
    }
    if (UE->provider && UE->provider->acquire) {
      int rc = UE->provider->acquire(UE->provider_ctx, blk, &local);
      if (rc != 0) {
        nodetect_total++;
        miss_streak++;
        if (miss_streak >= NR_SYNC_LOST_MISS_THRESH) {
          local.locked = 0;
        }
      } else {
        detect_total++;
        miss_streak = 0;
      }
    }
    if (!prev_locked && local.locked) {
      printf("SSB_LOCK_EVENT: pci=%u offset=%d cfo=%.2f snr=%.2f metric=%.3f pbch=%u mib=%u sfn=%u mib_payload=0x%06x det=%llu nodet=%llu\n",
             (unsigned)local.pci, local.coarse_offset_samp, local.cfo_hz,
             local.snr_db, local.pss_metric,
             (unsigned)local.pbch_ok, (unsigned)local.mib_ok,
             (unsigned)local.sfn, (unsigned)local.mib_payload,
             (unsigned long long)detect_total,
             (unsigned long long)nodetect_total);
    }
    prev_locked = local.locked;
    if ((sync_cnt % 50U) == 0U) {
      printf("sync_result: %s ssb_locked=%u pbch=%u mib=%u pci=%u ssb=%u sfn=%u mib_payload=0x%06x offset=%d cfo=%.2f snr=%.2f metric=%.3f miss_streak=%u det=%llu nodet=%llu dropped_sync=%llu dropped_meas=%llu dropped_solver=%llu\n",
             local.locked ? "detected" : "not-detected",
             (unsigned)local.locked, (unsigned)local.pbch_ok,
             (unsigned)local.mib_ok, (unsigned)local.pci,
             (unsigned)local.ssb_index, (unsigned)local.sfn,
             (unsigned)local.mib_payload, local.coarse_offset_samp,
             local.cfo_hz, local.snr_db, local.pss_metric, (unsigned)miss_streak,
             (unsigned long long)detect_total, (unsigned long long)nodetect_total,
             (unsigned long long)UE->sync_jobs_dropped,
             (unsigned long long)UE->meas_jobs_dropped,
             (unsigned long long)UE->solver_jobs_dropped);
    }
    sync_cnt++;

    pthread_mutex_lock(&UE->sync_mtx);
    UE->sync = local;
    UE->sync_job_done = 1;
    pthread_cond_signal(&UE->sync_cv);
    pthread_mutex_unlock(&UE->sync_mtx);

    nr_iq_block_put(blk);
  }

  return NULL;
}

static void *measure_actor_thread(void *arg)
{
  PHY_VARS_NR_TOA_UE *UE = (PHY_VARS_NR_TOA_UE *)arg;

  while (!oai_exit) {
    nr_iq_block_t *blk = NULL;
    nr_sync_state_t local_sync;
    uint64_t meas_job_id = 0;

    pthread_mutex_lock(&UE->meas_mtx);
    while (!oai_exit && UE->meas_job_pending == 0) {
      pthread_cond_wait(&UE->meas_cv, &UE->meas_mtx);
    }
    if (oai_exit) {
      pthread_mutex_unlock(&UE->meas_mtx);
      break;
    }

    blk = UE->meas_job_blk;
    local_sync = UE->meas_job_sync_snapshot;
    meas_job_id = UE->meas_job_id;
    UE->meas_job_blk = NULL;
    UE->meas_job_pending = 0;
    pthread_mutex_unlock(&UE->meas_mtx);

    /* Track & extract measurement on a local sync snapshot. */
    if (UE->provider && UE->provider->track) {
      (void)UE->provider->track(UE->provider_ctx, blk, &local_sync);
    }
    if (nr_ssb_check_lost_lock(&local_sync) != 0) {
      local_sync.locked = 0;
    }

    nr_toa_meas_t meas;
    memset(&meas, 0, sizeof(meas));
    if (UE->provider && UE->provider->extract_meas) {
      (void)UE->provider->extract_meas(UE->provider_ctx, blk, &local_sync,
                                         &meas);
    }

    pthread_mutex_lock(&UE->sync_mtx);
    UE->sync = local_sync;
    pthread_mutex_unlock(&UE->sync_mtx);

    /* Always enqueue to solver queue (even when meas.valid==0 in Phase-0). */
    pthread_mutex_lock(&UE->solver_mtx);
    if (UE->solver_q_count >= NR_SOLVER_Q_DEPTH) {
      UE->solver_jobs_dropped++;
    } else {
      UE->solver_q_job_id[UE->solver_q_tail] = meas_job_id;
      UE->solver_q[UE->solver_q_tail] = meas;
      UE->solver_q_tail =
          (UE->solver_q_tail + 1U) % (uint32_t)NR_SOLVER_Q_DEPTH;
      UE->solver_q_count++;
      pthread_cond_signal(&UE->solver_cv);
    }
    pthread_mutex_unlock(&UE->solver_mtx);

    pthread_mutex_lock(&UE->meas_mtx);
    UE->meas_job_done = 1;
    pthread_cond_signal(&UE->meas_cv);
    pthread_mutex_unlock(&UE->meas_mtx);

    nr_iq_block_put(blk);
  }

  return NULL;
}

static void *solver_actor_thread(void *arg)
{
  PHY_VARS_NR_TOA_UE *UE = (PHY_VARS_NR_TOA_UE *)arg;

  while (!oai_exit) {
    nr_toa_meas_t meas;
    uint64_t meas_job_id = 0;

    pthread_mutex_lock(&UE->solver_mtx);
    while (!oai_exit && UE->solver_q_count == 0) {
      pthread_cond_wait(&UE->solver_cv, &UE->solver_mtx);
    }
    if (oai_exit) {
      pthread_mutex_unlock(&UE->solver_mtx);
      break;
    }

    meas = UE->solver_q[UE->solver_q_head];
    meas_job_id = UE->solver_q_job_id[UE->solver_q_head];
    UE->solver_q_head =
        (UE->solver_q_head + 1U) % (uint32_t)NR_SOLVER_Q_DEPTH;
    UE->solver_q_count--;
    pthread_mutex_unlock(&UE->solver_mtx);

    /* Phase-0: epoch aggregation + call the (stub) WLS solver. */
    (void)nr_epoch_mgr_push(&UE->epoch_mgr, &meas);
    nr_toa_epoch_t epoch;
    memset(&epoch, 0, sizeof(epoch));
    if (nr_epoch_mgr_pop_ready(&UE->epoch_mgr, &epoch) == 0) {
      nr_solver_input_t in;
      nr_loc_solution_t sol;
      (void)nr_pos_build_equations(&epoch, &in);
      (void)nr_pos_solve_wls(&in, &sol);
      (void)nr_pos_validate_solution(&sol);
      (void)nr_trace_solution(&sol);
      if (sol.valid) {
        printf("solver_thread: job=%llu epoch=%llu num_meas=%u sol.valid=%u\n",
               (unsigned long long)meas_job_id,
               (unsigned long long)epoch.epoch_id,
               (unsigned)epoch.num_meas,
               (unsigned)sol.valid);
      }
    }
  }

  return NULL;
}

static void *TOA_UE_thread(void *arg)
{
  PHY_VARS_NR_TOA_UE *UE = (PHY_VARS_NR_TOA_UE *)arg;

  const uint32_t ssb_need = nr_v0_ssb_burst_len_fs(UE->app_cfg.sample_rate_hz);
  const uint32_t nsamps = (ssb_need > 4096U) ? (2U * ssb_need) : 4096U;
  UE->samples_per_slot = nsamps;
  UE->samples_per_frame = 2 * nsamps;
  UE->abs_samp_wr = 0;
  unsigned iter = 0;
  unsigned presync_no_lock_iter = 0;
  unsigned presync_hold_after_lock = 0;
  unsigned sweep_idx = 0;
  unsigned sweep_rounds = 0;
  unsigned sweep_hold = 0;
  unsigned sweep_step_iters = NR_FREQ_SWEEP_STEP_ITERS;
  unsigned gain_idx = 0U;
  unsigned gain_full_sweeps = 0U;
  unsigned hot_dwell_left = 0U;
  unsigned hot_focus_idx = 0U;
  nr_freq_sweep_plan_t sweep_plan;
  nr_gain_plan_t gain_plan;
  float *sweep_best_metric = NULL;
  float *sweep_heat = NULL;
  float *sweep_pbch_best = NULL;
  uint16_t *sweep_near_hits = NULL;
  memset(&sweep_plan, 0, sizeof(sweep_plan));
  memset(&gain_plan, 0, sizeof(gain_plan));
  if (nr_toa_build_sweep_plan(UE->app_cfg.center_freq_hz,
                              UE->app_cfg.sample_rate_hz,
                              &sweep_plan) != 0 ||
      sweep_plan.points == 0U) {
    printf("freq_sweep: failed to build sweep plan around %.0f Hz\n",
           UE->app_cfg.center_freq_hz);
    return NULL;
  }
  sweep_best_metric = (float *)calloc(sweep_plan.points, sizeof(*sweep_best_metric));
  if (!sweep_best_metric) {
    nr_toa_free_sweep_plan(&sweep_plan);
    return NULL;
  }
  sweep_heat = (float *)calloc(sweep_plan.points, sizeof(*sweep_heat));
  sweep_pbch_best = (float *)calloc(sweep_plan.points, sizeof(*sweep_pbch_best));
  sweep_near_hits = (uint16_t *)calloc(sweep_plan.points, sizeof(*sweep_near_hits));
  if (!sweep_heat || !sweep_pbch_best || !sweep_near_hits) {
    free(sweep_best_metric);
    free(sweep_heat);
    free(sweep_pbch_best);
    free(sweep_near_hits);
    nr_toa_free_sweep_plan(&sweep_plan);
    return NULL;
  }
  if (sweep_plan.points > 33U) {
    sweep_step_iters = NR_FREQ_SWEEP_STEP_ITERS_WIDE;
  }
  nr_toa_build_gain_plan(UE->app_cfg.rx_gain_db, &gain_plan);
  printf("freq_sweep: plan_points=%u center=%.0fHz first=%.0fHz last=%.0fHz step_iters=%u\n",
         sweep_plan.points, UE->app_cfg.center_freq_hz,
         sweep_plan.freqs_hz[0],
         sweep_plan.freqs_hz[sweep_plan.points - 1U],
         sweep_step_iters);
  printf("rx_gain_plan: count=%u start=%.1f end=%.1f current=%.1f\n",
         gain_plan.count,
         gain_plan.values_db[0],
         gain_plan.values_db[gain_plan.count - 1U],
         UE->app_cfg.rx_gain_db);
  fflush(stdout);
  UE->current_rx_freq_hz = UE->app_cfg.center_freq_hz;
  UE->current_rx_gscn = (int32_t)sweep_plan.gscn[0];

  while (!oai_exit) {
    switch (UE->state) {
    case TOA_STATE_WAIT_CLOCK:
      if (nr_toa_clock_ready(UE) == 1) {
        UE->state = TOA_STATE_PRESYNC;
      }
      break;

    case TOA_STATE_PRESYNC: {
      nr_sync_state_t cur_sync;
      pthread_mutex_lock(&UE->sync_mtx);
      int sync_backpressure = (UE->sync_q_count >= (NR_SYNC_Q_DEPTH / 2U));
      int has_real_lock = (UE->sync.locked &&
                           UE->sync.pbch_ok &&
                           UE->sync.pbch_confirmed &&
                           UE->sync.mib_ok);
      cur_sync = UE->sync;
      float cur_metric = cur_sync.pss_metric;
      float cur_evidence = nr_toa_sync_evidence_score(&cur_sync);
      int cur_nearmiss = nr_toa_sync_is_nearmiss(&cur_sync);
      unsigned evidence_idx = nr_toa_find_gscn_index(&sweep_plan, cur_sync.last_gscn, sweep_idx);
      pthread_mutex_unlock(&UE->sync_mtx);
      if (has_real_lock) {
        presync_hold_after_lock = 800U; /* Hold current freq after a lock event */
      }
      if (cur_metric > sweep_best_metric[evidence_idx]) {
        sweep_best_metric[evidence_idx] = cur_metric;
      }
      const float prev_heat = sweep_heat[evidence_idx];
      const float prev_pbch_best = sweep_pbch_best[evidence_idx];
      sweep_heat[evidence_idx] = fmaxf(cur_evidence, sweep_heat[evidence_idx] * NR_FREQ_SWEEP_HOT_DECAY);
      if (cur_sync.pbch_metric > sweep_pbch_best[evidence_idx]) {
        sweep_pbch_best[evidence_idx] = cur_sync.pbch_metric;
      }
      if (cur_nearmiss &&
          (cur_evidence > (prev_heat + 0.015f) ||
           cur_sync.pbch_metric > (prev_pbch_best + 0.010f) ||
           sweep_near_hits[evidence_idx] == 0U)) {
        if (sweep_near_hits[evidence_idx] < 65535U) {
          sweep_near_hits[evidence_idx]++;
        }
        if (hot_dwell_left < NR_FREQ_SWEEP_NEARMISS_DWELL) {
          hot_dwell_left = NR_FREQ_SWEEP_NEARMISS_DWELL;
          hot_focus_idx = evidence_idx;
          printf("freq_hotspot: idx=%u gscn=%ld rx_freq=%.0fHz score=%.3f pci=%u stage=%u near=%u\n",
                 evidence_idx,
                 (long)sweep_plan.gscn[evidence_idx],
                 sweep_plan.freqs_hz[evidence_idx],
                 cur_evidence,
                 (unsigned)cur_sync.pci,
                 (unsigned)cur_sync.pbch_fail_stage,
                 (unsigned)sweep_near_hits[evidence_idx]);
        }
      }
      if (!has_real_lock) {
        presync_no_lock_iter++;
        if (presync_hold_after_lock > 0U) {
          presync_hold_after_lock--;
        }
        if ((presync_no_lock_iter % sweep_step_iters) == 0U) {
          if (presync_hold_after_lock > 0U) {
            break;
          }
          if (hot_dwell_left > 0U) {
            hot_dwell_left--;
            sweep_idx = hot_focus_idx;
            if ((iter % 64U) == 0U) {
              printf("freq_hold: idx=%u gscn=%ld rx_freq=%.0fHz dwell_left=%u heat=%.3f pbch=%.3f near=%u\n",
                     sweep_idx,
                     (long)sweep_plan.gscn[sweep_idx],
                     sweep_plan.freqs_hz[sweep_idx],
                     hot_dwell_left,
                     sweep_heat[sweep_idx],
                     sweep_pbch_best[sweep_idx],
                     (unsigned)sweep_near_hits[sweep_idx]);
            }
          } else if (sweep_rounds < sweep_plan.points) {
            sweep_idx = (sweep_idx + 1U) % sweep_plan.points;
            sweep_rounds++;
          } else {
            unsigned best_i = nr_toa_select_focus_index(sweep_best_metric,
                                                        sweep_heat,
                                                        sweep_pbch_best,
                                                        sweep_near_hits,
                                                        sweep_plan.points);
            sweep_idx = best_i;
            if (sweep_hold < NR_FREQ_SWEEP_BEST_DWELL) {
              sweep_hold++;
              if (sweep_hold == 1U) {
                gain_full_sweeps++;
              }
            } else if ((gain_idx + 1U) < gain_plan.count &&
                       gain_full_sweeps >= NR_GAIN_STEP_FULL_SWEEPS) {
              gain_idx++;
              if (nr_toa_radio_set_rx_gain(UE->dev, gain_plan.values_db[gain_idx]) == 0) {
                UE->app_cfg.rx_gain_db = gain_plan.values_db[gain_idx];
                UE->rf_cfg.rx_gain_db = gain_plan.values_db[gain_idx];
                printf("rx_gain_step: idx=%u gain=%.1f dB after_full_sweep=%u\n",
                       gain_idx, gain_plan.values_db[gain_idx], gain_full_sweeps);
                fflush(stdout);
              }
              for (unsigned i = 0U; i < sweep_plan.points; i++) {
                sweep_best_metric[i] *= 0.50f;
                sweep_heat[i] *= NR_FREQ_SWEEP_GAIN_MEMORY;
                sweep_pbch_best[i] *= NR_FREQ_SWEEP_GAIN_MEMORY;
                sweep_near_hits[i] = (uint16_t)(sweep_near_hits[i] / 2U);
              }
              sweep_idx = 0U;
              sweep_rounds = 0U;
              sweep_hold = 0U;
              gain_full_sweeps = 0U;
              hot_dwell_left = 0U;
              hot_focus_idx = 0U;
            } else {
              if ((gain_idx + 1U) >= gain_plan.count) {
                sweep_idx = best_i;
              }
            }
          }
          if (nr_toa_radio_set_rx_freq(UE->dev, sweep_plan.freqs_hz[sweep_idx]) == 0) {
            UE->current_rx_freq_hz = sweep_plan.freqs_hz[sweep_idx];
            UE->current_rx_gscn = (int32_t)sweep_plan.gscn[sweep_idx];
            printf("freq_sweep(GSCN): idx=%u gscn=%ld rx_freq=%.0fHz best_metric=%.3f\n",
                   sweep_idx, (long)sweep_plan.gscn[sweep_idx], sweep_plan.freqs_hz[sweep_idx],
                   sweep_best_metric[sweep_idx]);
          }
        }
      } else {
        presync_no_lock_iter = 0;
      }
      if (sync_backpressure) {
        /* Allow sweep scheduling to continue even when actor queues are busy. */
        break;
      }
      (void)nr_toa_read_two_frames(UE, &UE->iq_ring);
      pthread_mutex_lock(&UE->sync_mtx);
      UE->state = (UE->sync.locked &&
                   UE->sync.pbch_ok &&
                   UE->sync.pbch_confirmed &&
                   UE->sync.mib_ok)
                      ? TOA_STATE_LOCKED
                      : TOA_STATE_PRESYNC;
      pthread_mutex_unlock(&UE->sync_mtx);
      break;
    }

    case TOA_STATE_LOCKED:
    case TOA_STATE_MEASURING:
      (void)nr_toa_read_one_slot(UE, &UE->iq_ring);
      pthread_mutex_lock(&UE->sync_mtx);
      if (!UE->sync.locked) {
        UE->state = TOA_STATE_RESYNC;
      }
      pthread_mutex_unlock(&UE->sync_mtx);
      break;

    case TOA_STATE_RESYNC:
      nr_toa_reset_tracking(UE);
      UE->state = TOA_STATE_PRESYNC;
      break;

    default:
      break;
    }

    iter++;
    if ((iter % NR_UE_LOOP_LOG_EVERY) == 0U) {
      pthread_mutex_lock(&UE->sync_mtx);
      printf("UE loop: state=%d locked=%u cum_tracking_shift_samp=%lld dropped_sync=%llu dropped_meas=%llu dropped_solver=%llu\n",
             (int)UE->state, (unsigned)UE->sync.locked,
             (long long)UE->sync.cum_tracking_shift_samp,
             (unsigned long long)UE->sync_jobs_dropped,
             (unsigned long long)UE->meas_jobs_dropped,
             (unsigned long long)UE->solver_jobs_dropped);
      pthread_mutex_unlock(&UE->sync_mtx);
    }
    /* Give actor threads some CPU while keeping RX mostly continuous. */
    usleep(200);
  }

  free(sweep_best_metric);
  free(sweep_heat);
  free(sweep_pbch_best);
  free(sweep_near_hits);
  nr_toa_free_sweep_plan(&sweep_plan);
  (void)TOA_THREAD_TOA_UE;
  return NULL;
}

int main(int argc, char **argv)
{
  PHY_VARS_NR_TOA_UE ue;
  memset(&ue, 0, sizeof(ue));
  oai_exit = 0;
  ue.state = TOA_STATE_WAIT_CLOCK;
  ue.provider_ctx = (void *)&ue;

  const char *cfgpath =
      (argc > 1) ? argv[1]
                 : "targets/PROJECTS/NR-TOA/CONF/ue.toa.ssb.usrpb210.conf";
  if (nr_toa_load_config(cfgpath, &ue.app_cfg) != 0) {
    return 1;
  }
  ue.provider = (ue.app_cfg.mode == NR_TOA_MODE_PRS_TOA) ? &nr_prs_provider : &nr_ssb_provider;

  if (ue.app_cfg.anchor_db_path[0] != '\0') {
    int n = 0;
    if (nr_toa_load_anchor_db(ue.app_cfg.anchor_db_path, ue.anchors, &n) == 0) {
      ue.n_anchors = n;
    }
  }

  printf("NR-TOA UE cfg: sdr=%s clock=%s time=%s f=%0.f Fs=%0.f rx_gain=%0.1f tx_gain=%0.1f mode=%u meas_mode=%u anchor_db_path=%s n_anchors=%d\n",
         ue.app_cfg.sdr,
         ue.app_cfg.clock_source,
         ue.app_cfg.time_source,
         ue.app_cfg.center_freq_hz,
         ue.app_cfg.sample_rate_hz,
         ue.app_cfg.rx_gain_db,
         ue.app_cfg.tx_gain_db,
         (unsigned)ue.app_cfg.mode,
         (unsigned)ue.app_cfg.meas_mode,
         ue.app_cfg.anchor_db_path,
         ue.n_anchors);
  if (ue.app_cfg.mode == NR_TOA_MODE_SSB_TOA) {
    printf("NR-TOA note: SSB mode now attempts real PBCH/MIB decode; treat ssb_locked=1 with mib=0 as a decode failure, not a successful cell accept.\n");
  }
  if (ue.app_cfg.iq_dump_enable) {
    const char *dump_dir = getenv("NR_TOA_IQ_DUMP_DIR");
    printf("NR-TOA note: IQ near-miss dump enabled, dir=%s\n",
           (dump_dir && dump_dir[0] != '\0') ? dump_dir : "/tmp/nr_toa_iq");
  }
  if (nr_toa_build_rf_cfg(&ue.app_cfg, &ue.rf_cfg) != 0) {
    return 1;
  }
  if (nr_toa_radio_init(&ue.dev, &ue.rf_cfg) != 0) {
    return 1;
  }
  if (nr_toa_radio_start(ue.dev) != 0) {
    return 1;
  }
  if (nr_iq_ring_init(&ue.iq_ring, 64) != 0) {
    return 1;
  }
  if (ue.provider->init(ue.provider_ctx) != 0) {
    return 1;
  }

  signal(SIGINT, nr_toa_sig_handler);
  signal(SIGTERM, nr_toa_sig_handler);

  /* Initialize actor/job synchronization primitives. */
  if (pthread_mutex_init(&ue.sync_mtx, NULL) != 0) {
    return 1;
  }
  pthread_cond_init(&ue.sync_cv, NULL);
  ue.sync_job_pending = 0;
  ue.sync_job_done = 0;
  ue.sync_job_blk = NULL;
  ue.sync_job_id = 0;
  ue.sync_q_head = 0;
  ue.sync_q_tail = 0;
  ue.sync_q_count = 0;
  memset(ue.sync_q, 0, sizeof(ue.sync_q));
  memset(ue.sync_q_job_id, 0, sizeof(ue.sync_q_job_id));
  ue.next_sync_job_id = 1;
  ue.sync_jobs_dropped = 0;

  if (pthread_mutex_init(&ue.meas_mtx, NULL) != 0) {
    return 1;
  }
  pthread_cond_init(&ue.meas_cv, NULL);
  ue.meas_job_pending = 0;
  ue.meas_job_done = 0;
  ue.meas_job_blk = NULL;
  ue.meas_job_id = 0;
  memset(&ue.meas_job_sync_snapshot, 0, sizeof(ue.meas_job_sync_snapshot));
  ue.next_meas_job_id = 1;
  ue.meas_jobs_dropped = 0;

  if (pthread_mutex_init(&ue.solver_mtx, NULL) != 0) {
    return 1;
  }
  pthread_cond_init(&ue.solver_cv, NULL);
  ue.solver_q_head = 0;
  ue.solver_q_tail = 0;
  ue.solver_q_count = 0;
  ue.next_solver_job_id = 1;
  ue.solver_jobs_dropped = 0;

  /* Start actor threads. */
  if (pthread_create(&ue.sync_tid, NULL, sync_actor_thread, &ue) != 0) {
    return 1;
  }
  if (pthread_create(&ue.meas_tid, NULL, measure_actor_thread, &ue) != 0) {
    return 1;
  }
  if (pthread_create(&ue.solver_tid, NULL, solver_actor_thread, &ue) != 0) {
    return 1;
  }

  /* Start UE orchestrator. */
  pthread_t tid;
  if (pthread_create(&tid, NULL, TOA_UE_thread, &ue) != 0) {
    return 1;
  }
  pthread_join(tid, NULL);

  /* Make sure actors are not stuck in cond waits. */
  pthread_mutex_lock(&ue.sync_mtx);
  ue.sync_job_pending = 0;
  while (ue.sync_q_count > 0U) {
    nr_iq_block_t *old = ue.sync_q[ue.sync_q_head];
    ue.sync_q[ue.sync_q_head] = NULL;
    ue.sync_q_head = (ue.sync_q_head + 1U) % (uint32_t)NR_SYNC_Q_DEPTH;
    ue.sync_q_count--;
    if (old) {
      nr_iq_block_put(old);
    }
  }
  pthread_cond_broadcast(&ue.sync_cv);
  pthread_mutex_unlock(&ue.sync_mtx);

  pthread_mutex_lock(&ue.meas_mtx);
  ue.meas_job_pending = 0;
  pthread_cond_broadcast(&ue.meas_cv);
  pthread_mutex_unlock(&ue.meas_mtx);

  pthread_mutex_lock(&ue.solver_mtx);
  ue.solver_q_count = 0;
  pthread_cond_broadcast(&ue.solver_cv);
  pthread_mutex_unlock(&ue.solver_mtx);

  pthread_join(ue.sync_tid, NULL);
  pthread_join(ue.meas_tid, NULL);
  pthread_join(ue.solver_tid, NULL);

  nr_iq_ring_free(&ue.iq_ring);
  return 0;
}
