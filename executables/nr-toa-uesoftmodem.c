#include "executables/nr-toa-softmodem.h"
#include "executables/nr-toa-threads.h"
#include "openair1/PHY/NR_POSITIONING/nr_pos_api.h"

#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>

volatile int oai_exit;

static void nr_toa_sig_handler(int signo)
{
  (void)signo;
  oai_exit = 1;
}

static void *sync_actor_thread(void *arg)
{
  PHY_VARS_NR_TOA_UE *UE = (PHY_VARS_NR_TOA_UE *)arg;
  uint64_t sync_cnt = 0;

  while (!oai_exit) {
    nr_iq_block_t *blk = NULL;

    pthread_mutex_lock(&UE->sync_mtx);
    while (!oai_exit && UE->sync_job_pending == 0) {
      pthread_cond_wait(&UE->sync_cv, &UE->sync_mtx);
    }
    if (oai_exit) {
      pthread_mutex_unlock(&UE->sync_mtx);
      break;
    }

    blk = UE->sync_job_blk;
    UE->sync_job_blk = NULL;
    UE->sync_job_pending = 0;
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
        local.locked = 0;
      }
    }
    if ((sync_cnt % 50U) == 0U) {
      printf("sync_result: %s locked=%u pci=%u offset=%d cfo=%.2f snr=%.2f\n",
             local.locked ? "detected" : "not-detected",
             (unsigned)local.locked, (unsigned)local.pci, local.coarse_offset_samp,
             local.cfo_hz, local.snr_db);
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
      printf("solver_thread: job=%llu epoch=%llu num_meas=%u sol.valid=%u\n",
             (unsigned long long)meas_job_id,
             (unsigned long long)epoch.epoch_id,
             (unsigned)epoch.num_meas,
             (unsigned)sol.valid);
    }
  }

  return NULL;
}

static void *TOA_UE_thread(void *arg)
{
  PHY_VARS_NR_TOA_UE *UE = (PHY_VARS_NR_TOA_UE *)arg;

  const uint32_t nsamps = 4096; /* Phase-0: fixed IQ window size. */
  UE->samples_per_slot = nsamps;
  UE->samples_per_frame = 2 * nsamps;
  UE->abs_samp_wr = 0;
  unsigned iter = 0;

  while (!oai_exit) {
    switch (UE->state) {
    case TOA_STATE_WAIT_CLOCK:
      if (nr_toa_clock_ready(UE) == 1) {
        UE->state = TOA_STATE_PRESYNC;
      }
      break;

    case TOA_STATE_PRESYNC:
      (void)nr_toa_read_two_frames(UE, &UE->iq_ring);
      pthread_mutex_lock(&UE->sync_mtx);
      UE->state = UE->sync.locked ? TOA_STATE_LOCKED : TOA_STATE_PRESYNC;
      pthread_mutex_unlock(&UE->sync_mtx);
      break;

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
    if ((iter % 200) == 0) {
      pthread_mutex_lock(&UE->sync_mtx);
      printf("UE loop: state=%d locked=%u cum_tracking_shift_samp=%lld\n",
             (int)UE->state, (unsigned)UE->sync.locked,
             (long long)UE->sync.cum_tracking_shift_samp);
      pthread_mutex_unlock(&UE->sync_mtx);
    }
    usleep(2000);
  }

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
                 : "targets/PROJECTS/NR-TOA/CONF/ue.toa.ssb.limesdr.conf";
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
