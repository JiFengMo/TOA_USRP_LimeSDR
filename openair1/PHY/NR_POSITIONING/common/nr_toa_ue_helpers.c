#include "openair1/PHY/NR_POSITIONING/nr_pos_types.h"
#include "openair1/PHY/NR_POSITIONING/nr_toa_ue.h"
#include "openair1/PHY/NR_POSITIONING/nr_pos_api.h"

#include <string.h>

int nr_toa_clock_ready(nr_toa_ue_t *ue)
{
  if (!ue || !ue->dev) {
    return 0;
  }
  if (nr_toa_wait_clock_lock(ue->dev, &ue->clk) != 0) {
    return 0;
  }
  return ue->clk.locked ? 1 : 0;
}

int nr_toa_read_two_frames(nr_toa_ue_t *ue, nr_iq_ring_t *ring)
{
  if (!ue || !ring || !ue->dev) {
    return -1;
  }

  uint32_t nsamps = ue->samples_per_slot ? ue->samples_per_slot : 4096;
  uint8_t rx_ant = (ue->app_cfg.rx_ant > 0) ? ue->app_cfg.rx_ant : 1;

  nr_iq_block_t *b1 = nr_iq_ring_alloc_ex(ring, nsamps, rx_ant);
  nr_iq_block_t *b2 = nr_iq_ring_alloc_ex(ring, nsamps, rx_ant);
  if (!b1 || !b2) {
    if (b1) {
      nr_iq_block_put(b1);
    }
    if (b2) {
      nr_iq_block_put(b2);
    }
    return -1;
  }

  /* Keep IQ processing in sync with the true RF sample rate. */
  b1->fs_hz = ue->rf_cfg.sample_rate;
  b2->fs_hz = ue->rf_cfg.sample_rate;

  if (nr_toa_radio_read(ue->dev, b1) != 0) {
    nr_iq_block_put(b1);
    nr_iq_block_put(b2);
    return -1;
  }
  if (nr_toa_radio_read(ue->dev, b2) != 0) {
    nr_iq_block_put(b1);
    nr_iq_block_put(b2);
    return -1;
  }

  nr_iq_ring_push(ring, b1);
  nr_iq_ring_push(ring, b2);

  ue->abs_samp_wr = b2->abs_samp0 + b2->nsamps;

  /* Enqueue sync job (drop-new policy if actor busy). */
  (void)nr_toa_enqueue_sync_job(ue, b2);
  return 0;
}

int nr_toa_read_one_slot(nr_toa_ue_t *ue, nr_iq_ring_t *ring)
{
  if (!ue || !ring || !ue->dev) {
    return -1;
  }

  uint32_t nsamps = ue->samples_per_slot ? ue->samples_per_slot : 4096;
  uint8_t rx_ant = (ue->app_cfg.rx_ant > 0) ? ue->app_cfg.rx_ant : 1;

  nr_iq_block_t *blk = nr_iq_ring_alloc_ex(ring, nsamps, rx_ant);
  if (!blk) {
    return -1;
  }

  /* Keep IQ processing in sync with the true RF sample rate. */
  blk->fs_hz = ue->rf_cfg.sample_rate;

  if (nr_toa_radio_read(ue->dev, blk) != 0) {
    nr_iq_block_put(blk);
    return -1;
  }

  nr_iq_ring_push(ring, blk);
  ue->abs_samp_wr = blk->abs_samp0 + blk->nsamps;

  /* Enqueue measure job (drop-new policy if actor busy). */
  (void)nr_toa_enqueue_measure_job(ue, blk);
  return 0;
}

int nr_toa_enqueue_sync_job(nr_toa_ue_t *ue, nr_iq_block_t *blk)
{
  if (!ue || !blk) {
    return -1;
  }

  pthread_mutex_lock(&ue->sync_mtx);
  if (ue->sync_q_count >= NR_SYNC_Q_DEPTH) {
    /* Keep freshest IQ: drop oldest sync job when queue is full. */
    nr_iq_block_t *old = ue->sync_q[ue->sync_q_head];
    ue->sync_q_head = (ue->sync_q_head + 1U) % (uint32_t)NR_SYNC_Q_DEPTH;
    ue->sync_q_count--;
    ue->sync_jobs_dropped++;
    if (old) {
      nr_iq_block_put(old);
    }
  }

  ue->sync_job_id = ue->next_sync_job_id++;
  ue->sync_q_job_id[ue->sync_q_tail] = ue->sync_job_id;
  ue->sync_q[ue->sync_q_tail] = blk;
  ue->sync_q_tail = (ue->sync_q_tail + 1U) % (uint32_t)NR_SYNC_Q_DEPTH;
  ue->sync_q_count++;
  ue->sync_job_pending = (ue->sync_q_count > 0U) ? 1 : 0;
  ue->sync_job_done = 0;

  /* Queue/actor owns one reference until popped and processed. */
  nr_iq_block_get(blk);

  pthread_cond_signal(&ue->sync_cv);
  pthread_mutex_unlock(&ue->sync_mtx);
  return 0;
}

int nr_toa_enqueue_measure_job(nr_toa_ue_t *ue, nr_iq_block_t *blk)
{
  if (!ue || !blk) {
    return -1;
  }

  /* Snapshot current synchronization state for measurement pipeline. */
  nr_sync_state_t snap;
  pthread_mutex_lock(&ue->sync_mtx);
  snap = ue->sync;
  pthread_mutex_unlock(&ue->sync_mtx);

  pthread_mutex_lock(&ue->meas_mtx);
  if (ue->meas_job_pending) {
    ue->meas_jobs_dropped++;
    pthread_mutex_unlock(&ue->meas_mtx);
    return -1;
  }

  ue->meas_job_id = ue->next_meas_job_id++;
  ue->meas_job_blk = blk;
  ue->meas_job_sync_snapshot = snap;
  ue->meas_job_pending = 1;
  ue->meas_job_done = 0;

  nr_iq_block_get(blk); /* Actor will put() after processing. */
  pthread_cond_signal(&ue->meas_cv);
  pthread_mutex_unlock(&ue->meas_mtx);
  return 0;
}

void nr_toa_reset_tracking(nr_toa_ue_t *ue)
{
  if (!ue) {
    return;
  }
  pthread_mutex_lock(&ue->sync_mtx);
  memset(&ue->sync, 0, sizeof(ue->sync));
  pthread_mutex_unlock(&ue->sync_mtx);
}

int nr_slot_contains_ssb(const nr_sync_state_t *sync)
{
  if (!sync) {
    return 0;
  }
  return sync->locked ? 1 : 0;
}
