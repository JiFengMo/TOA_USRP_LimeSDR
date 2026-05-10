#include "openair1/PHY/NR_POSITIONING/nr_pos_types.h"
#include "openair1/PHY/NR_POSITIONING/nr_toa_ue.h"
#include "openair1/PHY/NR_POSITIONING/nr_pos_api.h"

#include <stdio.h>
#include <string.h>

static void nr_toa_note_rx_block(nr_toa_ue_t *ue, const nr_iq_block_t *blk,
                                 const char *tag)
{
  if (!ue || !blk) {
    return;
  }

  if (ue->rx_read_count > 0U && blk->abs_samp0 != ue->rx_next_abs_samp) {
    const int64_t delta = (int64_t)blk->abs_samp0 - (int64_t)ue->rx_next_abs_samp;
    ue->rx_gap_count++;
    printf("rx_iq_continuity: %s discontinuity=%lld expected_abs=%llu got_abs=%llu nsamps=%u gaps=%llu reads=%llu\n",
           tag ? tag : "rx",
           (long long)delta,
           (unsigned long long)ue->rx_next_abs_samp,
           (unsigned long long)blk->abs_samp0,
           blk->nsamps,
           (unsigned long long)ue->rx_gap_count,
           (unsigned long long)ue->rx_read_count);
  } else if ((ue->rx_read_count % 100U) == 0U) {
    printf("rx_iq_continuity: %s ok abs=%llu nsamps=%u next=%llu gaps=%llu reads=%llu\n",
           tag ? tag : "rx",
           (unsigned long long)blk->abs_samp0,
           blk->nsamps,
           (unsigned long long)(blk->abs_samp0 + blk->nsamps),
           (unsigned long long)ue->rx_gap_count,
           (unsigned long long)ue->rx_read_count);
  }

  ue->rx_read_count++;
  ue->rx_next_abs_samp = blk->abs_samp0 + blk->nsamps;
}

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

  nr_iq_block_t *blk = nr_iq_ring_alloc_ex(ring, nsamps, rx_ant);
  if (!blk) {
    return -1;
  }

  /* Keep IQ processing in sync with the true RF sample rate. */
  blk->fs_hz = ue->rf_cfg.sample_rate;
  blk->rx_freq_hz = ue->current_rx_freq_hz;
  blk->rx_gscn = ue->current_rx_gscn;
  blk->target_pci = ue->current_target_pci;
  blk->scan_target_idx = ue->current_scan_target_idx;

  if (nr_toa_radio_read(ue->dev, blk) != 0) {
    nr_iq_block_put(blk);
    return -1;
  }
  nr_toa_note_rx_block(ue, blk, "presync");

  if (ue->rf_settle_reads > 0U) {
    ue->rf_settle_reads--;
    nr_iq_block_put(blk);
    return 0;
  }

  nr_iq_ring_push(ring, blk);
  ue->abs_samp_wr = blk->abs_samp0 + blk->nsamps;

  pthread_mutex_lock(&ue->sync_mtx);
  const int sync_busy = (ue->sync_q_count > 0U || ue->sync_job_blk != NULL);
  pthread_mutex_unlock(&ue->sync_mtx);

  if (!sync_busy) {
    /* One capture spans one SSB period plus the SSB burst, so it contains at
     * least one complete SSB without doubling the RX stall and search cost. */
    (void)nr_toa_enqueue_sync_job(ue, blk);
  }
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
  blk->rx_freq_hz = ue->current_rx_freq_hz;
  blk->rx_gscn = ue->current_rx_gscn;
  blk->target_pci = ue->current_target_pci;
  blk->scan_target_idx = ue->current_scan_target_idx;

  if (nr_toa_radio_read(ue->dev, blk) != 0) {
    nr_iq_block_put(blk);
    return -1;
  }
  nr_toa_note_rx_block(ue, blk, "locked");

  if (ue->rf_settle_reads > 0U) {
    ue->rf_settle_reads--;
    nr_iq_block_put(blk);
    return 0;
  }

  nr_iq_ring_push(ring, blk);
  ue->abs_samp_wr = blk->abs_samp0 + blk->nsamps;

  pthread_mutex_lock(&ue->sync_mtx);
  const int sync_busy = (ue->sync_q_count > 0U || ue->sync_job_blk != NULL);
  pthread_mutex_unlock(&ue->sync_mtx);

  if (!sync_busy) {
    (void)nr_toa_enqueue_sync_job(ue, blk);
  }
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
