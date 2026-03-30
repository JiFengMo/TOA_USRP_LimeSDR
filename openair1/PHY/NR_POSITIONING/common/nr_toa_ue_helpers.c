#include "openair1/PHY/NR_POSITIONING/nr_pos_types.h"

int nr_toa_clock_ready(void *ue)
{
  (void)ue;
  return 1;
}

int nr_toa_read_two_frames(void *ue, nr_iq_ring_t *ring)
{
  (void)ue;
  (void)ring;
  return 0;
}

int nr_toa_read_one_slot(void *ue, nr_iq_ring_t *ring)
{
  (void)ue;
  (void)ring;
  return 0;
}

void nr_toa_enqueue_sync_job(void *ue)
{
  (void)ue;
}

void nr_toa_enqueue_measure_job(void *ue)
{
  (void)ue;
}

void nr_toa_reset_tracking(void *ue)
{
  (void)ue;
}

int nr_slot_contains_ssb(const nr_sync_state_t *sync)
{
  if (!sync) {
    return 0;
  }
  return sync->locked ? 1 : 0;
}
