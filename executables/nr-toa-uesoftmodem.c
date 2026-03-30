#include "executables/nr-toa-softmodem.h"
#include "executables/nr-toa-threads.h"
#include "openair1/PHY/NR_POSITIONING/nr_pos_api.h"

#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

volatile int oai_exit;

static void *TOA_UE_thread(void *arg)
{
  PHY_VARS_NR_TOA_UE *UE = (PHY_VARS_NR_TOA_UE *)arg;

  while (!oai_exit) {

    switch (UE->state) {

    case TOA_STATE_WAIT_CLOCK:
      if (nr_toa_clock_ready(UE) == 1) {
        UE->state = TOA_STATE_PRESYNC;
      }
      break;

    case TOA_STATE_PRESYNC:
      nr_toa_read_two_frames(UE, &UE->iq_ring);
      nr_toa_enqueue_sync_job(UE);
      UE->state = TOA_STATE_LOCKED;
      break;

    case TOA_STATE_LOCKED:
    case TOA_STATE_MEASURING:
      nr_toa_read_one_slot(UE, &UE->iq_ring);
      if (nr_slot_contains_ssb(&UE->sync)) {
        nr_toa_enqueue_measure_job(UE);
      }
      if (UE->sync.locked == 0) {
        UE->state = TOA_STATE_RESYNC;
      }
      break;

    case TOA_STATE_RESYNC:
      nr_toa_reset_tracking(UE);
      UE->state = TOA_STATE_PRESYNC;
      break;

    default:
      break;
    }
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
  ue.provider = &nr_ssb_provider;

  const char *cfgpath =
      (argc > 1) ? argv[1]
                 : "targets/PROJECTS/NR-TOA/CONF/ue.toa.ssb.limesdr.conf";
  if (nr_toa_load_config(cfgpath, &ue.app_cfg) != 0) {
    return 1;
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
  if (ue.provider->init((void *)&ue) != 0) {
    return 1;
  }

  pthread_t tid;
  if (pthread_create(&tid, NULL, TOA_UE_thread, &ue) != 0) {
    return 1;
  }
  sleep(1);
  oai_exit = 1;
  pthread_join(tid, NULL);
  return 0;
}
