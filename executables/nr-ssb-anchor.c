#include "executables/nr-toa-threads.h"
#include "openair1/PHY/NR_POSITIONING/nr_pos_api.h"
#include "radio/COMMON/common_lib.h"

#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

volatile int oai_exit;

static void *ANCHOR_main_thread(void *arg)
{
  (void)arg;
  while (!oai_exit) {
    sleep(1);
  }
  return NULL;
}

int main(int argc, char **argv)
{
  oai_exit = 0;
  nr_toa_app_cfg_t app;
  openair0_config_t rf;
  memset(&app, 0, sizeof(app));
  if (nr_toa_load_config(
          (argc > 1) ? argv[1]
                     : "targets/PROJECTS/NR-TOA/CONF/anchor0.ssb.usrpb210.conf",
          &app) != 0) {
    return 1;
  }
  if (nr_toa_build_rf_cfg(&app, &rf) != 0) {
    return 1;
  }

  nr_ssb_tx_plan_t plan;
  if (nr_ssb_build_tx_plan(&app, &plan) != 0) {
    return 1;
  }

  openair0_device_t *dev = NULL;
  if (nr_toa_radio_init(&dev, &rf) != 0) {
    return 1;
  }
  if (nr_toa_radio_start(dev) != 0) {
    return 1;
  }

  (void)TOA_THREAD_ANCHOR_MAIN;
  (void)plan;
  pthread_t tid;
  if (pthread_create(&tid, NULL, ANCHOR_main_thread, NULL) != 0) {
    return 1;
  }
  sleep(1);
  oai_exit = 1;
  pthread_join(tid, NULL);
  return 0;
}
