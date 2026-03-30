/**
 * OAI-style RF abstraction (minimal subset for NR-TOA positioning branch).
 * Aligns with openair0_device_t + trx_* function pointers.
 */
#pragma once

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef uint64_t openair0_timestamp_t;

typedef enum {
  MIN_DEV_TYPE = 0,
  USRP_B200_DEV = 1,
  LMSSDR_DEV = 2,
  MAX_DEV_TYPE
} openair0_device_type_t;

typedef struct openair0_config {
  openair0_device_type_t device_type;
  double sample_rate;
  double tx_freq_hz;
  double rx_freq_hz;
  double tx_gain_db;
  double rx_gain_db;
  char *sdr_addrs;
  char *clock_source;
  char *time_source;
} openair0_config_t;

struct openair0_device;

typedef int (*trx_config_func_t)(struct openair0_device *device,
                                 openair0_config_t *openair0_cfg);
typedef int (*trx_start_func_t)(struct openair0_device *device);
typedef int (*trx_stop_func_t)(struct openair0_device *device);
typedef int (*trx_end_func_t)(struct openair0_device *device,
                              struct openair0_device **device2);

/**
 * trx_read_func: buff[] is per-antenna RX buffers; ptimestamp is first sample
 * hardware time (sample domain). One complex sample = 4 bytes (int16 I + Q).
 */
typedef int (*trx_read_func_t)(struct openair0_device *device,
                               openair0_timestamp_t *ptimestamp,
                               void **buff,
                               uint32_t nsamps,
                               int antenna);

typedef int (*trx_write_func_t)(struct openair0_device *device,
                                openair0_timestamp_t *ptimestamp,
                                void **buff,
                                uint32_t nsamps,
                                int antenna,
                                int flags);

typedef struct openair0_device {
  void *priv;
  void *openair0_cfg;
  trx_config_func_t trx_config_func;
  trx_start_func_t trx_start_func;
  trx_stop_func_t trx_stop_func;
  trx_end_func_t trx_end_func;
  trx_read_func_t trx_read_func;
  trx_write_func_t trx_write_func;
} openair0_device_t;

openair0_device_t *openair0_device_get_usrp(openair0_config_t *cfg);
openair0_device_t *openair0_device_get_lime(openair0_config_t *cfg);

#ifdef __cplusplus
}
#endif
