#ifndef RADIO_DEVICE_H
#define RADIO_DEVICE_H

#include <stdint.h>
#include <stddef.h>

/* 设备类型 */
typedef enum {
  RADIO_DEVICE_NONE = 0,
  RADIO_DEVICE_LIME = 1,
  RADIO_DEVICE_USRP = 2
} radio_device_type_t;

/* 复数 float IQ */
typedef struct {
  float r;
  float i;
} cf32_t;

/* 单通道配置 */
typedef struct {
  uint32_t channel_id;   /* 通道号 */
  double sample_rate;    /* 采样率 */
  double center_freq;    /* 中心频率 */
  double bandwidth;      /* 带宽 */
  double gain;           /* 增益 */
  char antenna[32];      /* 天线名 */
} radio_channel_config_t;

/* 流配置 */
typedef struct {
  uint32_t channel_id;   /* 通道号 */
  uint32_t buffer_size;  /* 一次读写的样点数 */
} radio_stream_config_t;

/* 接收结果 */
typedef struct {
  int samples_read;      /* 实际读到的样点数 */
  int flags;             /* 底层标志 */
  int64_t timestamp_ns;  /* 硬件时间戳 */
} radio_rx_result_t;

/* 前向声明 */
typedef struct radio_device_s radio_device_t;

/*
 * 通用设备对象
 * 风格尽量贴近 OAI：一个总设备结构体 + 一组函数指针
 */
struct radio_device_s {
  radio_device_type_t type;   /* 设备类型 */
  char name[32];              /* 设备名 */

  int is_open;                /* 是否已经打开 */
  int rx_stream_open;         /* RX流是否创建 */
  int tx_stream_open;         /* TX流是否创建 */
  int rx_running;             /* RX流是否激活 */
  int tx_running;             /* TX流是否激活 */

  radio_channel_config_t rx_cfg;        /* RX配置缓存 */
  radio_channel_config_t tx_cfg;        /* TX配置缓存 */
  radio_stream_config_t rx_stream_cfg;  /* RX流配置缓存 */
  radio_stream_config_t tx_stream_cfg;  /* TX流配置缓存 */

  void *priv;                 /* 底层私有指针 */

  /* 设备生命周期 */
  int  (*dev_open)(radio_device_t *device, const char *args);
  void (*dev_close)(radio_device_t *device);

  /* 配置接口 */
  int  (*trx_set_rx_gain)(radio_device_t *device, uint32_t channel, double gain);
  int  (*trx_set_tx_gain)(radio_device_t *device, uint32_t channel, double gain);

  int  (*trx_configure_rx)(radio_device_t *device, const radio_channel_config_t *cfg);
  int  (*trx_configure_tx)(radio_device_t *device, const radio_channel_config_t *cfg);

  /* 流控制 */
  int  (*trx_setup_rx_stream)(radio_device_t *device, const radio_stream_config_t *cfg);
  int  (*trx_setup_tx_stream)(radio_device_t *device, const radio_stream_config_t *cfg);

  int  (*trx_start_rx_stream)(radio_device_t *device);
  int  (*trx_stop_rx_stream)(radio_device_t *device);

  int  (*trx_start_tx_stream)(radio_device_t *device);
  int  (*trx_stop_tx_stream)(radio_device_t *device);

  /* 数据收发 */
  int  (*trx_read_func)(radio_device_t *device,
                        void **buffs,
                        uint32_t nsamps,
                        radio_rx_result_t *result,
                        int timeout_us);

  int  (*trx_write_func)(radio_device_t *device,
                         const void **buffs,
                         uint32_t nsamps,
                         int64_t timestamp_ns,
                         int flags,
                         int timeout_us);

  /* 硬件时间 */
  int64_t (*get_hw_time_ns)(radio_device_t *device);
  int     (*set_hw_time_ns)(radio_device_t *device, int64_t time_ns);
};

#endif