#pragma once

#include <stdint.h>

#define NR_FPGA_SSB_REG_MAGIC              0x0000u
#define NR_FPGA_SSB_REG_VERSION            0x0004u
#define NR_FPGA_SSB_REG_CONTROL            0x0008u
#define NR_FPGA_SSB_REG_STATUS             0x000cu
#define NR_FPGA_SSB_REG_IQ_BASE_LO         0x0010u
#define NR_FPGA_SSB_REG_IQ_BASE_HI         0x0014u
#define NR_FPGA_SSB_REG_IQ_NSAMPS          0x0018u
#define NR_FPGA_SSB_REG_FS_HZ_Q16          0x001cu
#define NR_FPGA_SSB_REG_TARGET_PCI         0x0020u
#define NR_FPGA_SSB_REG_RESULT_COARSE      0x0040u
#define NR_FPGA_SSB_REG_RESULT_FRAC_Q16    0x0044u
#define NR_FPGA_SSB_REG_RESULT_PCI         0x0048u
#define NR_FPGA_SSB_REG_RESULT_CFO_Q8      0x004cu
#define NR_FPGA_SSB_REG_RESULT_METRIC_Q16  0x0050u

#define NR_FPGA_SSB_MAGIC                  0x53534254u
#define NR_FPGA_SSB_VERSION                0x00010000u

#define NR_FPGA_SSB_CONTROL_START          (1u << 0)
#define NR_FPGA_SSB_CONTROL_IRQ_ENABLE     (1u << 1)
#define NR_FPGA_SSB_CONTROL_SOFT_RESET     (1u << 31)

#define NR_FPGA_SSB_STATUS_IDLE            (1u << 0)
#define NR_FPGA_SSB_STATUS_BUSY            (1u << 1)
#define NR_FPGA_SSB_STATUS_DONE            (1u << 2)
#define NR_FPGA_SSB_STATUS_ERROR           (1u << 3)
#define NR_FPGA_SSB_STATUS_RESULT_VALID    (1u << 4)

typedef struct {
  uint32_t coarse_offset_samp;
  int32_t frac_offset_q16;
  uint16_t pci;
  int32_t cfo_hz_q8;
  uint32_t metric_q16;
  uint32_t status;
} nr_fpga_ssb_result_t;
