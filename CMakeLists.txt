cmake_minimum_required(VERSION 3.16)
project(TOA_USRP_LimeSDR C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

add_compile_options(
  -Wall
  -Wextra
  -Wpedantic
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(SOAPYSDR REQUIRED SoapySDR)

include_directories(
  ${CMAKE_SOURCE_DIR}/include
  ${CMAKE_SOURCE_DIR}/include/radio
  ${CMAKE_SOURCE_DIR}/include/recorder
  ${CMAKE_SOURCE_DIR}/include/toa
  ${CMAKE_SOURCE_DIR}/include/nr
  ${CMAKE_SOURCE_DIR}/include/common
  ${SOAPYSDR_INCLUDE_DIRS}
)

link_directories(
  ${SOAPYSDR_LIBRARY_DIRS}
)

# IO + offline pipeline（元数据 / 读块 / 录制 session / 离线�?�?�?
add_library(toa_io STATIC
  src/io/iq_meta.c
  src/io/iq_reader.c
  src/io/capture_session.c
  src/recorder/iq_recorder.c
)

add_library(toa_pipeline STATIC
  src/pipeline/offline_runner.c
)

target_link_libraries(toa_pipeline PUBLIC toa_io)

set(COMMON_CLI_SRC src/common/cli.c)

# IQ 录制程序（兼�? Lime RX 连通性验证；�? test_lime_rx 已合并删除）
add_executable(record_lime_iq
  demo/record_lime_iq.c
  src/radio/radio_factory.c
  src/radio/radio_lime.c
  src/common/log.c
  ${COMMON_CLI_SRC}
)

target_link_libraries(record_lime_iq
  nr_core
  toa_io
  m
  ${SOAPYSDR_LIBRARIES}
)

target_compile_options(record_lime_iq PRIVATE ${SOAPYSDR_CFLAGS_OTHER})

# �? NR GSCN（FR1 <3 GHz 同�?�栅格）步进�?心�?�，每点�?�? IQ 录制；可�?�?多轮
add_executable(record_lime_gscn_scan
  demo/record_lime_gscn_scan.c
  src/radio/radio_factory.c
  src/radio/radio_lime.c
  src/common/log.c
  ${COMMON_CLI_SRC}
)

target_link_libraries(record_lime_gscn_scan
  nr_core
  toa_io
  m
  ${SOAPYSDR_LIBRARIES}
)

target_compile_options(record_lime_gscn_scan PRIVATE ${SOAPYSDR_CFLAGS_OTHER})

# radio 生命周期�?检（phase0�?
add_executable(test_radio_lifecycle
  demo/test_radio_lifecycle.c
  src/radio/radio_factory.c
  src/radio/radio_lime.c
  src/common/log.c
)

target_link_libraries(test_radio_lifecycle
  ${SOAPYSDR_LIBRARIES}
)

target_compile_options(test_radio_lifecycle PRIVATE ${SOAPYSDR_CFLAGS_OTHER})

# NR 模块（SSB�?描起步）
add_library(nr_core
  src/nr/gscn.c
  src/nr/nr_sequence.c
  src/nr/ssb_sync.c
  src/nr/nr_fft.c
  src/nr/nr_grid.c
  src/nr/pbch_dmrs.c
  src/nr/pbch_re.c
  src/nr/pbch_decode.c
  src/nr/polar.c
  src/nr/mib.c
)

# nr_core is pure C signal processing; no Soapy compile flags needed.

# TOA 模块基�?�库（先提供重构�?�架�?
add_library(toa_core
  src/toa/toa_context.c
  src/toa/prs_detector.c
  src/toa/toa_pipeline.c
  src/toa/toa_estimator.c
  src/toa/toa_solver_wls.c
  src/common/ring_buffer.c
  src/common/log.c
)

# toa_core does not use Soapy.

# 离线 PRS/TOA 框架程序
add_executable(toa_prs_offline
  demo/toa_prs_offline.c
  ${COMMON_CLI_SRC}
)

target_link_libraries(toa_prs_offline
  toa_core
  toa_pipeline
  toa_io
  m
)

# 近实�? TOA 框架程序
add_executable(toa_run_realtime
  demo/toa_run_realtime.c
  src/radio/radio_factory.c
  src/radio/radio_lime.c
)

target_link_libraries(toa_run_realtime
  toa_core
  m
  ${SOAPYSDR_LIBRARIES}
)

target_compile_options(toa_run_realtime PRIVATE ${SOAPYSDR_CFLAGS_OTHER})

# 离线 SSB �?描程序（Phase B�?
add_executable(nr_ssb_scan
  demo/nr_ssb_scan.c
  src/common/log.c
  ${COMMON_CLI_SRC}
)

target_link_libraries(nr_ssb_scan
  nr_core
  toa_pipeline
  toa_io
  m
)

add_executable(nr_ssb_live_gscn
  demo/nr_ssb_live_gscn.c
  src/radio/radio_factory.c
  src/radio/radio_lime.c
  src/common/log.c
  ${COMMON_CLI_SRC}
)

target_link_libraries(nr_ssb_live_gscn
  nr_core
  m
  ${SOAPYSDR_LIBRARIES}
)

target_compile_options(nr_ssb_live_gscn PRIVATE ${SOAPYSDR_CFLAGS_OTHER})

# MIB 解码链（�? SSB + DMRS 对齐；原 nr_pbch_dmrs_probe 已合并删除）
add_executable(nr_mib_decode
  demo/nr_mib_decode.c
  src/common/log.c
  ${COMMON_CLI_SRC}
)

target_link_libraries(nr_mib_decode
  nr_core
  toa_pipeline
  toa_io
  m
)

add_executable(nr_ssb_selftest demo/nr_ssb_selftest.c)
target_link_libraries(nr_ssb_selftest nr_core m)

add_executable(nr_synth_ssb demo/nr_synth_ssb.c)
target_link_libraries(nr_synth_ssb nr_core toa_io m)