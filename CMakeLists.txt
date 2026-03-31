cmake_minimum_required(VERSION 3.16)
project(NR_TOA_USRP_LimeSDR LANGUAGES C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-Wall -Wextra -Wpedantic)

include_directories(${CMAKE_SOURCE_DIR})
find_package(PkgConfig REQUIRED)
pkg_check_modules(UHD REQUIRED uhd)

add_library(nr_radio STATIC
  radio/USRP/usrp_lib.cpp
  radio/LIME/lime_lib.cpp
)
target_include_directories(nr_radio PUBLIC ${UHD_INCLUDE_DIRS})
target_link_directories(nr_radio PUBLIC ${UHD_LIBRARY_DIRS})
target_link_libraries(nr_radio PUBLIC ${UHD_LIBRARIES})

set(NR_POSITIONING_SOURCES
  openair1/PHY/NR_POSITIONING/common/nr_iq_ring.c
  openair1/PHY/NR_POSITIONING/common/nr_epoch_mgr.c
  openair1/PHY/NR_POSITIONING/common/nr_anchor_db.c
  openair1/PHY/NR_POSITIONING/common/nr_trace_sink.c
  openair1/PHY/NR_POSITIONING/common/nr_pos_solver.c
  openair1/PHY/NR_POSITIONING/common/nr_toa_cfg_loader.c
  openair1/PHY/NR_POSITIONING/common/nr_radio_if.c
  openair1/PHY/NR_POSITIONING/common/nr_clock_mgr.c
  openair1/PHY/NR_POSITIONING/common/nr_ssb_tx_scheduler.c
  openair1/PHY/NR_POSITIONING/common/nr_ssb_beacon_gen.c
  openair1/PHY/NR_POSITIONING/common/nr_anchor_write.c
  openair1/PHY/NR_POSITIONING/common/nr_meas_assoc.c
  openair1/PHY/NR_POSITIONING/common/nr_toa_ue_helpers.c
  openair1/PHY/NR_POSITIONING/ssb/nr_pbch.c
  openair1/PHY/NR_POSITIONING/ssb/nr_ssb_ref.c
  openair1/PHY/NR_POSITIONING/ssb/nr_ssb_sync.c
  openair1/PHY/NR_POSITIONING/ssb/nr_ssb_extract.c
  openair1/PHY/NR_POSITIONING/ssb/nr_ssb_chest.c
  openair1/PHY/NR_POSITIONING/ssb/nr_ssb_toa_est.c
  openair1/PHY/NR_POSITIONING/ssb/nr_ssb_provider.c
  openair1/PHY/NR_POSITIONING/prs/nr_prs_extract.c
  openair1/PHY/NR_POSITIONING/prs/nr_prs_chest.c
  openair1/PHY/NR_POSITIONING/prs/nr_prs_toa_est.c
  openair1/PHY/NR_POSITIONING/prs/nr_prs_provider.c
)

add_library(nr_positioning STATIC ${NR_POSITIONING_SOURCES})
target_link_libraries(nr_positioning PUBLIC nr_radio m pthread)

add_executable(nr-toa-uesoftmodem executables/nr-toa-uesoftmodem.c)
target_link_libraries(nr-toa-uesoftmodem PRIVATE nr_positioning)

add_executable(nr-ssb-anchor executables/nr-ssb-anchor.c)
target_link_libraries(nr-ssb-anchor PRIVATE nr_positioning)

add_executable(replay_toa_iq tools/replay_toa_iq.c)
target_link_libraries(replay_toa_iq PRIVATE nr_positioning)

add_executable(test_ssb_sync openair1/PHY/NR_POSITIONING/tests/test_ssb_sync.c)
target_link_libraries(test_ssb_sync PRIVATE nr_positioning)

add_executable(test_ssb_toa openair1/PHY/NR_POSITIONING/tests/test_ssb_toa.c)
target_link_libraries(test_ssb_toa PRIVATE nr_positioning)

add_executable(test_prs_toa openair1/PHY/NR_POSITIONING/tests/test_prs_toa.c)
target_link_libraries(test_prs_toa PRIVATE nr_positioning)

add_executable(test_solver openair1/PHY/NR_POSITIONING/tests/test_solver.c)
target_link_libraries(test_solver PRIVATE nr_positioning)
