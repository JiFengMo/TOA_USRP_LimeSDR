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
  ${SOAPYSDR_INCLUDE_DIRS}
)

link_directories(
  ${SOAPYSDR_LIBRARY_DIRS}
)

# 先保留原来的测试程序
add_executable(test_lime_rx
  demo/test_lime_rx.c
  src/radio/radio_factory.c
  src/radio/radio_lime.c
)

target_link_libraries(test_lime_rx
  ${SOAPYSDR_LIBRARIES}
)

target_compile_options(test_lime_rx PRIVATE ${SOAPYSDR_CFLAGS_OTHER})

# 新增 IQ 录制程序
add_executable(record_lime_iq
  demo/record_lime_iq.c
  src/radio/radio_factory.c
  src/radio/radio_lime.c
  src/recorder/iq_recorder.c
)

target_link_libraries(record_lime_iq
  ${SOAPYSDR_LIBRARIES}
)

target_compile_options(record_lime_iq PRIVATE ${SOAPYSDR_CFLAGS_OTHER})