# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.22)

set(BOARD_ROOT ${CMAKE_SOURCE_DIR})
set(BOARD "esp32_lolin_oled")
set(DTC_OVERLAY_FILE "esp32_lolin_oled.overlay")

find_package(Zephyr)
project(ze-clock)

nanopb_generate_cpp(proto_sources proto_headers RELPATH . src/simple.proto)
zephyr_library_include_directories(${CMAKE_CURRENT_BINARY_DIR})

FILE(GLOB app_sources src/*.c)
target_sources(app PRIVATE ${proto_sources} ${app_sources})

