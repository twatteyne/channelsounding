# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#
# Unit tests of ../lib/cs_ranging.c with the Nordic cs_de library (native_sim).

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(cs_ranging_test)

test_runner_generate(src/test_cs_ranging.c)
target_sources(app PRIVATE
  src/test_cs_ranging.c
  ${CMAKE_CURRENT_SOURCE_DIR}/../lib/cs_ranging.c
)
target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../lib)
