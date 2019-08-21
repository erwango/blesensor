# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.13.1)
set(SHIELD x_nucleo_iks01a1)
include($ENV{ZEPHYR_BASE}/cmake/app/boilerplate.cmake NO_POLICY_SCOPE)
project(blesensor)

target_sources(app PRIVATE
  src/main.c
  src/environmental.c
  src/quaternions.c
  src/command.c
  /home/stuserlogin/stage/zephyrproject/zephyr/samples/bluetooth/gatt/hrs.c
  /home/stuserlogin/stage/zephyrproject/zephyr/samples/bluetooth/gatt/bas.c
  /home/stuserlogin/stage/zephyrproject/zephyr/samples/bluetooth/gatt/cts.c
)

zephyr_library_include_directories($ENV{ZEPHYR_BASE}/samples/bluetooth)
