cmake_minimum_required(VERSION 3.10)
project(mc_rtc_controller)

find_package(mc_rtc REQUIRED)

add_controller(
  "G1CoMPosture"
  "${CMAKE_CURRENT_SOURCE_DIR}/mc_com_posture_controller.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/mc_com_posture_controller.h"
)
