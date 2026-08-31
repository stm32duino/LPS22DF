# v3.21 implemented semantic changes regarding $<TARGET_OBJECTS:...>
# See https://cmake.org/cmake/help/v3.21/command/target_link_libraries.html#linking-object-libraries-via-target-objects
cmake_minimum_required(VERSION 3.21)

add_library(LPS22DF INTERFACE)
add_library(LPS22DF_usage INTERFACE)

target_include_directories(LPS22DF_usage INTERFACE
  src
)


target_link_libraries(LPS22DF_usage INTERFACE
  base_config
)

target_link_libraries(LPS22DF INTERFACE LPS22DF_usage)



add_library(LPS22DF_bin OBJECT EXCLUDE_FROM_ALL
  src/lps22df_reg.c
  src/LPS22DFSensor.cpp
)
target_link_libraries(LPS22DF_bin PUBLIC LPS22DF_usage)

target_link_libraries(LPS22DF INTERFACE
  LPS22DF_bin
  $<TARGET_OBJECTS:LPS22DF_bin>
)

