cmake_minimum_required(VERSION 3.8)
project(calibration)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


set(CMAKE_CXX_STANDARD 17)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CALIB_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include)



#add_subdirectory(src)


set(BUILD_SAMPLES ON CACHE BOOL "Build samples")
if(BUILD_SAMPLES)
  add_subdirectory(samples)
endif()
