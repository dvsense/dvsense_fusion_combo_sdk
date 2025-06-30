cmake_minimum_required(VERSION 3.10)
project(camera_combo)

set(CMAKE_CXX_STANDARD_REQUIRED True)

set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build.")
set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "RelWithDebInfo" "MinSizeRel")
set(CAMERA_MANAGER_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)

message(STATUS "CAMERA_MANAGER_INCLUDE_DIR: ${CAMERA_MANAGER_INCLUDE_DIR}")

#设置dll生成路径
#set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/samples)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/samples)
#set(CMAKE_BUILD_RPATH ${CMAKE_BINARY_DIR}/samples/camera_manager)

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    # 设置Debug构建类型的编译选项
    add_definitions(-DDEBUG)
    if(NOT WIN32)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0 -g")
    endif()
elseif(CMAKE_BUILD_TYPE STREQUAL "Release")
    # 设置Release构建类型的编译选项
    if(NOT WIN32)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -DNDEBUG")
    endif()
elseif(CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo")
    # 设置RelWithDebInfo构建类型的编译选项
    if(NOT WIN32)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -g")
    endif()
elseif(CMAKE_BUILD_TYPE STREQUAL "MinSizeRel")
    # 设置MinSizeRel构建类型的编译选项
    if(NOT WIN32)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Os")
    endif()
endif()

add_library(camera_combo_manager SHARED
    camera_combo_manager.cpp
    dvs_visualizer.cpp
  "dvs_process.cpp")
find_package(OpenCV REQUIRED)
# 打印 OpenCV 信息（用于调试）
message(STATUS "OpenCV Version: ${OpenCV_VERSION}")
message(STATUS "OpenCV Include Dir: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "OpenCV Libraries: ${OpenCV_LIBS}")
target_include_directories(camera_combo_manager PUBLIC
    ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(camera_combo_manager
PUBLIC
    ${OpenCV_LIBS}
)

find_package(Boost COMPONENTS filesystem REQUIRED)
# find_library(MVS_LIB NAMES MvCameraControl PATHS ls /opt/MVS/lib/64)
find_library(MVS_LIB NAMES MvCameraControl PATHS "C:/Program Files (x86)/MVS/Development/Libraries/win64/")
find_package(DvsenseDriver COMPONENTS Driver REQUIRED)
# set(MVS_LIB_DIR D:/Software/MVS/Development/Libraries/win64)
# set(MVS_INCLUDE_DIR /opt/MVS/include/)
set(MVS_INCLUDE_DIR C:/Program Files (x86)/MVS/Development/Includes)

find_package(MetavisionSDK REQUIRED COMPONENTS 
    core
    driver
    ui
)

include_directories(
    ${MVS_INCLUDE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CAMERA_MANAGER_DIR}
)

target_link_libraries(camera_combo_manager 
PUBLIC
    ${MVS_LIB}
    opencv_core
    opencv_imgproc
    MetavisionSDK::core
    MetavisionSDK::driver
    ${Boost_LIBRARIES}
    DvsenseDriver::Driver
)

target_include_directories(camera_combo_manager
PRIVATE
    ${CAMERA_MANAGER_INCLUDE_DIR}
PUBLIC
    ${MVS_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
)
