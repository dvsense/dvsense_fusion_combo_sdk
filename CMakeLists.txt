cmake_minimum_required(VERSION 3.8)
project(fusion_camera)

if(WIN32)
    cmake_policy(SET CMP0091 NEW)
    set(CMAKE_USE_WIN32_MSVC_LONG_PATHS ON)
    set(MVS_LIB_SEARCH_PATHS "C:/Program Files (x86)/MVS/Development/Libraries/win64/" CACHE PATH "MVS root dir")
    set(MVS_INCLUDE_DIR "C:/Program Files (x86)/MVS/Development/Includes" CACHE PATH "MVS include dir")
else()
    set(MVS_LIB_SEARCH_PATHS /opt/MVS/lib/64 CACHE PATH "MVS root dir")
    set(MVS_INCLUDE_DIR /opt/MVS/include/ CACHE PATH "MVS include dir")
endif()
find_library(MVS_LIB NAMES MvCameraControl PATHS ${MVS_LIB_SEARCH_PATHS})
if(NOT MVS_LIB)
    message(FATAL_ERROR "MvCameraControl library not found")
endif()

set(CALIB_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build.")
set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "RelWithDebInfo" "MinSizeRel")
set(CAMERA_MANAGER_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include/dvs
                               ${CMAKE_CURRENT_SOURCE_DIR}/rgb)

message(STATUS "CAMERA_MANAGER_INCLUDE_DIR: ${CAMERA_MANAGER_INCLUDE_DIR}")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/samples)

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    # 设置Debug构建类型的编译选项
    #add_definitions(-DDEBUG)
    #target_compile_definitions(camera_combo PRIVATE DEBUG)
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

# find_package(FFmpeg REQUIRED)
find_library(AVCODEC_LIB 
    NAMES avcodec libavcodec
    HINTS ${FFMPEG_DIR}/lib
    REQUIRED
)
find_library(AVFORMAT_LIB 
    NAMES avformat libavformat
    HINTS ${FFMPEG_DIR}/lib
    REQUIRED
)
find_library(AVUTIL_LIB 
    NAMES avutil libavutil
    HINTS ${FFMPEG_DIR}/lib
    REQUIRED
)
find_library(SWSCALE_LIB 
    NAMES swscale libswscale
    HINTS ${FFMPEG_DIR}/lib
    REQUIRED
)

add_library(DvsRgbFusionCamera SHARED
    "src/camera_manager/DvsRgbFusionCamera.cpp"
    "src/dvs/DvsEventCamera.cpp"
    "src/rgb/hik/HikCamera.cpp"
    "src/rgb/RgbCamera.cpp"
    "src/rgb/ApsTypes.cpp"
    "src/camera_manager/DataToVideo.cpp" )

find_package(OpenCV REQUIRED)
find_package(Boost COMPONENTS filesystem REQUIRED)
find_package(DvsenseDriver COMPONENTS Driver REQUIRED)


target_include_directories(DvsRgbFusionCamera
    PUBLIC
    ${FFMPEG_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include/dvs
    ${CMAKE_CURRENT_SOURCE_DIR}/include/rgb
    ${CMAKE_CURRENT_SOURCE_DIR}/include/camera_manager
    ${MVS_INCLUDE_DIR}
)

target_link_libraries(DvsRgbFusionCamera 
PUBLIC
    ${MVS_LIB}
    opencv_core
    opencv_imgproc
    ${Boost_LIBRARIES}
    DvsenseDriver::Driver
    ${AVCODEC_LIB}
    ${AVFORMAT_LIB}
    ${AVUTIL_LIB}
    ${SWSCALE_LIB}
)

option(BUILD_SAMPLES "Build samples" OFF)
if (BUILD_SAMPLES)
    add_subdirectory(samples/FusionShow)
    add_subdirectory(samples/FileReader)
endif()

option(BUILD_DOCS "Build documentation" OFF)
if(BUILD_DOCS)
    find_package(Doxygen REQUIRED)
    set(DOXYGEN_IN ${CMAKE_CURRENT_LIST_DIR}/docs/Doxyfile.in)
    set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)

    configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)

    add_custom_target( docs ALL
        COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMENT "Generating API documentation with Doxygen"
        VERBATIM )
endif()

