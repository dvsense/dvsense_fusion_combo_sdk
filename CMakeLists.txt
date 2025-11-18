cmake_minimum_required(VERSION 3.8)
project(DvsenseComboSdk)

# Use standard install directories (bin, lib, include, etc.)
include(GNUInstallDirs)

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

set(FOUND_DAHENG_LIB OFF)
if(WIN32)
    set(DAHENG_LIBS "")
    set(DAHENG_INCLUDE_DIR "C:/Program Files/Daheng Imaging/GalaxySDK/Development/VC SDK/inc" CACHE PATH "Daheng camera include dir")

    # On Windows provide explicit SDK paths (find_library may fail); use forward slashes and CACHE the values
    find_library(DAHENG_GXIAPI_LIB
        NAMES GxIAPI
        PATHS
            "C:/Program Files/Daheng Imaging/GalaxySDK/Development/VC SDK/lib/x64"
    )
    if(DAHENG_GXIAPI_LIB)
        list(APPEND DAHENG_LIBS "${DAHENG_GXIAPI_LIB}")
        message(STATUS "Found Daheng GxIAPI: ${DAHENG_GXIAPI_LIB}")
        set(FOUND_DAHENG_LIB ON)
    else()
        message(WARNING "Daheng GxIAPI.lib not found")
        set(FOUND_DAHENG_LIB OFF)
    endif()

    find_library(DAHENG_DXIMAGEPROC_LIB
        NAMES DxImageProc
        PATHS
            "C:/Program Files/Daheng Imaging/GalaxySDK/Development/VC SDK/lib/x64"
    )
    if(DAHENG_DXIMAGEPROC_LIB)
        list(APPEND DAHENG_LIBS "${DAHENG_DXIMAGEPROC_LIB}")
        message(STATUS "Found Daheng DxImageProc: ${DAHENG_DXIMAGEPROC_LIB}")
        set(FOUND_DAHENG_LIB ON)
    else()
        message(WARNING "Daheng DxImageProc.lib not found")
        set(FOUND_DAHENG_LIB OFF)
    endif()

    if(FOUND_DAHENG_LIB)
        message(STATUS "Daheng include directories: ${DAHENG_INCLUDE_DIR}")
        include_directories(${DAHENG_INCLUDE_DIR})
        include_directories("C:/Program Files/Daheng Imaging/GalaxySDK/Development/VC SDK/inc")
        message(STATUS "Daheng camera library found at ${DAHENG_LIBS}")
    else()
        message(STATUS "Daheng camera library not found at ${DAHENG_LIBS}")
    endif()
else()
    set(DAHENG_LIB_SEARCH_PATHS /usr/local/Galaxy_camera/lib/x86_64/ CACHE PATH "Daheng camera lib dir")
    set(DAHENG_INCLUDE_DIR /usr/local/Galaxy_camera/inc/ CACHE PATH "Daheng camera include dir")
    find_library(DAHENG_LIB NAMES gxiapi PATHS ${DAHENG_LIB_SEARCH_PATHS})
    if(DAHENG_LIB)
        set(FOUND_DAHENG_LIB ON)
        message(STATUS "Daheng camera library found at ${DAHENG_LIB}")
    endif()
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

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/calibration_result.json DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})

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

find_package(OpenCV REQUIRED)
find_package(Boost COMPONENTS filesystem REQUIRED)
find_package(DvsenseDriver COMPONENTS Driver REQUIRED)

add_library(DvsRgbFusionCamera SHARED
    "src/CameraManager/DvsRgbFusionCamera.cpp"
    "src/dvs/DvsEventCamera.cpp"
    "src/rgb/hik/HikCamera.cpp"
    "src/rgb/RgbCamera.cpp"   
    "src/CameraManager/DataToVideo.cpp" )

target_include_directories(DvsRgbFusionCamera
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
        $<BUILD_INTERFACE:${FFMPEG_DIR}/include>
        $<BUILD_INTERFACE:${MVS_INCLUDE_DIR}>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_link_libraries(DvsRgbFusionCamera 
PUBLIC
    ${MVS_LIB}
    opencv_core
    DvsenseDriver::Driver
    ${AVCODEC_LIB}
    ${AVFORMAT_LIB}
    ${AVUTIL_LIB}
    ${SWSCALE_LIB}
)

if(FOUND_DAHENG_LIB)
    target_sources(DvsRgbFusionCamera PRIVATE "src/rgb/daheng/DahengCamera.cpp")
    target_include_directories(DvsRgbFusionCamera
    PUBLIC
        ${DAHENG_INCLUDE_DIR}
    )
    target_link_libraries(DvsRgbFusionCamera 
    PUBLIC
        ${DAHENG_LIBS}
    )
    target_compile_definitions(DvsRgbFusionCamera PRIVATE USE_DAHENG_CAMERA)
    message(STATUS "Daheng camera library found and linked.")
else()
    message(STATUS "Daheng camera library not found. Skipping Daheng camera support.")
endif()

# Provide a namespaced ALIAS for in-tree consumers matching the exported name
add_library(DvsRgbFusionCamera::DvsRgbFusionCamera ALIAS DvsRgbFusionCamera)

add_library(DvsRgbCalib SHARED
    "src/DvsRgbCalib/CalibrateThroughFile.cpp"
)

target_include_directories(DvsRgbCalib
PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/DvsRgbCalib>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/DvsRgbCalib>
)

target_link_libraries(DvsRgbCalib 
PUBLIC
    opencv_core
    opencv_imgproc
) 

# Provide a namespaced ALIAS
add_library(DvsRgbCalib::DvsRgbCalib ALIAS DvsRgbCalib)


option(BUILD_SAMPLES "Build samples" OFF)
if (BUILD_SAMPLES)
    add_subdirectory(samples/FusionShow)
    add_subdirectory(samples/FileReader)
    add_subdirectory(samples/FileCut)
    add_subdirectory(samples/ExternalTriggerShow)
endif()

option(BUILD_DOCS "Build documentation" OFF)
if(BUILD_DOCS)
    find_package(Doxygen REQUIRED)
    set(DOXYGEN_IN ${CMAKE_CURRENT_LIST_DIR}/docs/Doxyfile_zh.in)
    set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)

    configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)

    add_custom_target( docs ALL
        COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMENT "Generating API documentation with Doxygen"
        VERBATIM )
endif()

# --------------------
# Install rules
# --------------------

# Install the shared library
install(TARGETS DvsRgbFusionCamera
    EXPORT DvsRgbFusionCameraTargets
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# Install the calibration library
install(TARGETS DvsRgbCalib
    EXPORT DvsRgbCalibTargets
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# Install public headers (preserve folder structure under include/)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
        FILES_MATCHING
        PATTERN "*.h"
        PATTERN "*.hpp")

# Optionally install sample executables when built
if (BUILD_SAMPLES)
    if (TARGET FusionShow)
        install(TARGETS FusionShow RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
    endif()
    if (TARGET FileReader)
        install(TARGETS FileReader RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
    endif()
endif()

# Export targets for use by other CMake projects (basic export without full package config)
install(EXPORT DvsRgbFusionCameraTargets
        FILE DvsRgbFusionCameraTargets.cmake
        NAMESPACE DvsRgbFusionCamera::
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/DvsRgbFusionCamera)

# Export calib targets
install(EXPORT DvsRgbCalibTargets
    FILE DvsRgbCalibTargets.cmake
    NAMESPACE DvsRgbCalib::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/DvsRgbCalib)

# Generate and install a basic Config and ConfigVersion file
include(CMakePackageConfigHelpers)
set(DVSRGBFUSIONCAMERA_CONFIG_INSTALL_DIR ${CMAKE_INSTALL_LIBDIR}/cmake/DvsRgbFusionCamera)
configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/DvsRgbFusionCameraConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/DvsRgbFusionCameraConfig.cmake
    INSTALL_DESTINATION ${DVSRGBFUSIONCAMERA_CONFIG_INSTALL_DIR}
)
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/DvsRgbFusionCameraConfigVersion.cmake
    VERSION 1.0.0
    COMPATIBILITY SameMajorVersion
)
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/DvsRgbFusionCameraConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/DvsRgbFusionCameraConfigVersion.cmake
    DESTINATION ${DVSRGBFUSIONCAMERA_CONFIG_INSTALL_DIR}
)

# Config for DvsRgbCalib
set(DVSRGBCALIB_CONFIG_INSTALL_DIR ${CMAKE_INSTALL_LIBDIR}/cmake/DvsRgbCalib)
configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/DvsRgbCalibConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/DvsRgbCalibConfig.cmake
    INSTALL_DESTINATION ${DVSRGBCALIB_CONFIG_INSTALL_DIR}
)
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/DvsRgbCalibConfigVersion.cmake
    VERSION 1.0.0
    COMPATIBILITY SameMajorVersion
)
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/DvsRgbCalibConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/DvsRgbCalibConfigVersion.cmake
    DESTINATION ${DVSRGBCALIB_CONFIG_INSTALL_DIR}
)

# --------------------
# Uninstall target
# --------------------
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/cmake_uninstall.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake
    @ONLY
)

add_custom_target(uninstall
    COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake
    COMMENT "Uninstalling files from install_manifest.txt"
)
