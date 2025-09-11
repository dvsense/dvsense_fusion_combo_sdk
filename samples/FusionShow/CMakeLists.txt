cmake_minimum_required(VERSION 3.10)
project(DvsFusionShow)

# Allow building this sample against an installed package
find_package(DvsRgbFusionCamera REQUIRED)
find_package(DvsRgbCalib REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(FusionShow FusionShow.cpp)
target_link_libraries(FusionShow PRIVATE 
    DvsRgbFusionCamera::DvsRgbFusionCamera 
    DvsRgbCalib::DvsRgbCalib
    opencv_highgui
)

if(NOT EXISTS ${CMAKE_BINARY_DIR}/bin/output)
    file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/bin/output)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)