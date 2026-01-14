cmake_minimum_required(VERSION 3.10)
project(DvsFileCut)

find_package(DvsRgbFusionCamera REQUIRED)
find_package(DvsRgbCalib REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(DvsFileCut FileCut.cpp)
target_link_libraries(DvsFileCut PRIVATE     
    DvsRgbFusionCamera::DvsRgbFusionCamera 
    DvsRgbCalib::DvsRgbCalib
    opencv_videoio
    opencv_imgproc
    opencv_highgui
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)