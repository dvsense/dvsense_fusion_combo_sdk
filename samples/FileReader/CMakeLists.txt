cmake_minimum_required(VERSION 3.10)
project(DvsFileReader)

find_package(DvsRgbFusionCamera REQUIRED)
find_package(DvsRgbCalib REQUIRED)

add_executable(FileReader FileReader.cpp)
target_link_libraries(FileReader PRIVATE     
    DvsRgbFusionCamera::DvsRgbFusionCamera 
    DvsRgbCalib::DvsRgbCalib
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)