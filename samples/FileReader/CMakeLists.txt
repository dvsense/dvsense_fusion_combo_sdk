cmake_minimum_required(VERSION 3.10)
project(DvsFileReader)

add_executable(FileReader FileReader.cpp)
target_link_libraries(FileReader 
    DvsRgbFusionCamera
    ${OpenCV_LIBS}
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)