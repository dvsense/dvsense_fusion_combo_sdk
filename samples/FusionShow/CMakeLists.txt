cmake_minimum_required(VERSION 3.10)
project(DvsFusionShow)

add_executable(FusionShow FusionShow.cpp)
target_link_libraries(FusionShow 
PRIVATE
    DvsRgbFusionCamera
    CalibrateThroughFile
    ${OpenCV_LIBS}
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)