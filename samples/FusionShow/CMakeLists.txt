cmake_minimum_required(VERSION 3.10)
project(DvsFusionShow)

# Allow building this sample against an installed package
find_package(DvsRgbFusionCamera REQUIRED)

add_executable(FusionShow FusionShow.cpp)
target_link_libraries(FusionShow PRIVATE DvsRgbFusionCamera::DvsRgbFusionCamera)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)